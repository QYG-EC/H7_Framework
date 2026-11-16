#include "el05_motor.h"

#include "sys_timestamp.h"

#include <cmath>

Class_EL05Motor* Class_EL05Motor::head_ = nullptr;

namespace
{
constexpr uint32_t FeedbackType = 2U << 24;
constexpr uint32_t FeedbackMask = 0x1F00FFFFU;
constexpr float PositionMax = 12.57f;
constexpr float VelocityMax = 50.0f;
constexpr float TorqueMax = 6.0f;

class Critical
{
  public:
    Critical() : state(__get_PRIMASK())
    {
        __disable_irq();
        __DMB();
    }
    ~Critical()
    {
        __DMB();
        __set_PRIMASK(state);
    }

  private:
    uint32_t state;
};

uint32_t NowMs()
{
    return static_cast<uint32_t>(SYS_Timestamp_Get_Microsecond() / 1000U);
}

uint16_t Encode(float value, float min, float max)
{
    return static_cast<uint16_t>((value - min) * 65535.0f / (max - min));
}

void Write(uint8_t* dst, uint16_t value)
{
    dst[0] = static_cast<uint8_t>(value >> 8);
    dst[1] = static_cast<uint8_t>(value);
}

uint16_t Read(const uint8_t* src)
{
    return static_cast<uint16_t>((src[0] << 8) | src[1]);
}

float Decode(const uint8_t* data, float max)
{
    return Read(data) * (2.0f * max / 65535.0f) - max;
}
} // namespace

bool Class_EL05Motor::Init(FDCAN_HandleTypeDef* bus, uint8_t motor_id, uint8_t master_id,
                           uint32_t timeout_ms)
{
    Critical lock;
    if (bus_ || !BSP_CAN_ExtendedEnabled(bus) || !timeout_ms ||
        motor_id == master_id || motor_id == 255 || master_id == 255)
        return false;
    for (auto* motor = head_; motor; motor = motor->next_)
    {
        if (motor->bus_ == bus && motor->motor_id_ == motor_id)
            return false;
    }
    const uint32_t id = FeedbackType | (static_cast<uint32_t>(motor_id) << 8) | master_id;
    if (!daemon_.SetTimeoutMs(timeout_ms) ||
        !BSP_CAN_RegisterMaskedCallback(id, FeedbackMask, CAN_ID_EXTENDED, bus, Receive, this))
        return false;

    bus_ = bus;
    motor_id_ = motor_id;
    master_id_ = master_id;
    timeout_ms_ = timeout_ms;
    next_ = head_;
    head_ = this;
    return true;
}

bool Class_EL05Motor::SubmitAction(uint32_t type)
{
    Struct_CAN_Tx_Msg frame{};
    frame.hfdcan = bus_;
    frame.id_type = CAN_ID_EXTENDED;
    frame.id = (type << 24) | (static_cast<uint32_t>(master_id_) << 8) | motor_id_;
    frame.len = 8;
    return CAN_Tx_PerformOwned(&frame, this);
}

void Class_EL05Motor::Stop()
{
    confirmed_ = false;
    stop_pending_ = !SubmitAction(4);
}

bool Class_EL05Motor::RequestEnabled(bool enabled)
{
    Critical lock;
    if (!bus_)
        return false;
    if (!enabled)
    {
        if (requested_ || stop_pending_)
            Stop();
        requested_ = false;
        locked_ = false;
        return !stop_pending_;
    }
    if (locked_ || stop_pending_)
        return false;
    if (requested_)
        return true;
    if (!SubmitAction(3))
        return false;
    requested_ = true;
    enable_started_ms_ = NowMs();
    return true;
}

bool Class_EL05Motor::Ready() const
{
    return requested_ && !locked_ && confirmed_ && daemon_.IsOnline();
}

bool Class_EL05Motor::SetMotion(float p, float v, float kp, float kd, float torque)
{
    if (!std::isfinite(p) || std::fabs(p) > PositionMax ||
        !std::isfinite(v) || std::fabs(v) > VelocityMax ||
        !std::isfinite(torque) || std::fabs(torque) > TorqueMax ||
        !(kp >= 0.0f && kp <= 500.0f) || !(kd >= 0.0f && kd <= 5.0f))
        return false;

    Struct_CAN_Tx_Msg frame{};
    frame.id_type = CAN_ID_EXTENDED;
    frame.len = 8;
    Write(frame.data, Encode(p, -PositionMax, PositionMax));
    Write(frame.data + 2, Encode(v, -VelocityMax, VelocityMax));
    Write(frame.data + 4, Encode(kp, 0, 500));
    Write(frame.data + 6, Encode(kd, 0, 5));
    const uint32_t torque_raw = Encode(torque, -TorqueMax, TorqueMax);

    Critical lock;
    if (!Ready())
        return false;
    frame.hfdcan = bus_;
    frame.id = (1U << 24) | (torque_raw << 8) | motor_id_;
    return CAN_Tx_PerformOwned(&frame, this);
}

void Class_EL05Motor::Receive(FDCAN_HandleTypeDef*, uint32_t id, uint8_t* data,
                              uint32_t len, void* context)
{
    if (len != 8)
        return;
    auto& motor = *static_cast<Class_EL05Motor*>(context);
    Critical lock;
    const bool timed_out = motor.confirmed_ ? !motor.daemon_.IsOnline() : NowMs() - motor.enable_started_ms_ >= motor.timeout_ms_;
    motor.feedback_.position_rad = Decode(data, PositionMax);
    motor.feedback_.velocity_rad_s = Decode(data + 2, VelocityMax);
    motor.feedback_.torque_nm = Decode(data + 4, TorqueMax);
    motor.feedback_.temperature_c = Read(data + 6) / 10.0f;
    motor.feedback_.faults = (id >> 16) & 0x3FU;
    motor.feedback_.mode = (id >> 22) & 3U;
    motor.daemon_.Feed();

    if (!motor.requested_ || motor.locked_)
        return;
    if (timed_out || motor.feedback_.faults || (motor.confirmed_ && motor.feedback_.mode != 2))
    {
        motor.locked_ = true;
        motor.Stop();
    }
    else if (motor.feedback_.mode == 2)
    {
        motor.confirmed_ = true;
    }
}

Struct_EL05Motor_Snapshot Class_EL05Motor::GetSnapshot() const
{
    Critical lock;
    auto snapshot = feedback_;
    snapshot.online = daemon_.IsOnline();
    snapshot.requested_enabled = requested_;
    snapshot.ready = Ready();
    return snapshot;
}

void Class_EL05Motor::ServiceAll()
{
    for (auto* motor = head_; motor; motor = motor->next_)
    {
        Critical lock;
        if (motor->requested_ && !motor->locked_ &&
            (motor->confirmed_ ? !motor->daemon_.IsOnline() : NowMs() - motor->enable_started_ms_ >= motor->timeout_ms_))
        {
            motor->locked_ = true;
            motor->Stop();
        }
        else if (motor->stop_pending_)
        {
            motor->Stop();
        }
    }
}
