#include "robstride_motor.h"

#include "sys_timestamp.h"

#include <cmath>
#include <cstring>

namespace
{
constexpr uint32_t kPositionValidTimeoutMs = 100U;
constexpr float kConfigFloatTolerance = 1e-3f;

uint32_t NowMs()
{
    return static_cast<uint32_t>(SYS_Timestamp_Get_Microsecond() / 1000ULL);
}

struct CriticalSection
{
    uint32_t primask = __get_PRIMASK();
    CriticalSection()
    {
        __disable_irq();
        __DMB();
    }
    ~CriticalSection()
    {
        __DMB();
        __set_PRIMASK(primask);
    }
};
bool ValidMode(uint8_t mode)
{
    return mode <= 3U || mode == 5U;
}
uint16_t Encode(float value, float minimum, float maximum)
{
    value = std::fmax(minimum, std::fmin(maximum, value));
    return static_cast<uint16_t>((value - minimum) * 65535.0f / (maximum - minimum));
}
uint16_t ReadBE(const uint8_t* data)
{
    return static_cast<uint16_t>((data[0] << 8U) | data[1]);
}
void WriteBE(uint8_t* data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value >> 8U);
    data[1] = static_cast<uint8_t>(value);
}
uint32_t ReadLE(const uint8_t* data)
{
    return uint32_t(data[0]) | (uint32_t(data[1]) << 8U) |
           (uint32_t(data[2]) << 16U) | (uint32_t(data[3]) << 24U);
}
} // namespace

Class_RobStride_Motor* Class_RobStride_Motor::head = nullptr;

bool Class_RobStride_Motor::Init(const Struct_RobStride_Config& value)
{
    if (registration_attempted || value.hfdcan == nullptr || !BSP_CAN_ExtendedEnabled(value.hfdcan) ||
        value.can_id == 0U || value.can_id >= 0xFEU ||
        value.host_id == 0xFEU || value.host_id == 0xFFU || value.host_id == value.can_id ||
        static_cast<uint8_t>(value.model) > 7U || !std::isfinite(value.position_max_rad) ||
        value.position_max_rad <= 0.0f)
        return false;

    // 厂商现行私有协议映射值，与电机额定机械性能区分。
    static constexpr float speeds[] = {50, 33, 44, 44, 20, 15, 50, 50};
    static constexpr float torques[] = {6, 14, 17, 17, 60, 120, 5.5f, 36};
    static constexpr float currents[] = {11, 15.5f, 23, 23, 27, 45, 11, 27};
    const auto model = static_cast<uint8_t>(value.model);
    registration_attempted = true;
    config = value;
    velocity_max = speeds[model];
    torque_max = torques[model];
    current_max = currents[model];
    if (model == 4U || model == 5U || model == 7U)
    {
        kp_max = 5000;
        kd_max = 100;
    }
    // 一个节点统一接收类型 0/2/17/21，避免多项注册出现部分成功。
    // 失败对象不可销毁或改换节点重试：BSP/Daemon 注册表不提供注销。
    if (!DaemonManager::Register(feedback_daemon) ||
        !BSP_CAN_RegisterMaskedCallback(uint32_t(config.can_id) << 8U, 0x0000FF00U,
                                        CAN_ID_EXTENDED, config.hfdcan, FeedbackCallback, this))
        return false;
    initialized = true;
    next = head;
    head = this;
    return true; // Init 不发送命令，不使能，不隐式切模式。
}

bool Class_RobStride_Motor::Send(uint8_t type, const uint8_t* data, uint16_t extra)
{
    if (!initialized)
        return false;
    Struct_CAN_Tx_Msg message{};
    message.hfdcan = config.hfdcan;
    message.id_type = CAN_ID_EXTENDED;
    message.id = (uint32_t(type) << 24U) |
                 (uint32_t(type == 1U ? extra : config.host_id) << 8U) | config.can_id;
    message.len = 8U;
    if (data != nullptr)
        std::memcpy(message.data, data, 8U);
    // 力矩位于 ID，参数目标共享同一个 ID；都不能以完整 ID 的周期槽保存。
    // FIFO 保序，停止命令排在已接受的目标后，不会被旧周期槽重新覆盖。
    return CAN_Tx_Submit(&message);
}

void Class_RobStride_Motor::FeedbackCallback(FDCAN_HandleTypeDef* bus, uint32_t id,
                                             uint8_t* data, uint32_t length, void* context)
{
    auto& motor = *static_cast<Class_RobStride_Motor*>(context);
    if (!motor.initialized || bus != motor.config.hfdcan || data == nullptr || length != 8U ||
        id > 0x1FFFFFFFU || ((id >> 8U) & 0xFFU) != motor.config.can_id)
        return;
    const uint8_t type = static_cast<uint8_t>(id >> 24U);
    const uint8_t target = static_cast<uint8_t>(id & 0xFFU);
    auto& feedback = motor.feedback;
    if (type == 0U && target == 0xFEU && (id & 0x00FF0000U) == 0U)
    {
        std::memcpy(feedback.uid, data, 8U);
        feedback.identity_received = true;
        return;
    }
    if (type == 21U)
    {
        if (target != motor.config.host_id && target != 0xFEU)
            return;
        feedback.fault_code = ReadLE(data);
        feedback.warning_code = ReadLE(data + 4);
        if (feedback.fault_code != 0U)
        {
            feedback.requested_enabled = false;
            motor.clear_error_pending = false;
            motor.base_parameters_configured = false;
            motor.stop_pending = true; // ISR 只标记，StatusTask 提交停机。
            ++motor.stop_request_seq;
        }
        return;
    }
    if (target != motor.config.host_id)
        return;
    if (type == 2U)
    {
        const uint8_t state = static_cast<uint8_t>((id >> 22U) & 3U);
        if (feedback.requested_enabled && feedback.state == 2U && state != 2U)
        {
            // 非主动失能的复位/看门狗停机不能沿用旧模式与限制确认。
            feedback.requested_enabled = false;
            feedback.mode_confirmed = false;
            motor.base_parameters_configured = false;
            motor.stop_pending = true;
            ++motor.stop_request_seq;
        }
        // 即使低频服务尚未来得及观察离线，也不能让恢复反馈复活旧许可。
        if (feedback.mode_confirmed && !motor.feedback_daemon.IsOnline())
        {
            if (feedback.requested_enabled)
            {
                motor.stop_pending = true;
                ++motor.stop_request_seq;
            }
            feedback.requested_enabled = false;
            feedback.mode_confirmed = false;
            motor.base_parameters_configured = false;
        }
        const float direction = motor.config.reverse ? -1.0f : 1.0f;
        feedback.position_rad = direction * (ReadBE(data) * (2.0f * motor.config.position_max_rad / 65535.0f) - motor.config.position_max_rad);
        feedback.velocity_rad_s = direction * (ReadBE(data + 2) * (2.0f * motor.velocity_max / 65535.0f) - motor.velocity_max);
        feedback.torque_nm = direction * (ReadBE(data + 4) * (2.0f * motor.torque_max / 65535.0f) - motor.torque_max);
        feedback.temperature_c = ReadBE(data + 6) * 0.1f;
        feedback.faults = static_cast<uint8_t>((id >> 16U) & 0x3FU);
        // Type 2 仅有 6 个故障位，不自动抹掉 Type 21 的 32 位详细故障；
        // 仅在显式清错后收到无故障反馈时同步清零旧详细故障码。
        if (feedback.faults == 0U && motor.clear_error_pending)
        {
            feedback.fault_code = 0U;
            motor.clear_error_pending = false;
        }
        feedback.state = state;
        motor.feedback_daemon.Feed(); // 身份/参数回复不冒充运动反馈在线。
        if (feedback.faults != 0U)
        {
            feedback.requested_enabled = false;
            motor.clear_error_pending = false;
            motor.base_parameters_configured = false;
            motor.stop_pending = true;
            ++motor.stop_request_seq;
        }
    }
    else if (type == 17U)
    {
        feedback.parameter_index = static_cast<uint16_t>(ReadLE(data) & 0xFFFFU);
        feedback.parameter_bits = ReadLE(data + 4);
        feedback.parameter_ok = ((id >> 16U) & 0xFFU) == 0U;
        ++feedback.parameter_sequence;
        if (feedback.parameter_ok && feedback.parameter_index == 0x7019U)
        {
            float position;
            std::memcpy(&position, &feedback.parameter_bits, sizeof(position));
            if (std::isfinite(position))
            {
                feedback.total_position_rad = motor.config.reverse ? -position : position;
                motor.position_received = true;
                motor.position_rx_ms = NowMs();
            }
        }
        if (motor.control_configuration_requested && motor.configuration_waiting)
        {
            uint16_t index = 0U;
            uint32_t bits = 0U;
            bool is_float = false;
            if (motor.ControlConfigurationParameter(motor.configuration_step, index, bits, is_float) &&
                feedback.parameter_ok && feedback.parameter_index == index)
            {
                bool matched = false;
                if (is_float)
                {
                    float actual = 0.0f;
                    float expected = 0.0f;
                    std::memcpy(&actual, &feedback.parameter_bits, sizeof(actual));
                    std::memcpy(&expected, &bits, sizeof(expected));
                    matched = std::isfinite(actual) && std::fabs(actual - expected) <= kConfigFloatTolerance;
                }
                else
                {
                    matched = feedback.parameter_bits == bits;
                }
                if (matched)
                {
                    if (motor.configuration_step == 6U && motor.base_parameters_configured)
                        motor.configuration_step = 8U;
                    else
                        ++motor.configuration_step;
                    if (motor.configuration_step == 8U)
                        motor.base_parameters_configured = true;
                    motor.configuration_waiting = false;
                }
            }
        }
        if (motor.mode_pending && NowMs() - motor.mode_request_ms < 1000U &&
            feedback.parameter_ok && feedback.parameter_index == 0x7005U &&
            data[4] == motor.pending_mode)
        {
            feedback.mode = static_cast<Enum_RobStride_Mode>(motor.pending_mode);
            feedback.mode_confirmed = true;
            motor.mode_pending = false;
        }
    }
}

Struct_RobStride_Snapshot Class_RobStride_Motor::GetFeedbackSnapshot() const
{
    CriticalSection lock;
    auto result = feedback;
    result.configuration_step = configuration_step;
    result.online = initialized && feedback_daemon.IsOnline();
    result.position_valid = result.online && position_received && NowMs() - position_rx_ms < kPositionValidTimeoutMs;
    result.control_configured = result.online && result.mode_confirmed &&
                                control_configuration_requested && configuration_step == 8U && result.mode == control_mode;
    result.ready = result.online && result.requested_enabled && result.state == 2U &&
                   result.faults == 0U && result.fault_code == 0U && result.mode_confirmed && !stop_pending;
    return result;
}
bool Class_RobStride_Motor::IsOnline() const
{
    return GetFeedbackSnapshot().online;
}
bool Class_RobStride_Motor::IsHealthy() const
{
    return GetFeedbackSnapshot().ready;
}
bool Class_RobStride_Motor::CanConfigureUnlocked() const
{
    return initialized && feedback_daemon.IsOnline() && feedback.state == 0U &&
           !feedback.requested_enabled && !stop_pending;
}
bool Class_RobStride_Motor::CanConfigure() const
{
    CriticalSection lock;
    return CanConfigureUnlocked();
}

bool Class_RobStride_Motor::RequestEnabled(bool enabled)
{
    if (!enabled)
    {
        bool need_send = false;
        uint32_t seq = 0U;
        {
            CriticalSection lock;
            if (!initialized)
                return false;
            if (!lifecycle_requested || feedback.requested_enabled)
            {
                stop_pending = true;
                ++stop_request_seq;
            }
            feedback.requested_enabled = false;
            lifecycle_requested = true;
            need_send = stop_pending;
            seq = stop_request_seq;
        }
        if (!need_send)
            return true;
        if (!Send(4U))
            return false;
        CriticalSection lock;
        if (stop_request_seq == seq)
            stop_pending = false;
        return !stop_pending;
    }

    Enum_RobStride_Mode mode = Enum_RobStride_Mode::MOTION;
    float total_position_rad = 0.0f;
    {
        CriticalSection lock;
        if (!initialized)
            return false;
        const bool online = feedback_daemon.IsOnline();
        const bool position_valid = online && position_received && NowMs() - position_rx_ms < kPositionValidTimeoutMs;
        const bool control_configured = online && feedback.mode_confirmed &&
                                        control_configuration_requested && configuration_step == 8U &&
                                        feedback.mode == control_mode;
        if (!online || !feedback.mode_confirmed || feedback.faults || feedback.fault_code || stop_pending ||
            (control_configuration_requested && !control_configured))
            return false;
        if (feedback.requested_enabled)
            return true;
        mode = feedback.mode;
        total_position_rad = feedback.total_position_rad;
        if (mode == Enum_RobStride_Mode::POSITION_CSP && !position_valid)
            return false;
    }

    // 临界区外提交非阻塞 CAN 队列，避免关全局中断时调用 FreeRTOS API。
    // 原生速度/CSP 模式使能前先清除旧目标，保持 FIFO 顺序。
    if (mode == Enum_RobStride_Mode::SPEED && !WriteFloat(0x700AU, 0.0f))
        return false;
    if (mode == Enum_RobStride_Mode::POSITION_CSP &&
        !WriteFloat(0x7016U, config.reverse ? -total_position_rad : total_position_rad))
        return false;
    if (!Send(3U))
        return false;

    CriticalSection lock;
    const bool online = feedback_daemon.IsOnline();
    const bool control_configured = online && feedback.mode_confirmed &&
                                    control_configuration_requested && configuration_step == 8U &&
                                    feedback.mode == control_mode;
    if (!online || !feedback.mode_confirmed || feedback.mode != mode || feedback.faults || feedback.fault_code ||
        stop_pending || (control_configuration_requested && !control_configured))
    {
        // 发帧期间状态失效时标记补发停机，避免残留已入队的使能请求。
        feedback.requested_enabled = false;
        stop_pending = true;
        ++stop_request_seq;
        return false;
    }
    feedback.requested_enabled = true;
    feedback.state = 0U; // 必须等请求后反馈确认运行。
    lifecycle_requested = true;
    return true;
}

void Class_RobStride_Motor::ServiceAll()
{
    for (auto* motor = head; motor != nullptr; motor = motor->next)
    {
        const uint32_t now = NowMs();

        bool need_stop = false;
        uint32_t stop_seq = 0U;
        bool need_enable = false;
        Enum_RobStride_Mode enable_mode = Enum_RobStride_Mode::MOTION;
        bool enable_position_valid = false;
        float enable_position_rad = 0.0f;
        bool need_mode_query = false;
        bool need_status_query = false;
        bool run_control_config = false;
        bool need_position_read = false;

        {
            CriticalSection lock;
            const bool online = motor->initialized && motor->feedback_daemon.IsOnline();
            if (!online)
            {
                motor->feedback.mode_confirmed = false;
                motor->base_parameters_configured = false;
                if (motor->feedback.requested_enabled)
                {
                    motor->feedback.requested_enabled = false;
                    motor->stop_pending = true; // 恢复后须上层重新请求，不重放运动目标。
                    ++motor->stop_request_seq;
                }
            }
            if (motor->stop_pending)
            {
                need_stop = true;
                stop_seq = motor->stop_request_seq;
            }
            if (online && motor->feedback.requested_enabled && motor->feedback.state != 2U &&
                motor->feedback.faults == 0U && motor->feedback.fault_code == 0U && !motor->stop_pending)
            {
                need_enable = true;
                enable_mode = motor->feedback.mode;
                enable_position_valid = motor->position_received && now - motor->position_rx_ms < kPositionValidTimeoutMs;
                enable_position_rad = motor->feedback.total_position_rad;
            }
            if (motor->mode_pending && now - motor->mode_request_ms >= 1000U)
                motor->mode_pending = false; // 不确认模式，允许调用方重新发起切换。
            if (motor->mode_pending && now - motor->mode_query_ms >= 100U)
                need_mode_query = true;
            if (motor->control_configuration_requested)
            {
                run_control_config = true;
                // 部分固件不维持类型 24 主动上报。失能时用停止帧取得类型 2 回复，
                // 不依赖参数回复喂狗，也不为维持在线而使能。失败项保留到下次服务重试。
                if (!motor->feedback.requested_enabled && now - motor->status_query_ms >= 50U)
                    need_status_query = true;
                if (online)
                    need_position_read = true;
            }
        }

        if (need_stop && motor->Send(4U))
        {
            CriticalSection lock;
            if (motor->stop_request_seq == stop_seq)
                motor->stop_pending = false;
        }
        if (need_enable)
        {
            bool prewrite_ok = true;
            if (enable_mode == Enum_RobStride_Mode::SPEED)
                prewrite_ok = motor->WriteFloat(0x700AU, 0.0f);
            else if (enable_mode == Enum_RobStride_Mode::POSITION_CSP)
                prewrite_ok = enable_position_valid &&
                              motor->WriteFloat(0x7016U, motor->config.reverse ? -enable_position_rad : enable_position_rad);
            if (prewrite_ok)
                (void) motor->Send(3U);
        }
        if (need_mode_query && motor->ReadParameter(0x7005U))
        {
            CriticalSection lock;
            motor->mode_query_ms = now;
        }
        if (need_status_query && motor->Send(4U))
        {
            CriticalSection lock;
            motor->status_query_ms = now;
        }
        if (run_control_config)
            motor->ServiceControlConfiguration(now);
        if (need_position_read)
            (void) motor->ReadParameter(0x7019U);
    }
}

bool Class_RobStride_Motor::ConfigureSpeed(float current, float torque, float acceleration, uint32_t timeout_ms, float position_speed)
{
    CriticalSection lock;
    if (!initialized || control_configuration_requested || !std::isfinite(current) || current <= 0 || current > current_max ||
        !std::isfinite(torque) || torque <= 0 || torque > torque_max || !std::isfinite(acceleration) || acceleration <= 0 ||
        timeout_ms == 0U || timeout_ms > UINT32_MAX / 20U ||
        !std::isfinite(position_speed) || position_speed <= 0.0f || position_speed > velocity_max)
        return false;
    speed_current_limit = current;
    speed_torque_limit = torque;
    speed_acceleration = acceleration;
    speed_timeout_ms = timeout_ms;
    position_speed_limit = position_speed;
    control_configuration_requested = true;
    base_parameters_configured = false;
    feedback.requested_enabled = false;
    stop_pending = true;
    ++stop_request_seq;
    return true;
}

bool Class_RobStride_Motor::RequestControlMode(Enum_RobStride_Mode mode)
{
    uint32_t seq = 0U;
    {
        CriticalSection lock;
        if (!control_configuration_requested ||
            (mode != Enum_RobStride_Mode::SPEED && mode != Enum_RobStride_Mode::POSITION_CSP))
            return false;
        if (control_mode == mode)
            return true;
        control_mode = mode;
        const bool fast_switch = base_parameters_configured && feedback_daemon.IsOnline() &&
                                 feedback.faults == 0U && feedback.fault_code == 0U;
        configuration_step = fast_switch ? 2U : 0U;
        if (!fast_switch)
            base_parameters_configured = false;
        configuration_query_ms = NowMs();
        configuration_waiting = false;
        mode_pending = false;
        feedback.mode_confirmed = false;
        feedback.requested_enabled = false;
        stop_pending = true;
        seq = ++stop_request_seq;
    }
    if (Send(4U))
    {
        CriticalSection lock;
        if (stop_request_seq == seq)
            stop_pending = false;
    }
    return true;
}

bool Class_RobStride_Motor::ControlConfigurationParameter(uint8_t step, uint16_t& index, uint32_t& bits, bool& is_float) const
{
    if (step < 4U || step > 7U)
        return false;
    static constexpr uint16_t indexes[] = {0x7018U, 0x700BU, 0x7022U, 0x7028U};
    index = indexes[step - 4U];
    if (step == 6U && control_mode == Enum_RobStride_Mode::POSITION_CSP)
        index = 0x7017U;
    if (step == 7U)
    {
        bits = speed_timeout_ms * 20U;
        is_float = false;
    }
    else
    {
        const float value = step == 4U ? speed_current_limit : step == 5U                                ? speed_torque_limit
                                                           : control_mode == Enum_RobStride_Mode::POSITION_CSP ? position_speed_limit
                                                                                                               : speed_acceleration;
        std::memcpy(&bits, &value, sizeof(bits));
        is_float = true;
    }
    return true;
}

void Class_RobStride_Motor::ServiceControlConfiguration(uint32_t now)
{
    enum class ConfigAction : uint8_t
    {
        NONE,
        SEND_STOP,
        ENABLE_REPORT,
        SET_MODE,
        WRITE_PARAM,
        READ_PARAM
    };

    ConfigAction action = ConfigAction::NONE;
    uint8_t step_snapshot = 0U;
    Enum_RobStride_Mode target_mode = Enum_RobStride_Mode::SPEED;
    uint16_t param_index = 0U;
    uint32_t param_bits = 0U;

    {
        CriticalSection lock;
        if (configuration_step == 8U)
        {
            if (feedback.mode_confirmed)
                return;
            configuration_step = 0U; // 掉线/重启后必须重新配置，不继承电机内未知状态。
            configuration_waiting = false;
            base_parameters_configured = false;
        }
        step_snapshot = configuration_step;
        target_mode = control_mode;
        if (configuration_step == 0U)
        {
            action = ConfigAction::SEND_STOP;
        }
        else if (configuration_step == 1U)
        {
            action = ConfigAction::ENABLE_REPORT;
        }
        else if (configuration_step == 2U)
        {
            if (CanConfigureUnlocked() && !mode_pending)
                action = ConfigAction::SET_MODE;
            else if (now - configuration_query_ms >= 1000U)
            {
                configuration_step = 0U; // 电机晚于 MCU 上电或切换超时时重做完整配置。
                base_parameters_configured = false;
            }
        }
        else if (configuration_step == 3U)
        {
            if (feedback.mode_confirmed && feedback.mode == control_mode)
                configuration_step = base_parameters_configured ? 6U : 4U;
            else if (!mode_pending)
                configuration_step = 2U;
        }
        else if (CanConfigureUnlocked())
        {
            bool is_float = false;
            if (ControlConfigurationParameter(configuration_step, param_index, param_bits, is_float))
            {
                if (!configuration_waiting)
                    action = ConfigAction::WRITE_PARAM;
                else if (now - configuration_query_ms >= 500U)
                    configuration_waiting = false; // 未确认则重写；始终禁止使能。
                else
                    action = ConfigAction::READ_PARAM;
            }
        }
    }

    switch (action)
    {
    case ConfigAction::SEND_STOP:
        if (Send(4U))
        {
            CriticalSection lock;
            if (configuration_step == 0U && control_mode == target_mode)
            {
                feedback.requested_enabled = false;
                configuration_step = 1U;
            }
        }
        break;
    case ConfigAction::ENABLE_REPORT:
        if (SetActiveReporting(true))
        {
            CriticalSection lock;
            if (configuration_step == 1U && control_mode == target_mode)
            {
                configuration_query_ms = now;
                configuration_step = 2U;
            }
        }
        break;
    case ConfigAction::SET_MODE:
        if (SetMode(target_mode))
        {
            CriticalSection lock;
            if (configuration_step == 2U && control_mode == target_mode)
                configuration_step = 3U;
        }
        break;
    case ConfigAction::WRITE_PARAM:
        if (WriteBits(param_index, param_bits))
        {
            CriticalSection lock;
            if (configuration_step == step_snapshot && control_mode == target_mode)
            {
                configuration_waiting = true;
                configuration_query_ms = now;
            }
        }
        break;
    case ConfigAction::READ_PARAM:
        (void) ReadParameter(param_index);
        break;
    case ConfigAction::NONE:
    default:
        break;
    }
}

bool Class_RobStride_Motor::WriteBits(uint16_t index, uint32_t bits)
{
    uint8_t data[8] = {static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8U), 0, 0,
                       static_cast<uint8_t>(bits), static_cast<uint8_t>(bits >> 8U),
                       static_cast<uint8_t>(bits >> 16U), static_cast<uint8_t>(bits >> 24U)};
    return Send(18U, data);
}
bool Class_RobStride_Motor::WriteFloat(uint16_t index, float value)
{
    if (!std::isfinite(value))
        return false;
    uint32_t bits;
    static_assert(sizeof(bits) == sizeof(value), "Protocol requires 32-bit float");
    std::memcpy(&bits, &value, sizeof(bits));
    return WriteBits(index, bits);
}
bool Class_RobStride_Motor::SetMode(Enum_RobStride_Mode mode)
{
    const uint8_t raw = static_cast<uint8_t>(mode);
    const uint32_t now = NowMs();
    {
        CriticalSection lock;
        if (!CanConfigureUnlocked() || !ValidMode(raw) || mode_pending)
            return false;
        pending_mode = raw;
        mode_pending = true;
        feedback.mode_confirmed = false;
        mode_request_ms = now;
        // 下一个服务周期立即首次回读 0x7005，避免固定空等 100 ms；未确认则每 100 ms 重试。
        mode_query_ms = now - 100U;
    }
    if (!WriteBits(0x7005U, raw))
    {
        CriticalSection lock;
        if (mode_pending && pending_mode == raw)
            mode_pending = false;
        return false;
    }
    // 写入成功不等于已切换；低频服务发送参数回读，确认后才允许使能。
    return true;
}
bool Class_RobStride_Motor::SetMotion(float position, float velocity, float kp, float kd, float torque)
{
    const auto state = GetFeedbackSnapshot();
    if (!state.ready || state.mode != Enum_RobStride_Mode::MOTION ||
        !std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(kp) ||
        !std::isfinite(kd) || !std::isfinite(torque) || kp < 0 || kd < 0)
        return false;
    const float direction = config.reverse ? -1.0f : 1.0f;
    uint8_t data[8];
    WriteBE(data, Encode(direction * position, -config.position_max_rad, config.position_max_rad));
    WriteBE(data + 2, Encode(direction * velocity, -velocity_max, velocity_max));
    WriteBE(data + 4, Encode(kp, 0, kp_max));
    WriteBE(data + 6, Encode(kd, 0, kd_max));
    return Send(1U, data, Encode(direction * torque, -torque_max, torque_max));
}
bool Class_RobStride_Motor::SetTorque(float torque)
{
    return SetMotion(0, 0, 0, 0, torque);
}
bool Class_RobStride_Motor::SetSpeed(float velocity)
{
    const auto state = GetFeedbackSnapshot();
    if (!state.ready || state.mode != Enum_RobStride_Mode::SPEED || !std::isfinite(velocity))
        return false;
    velocity = std::fmax(-velocity_max, std::fmin(velocity_max, velocity));
    return WriteFloat(0x700AU, config.reverse ? -velocity : velocity);
}
bool Class_RobStride_Motor::SetCurrent(float current)
{
    const auto state = GetFeedbackSnapshot();
    if (!state.ready || state.mode != Enum_RobStride_Mode::CURRENT || !std::isfinite(current))
        return false;
    current = std::fmax(-current_max, std::fmin(current_max, current));
    return WriteFloat(0x7006U, config.reverse ? -current : current);
}
bool Class_RobStride_Motor::SetPosition(float position)
{
    const auto state = GetFeedbackSnapshot();
    if (!state.ready || (state.mode != Enum_RobStride_Mode::POSITION_PP &&
                         state.mode != Enum_RobStride_Mode::POSITION_CSP))
        return false;
    return WriteFloat(0x7016U, config.reverse ? -position : position);
}
bool Class_RobStride_Motor::SetCurrentLimit(float current)
{
    return CanConfigure() && std::isfinite(current) && current >= 0 && current <= current_max && WriteFloat(0x7018U, current);
}
bool Class_RobStride_Motor::SetTorqueLimit(float torque)
{
    return CanConfigure() && std::isfinite(torque) && torque >= 0 && torque <= torque_max && WriteFloat(0x700BU, torque);
}
bool Class_RobStride_Motor::SetCanTimeoutMs(uint32_t timeout_ms)
{
    return CanConfigure() && timeout_ms <= UINT32_MAX / 20U && WriteBits(0x7028U, timeout_ms * 20U);
}
bool Class_RobStride_Motor::SetSpeedLimit(float velocity)
{
    uint16_t index = 0U;
    {
        CriticalSection lock;
        if (!CanConfigureUnlocked() || !feedback.mode_confirmed || !std::isfinite(velocity) || velocity < 0 || velocity > velocity_max)
            return false;
        if (feedback.mode == Enum_RobStride_Mode::POSITION_PP)
            index = 0x7024U;
        else if (feedback.mode == Enum_RobStride_Mode::POSITION_CSP)
            index = 0x7017U;
        else
            return false;
    }
    return WriteFloat(index, velocity);
}
bool Class_RobStride_Motor::SetAcceleration(float acceleration)
{
    uint16_t index = 0U;
    {
        CriticalSection lock;
        if (!CanConfigureUnlocked() || !feedback.mode_confirmed || !std::isfinite(acceleration) || acceleration <= 0)
            return false;
        if (feedback.mode == Enum_RobStride_Mode::POSITION_PP)
            index = 0x7025U;
        else if (feedback.mode == Enum_RobStride_Mode::SPEED)
            index = 0x7022U;
        else
            return false;
    }
    return WriteFloat(index, acceleration);
}
bool Class_RobStride_Motor::ClearError()
{
    if (!CanConfigure())
        return false;
    uint8_t data[8] = {1U};
    if (!Send(4U, data))
        return false;
    CriticalSection lock;
    clear_error_pending = true;
    return true; // 故障值只随设备反馈更新，不以入队成功冒充清错确认。
}
bool Class_RobStride_Motor::SetZeroPosition()
{
    {
        CriticalSection lock;
        if (!CanConfigureUnlocked() || !feedback.mode_confirmed ||
            (feedback.mode != Enum_RobStride_Mode::MOTION && feedback.mode != Enum_RobStride_Mode::POSITION_CSP))
            return false;
    }
    uint8_t data[8] = {1U};
    return Send(6U, data); // 不提前重置本地位置，等待真实反馈。
}
bool Class_RobStride_Motor::QueryIdentity()
{
    return Send(0U);
}
bool Class_RobStride_Motor::ReadParameter(uint16_t index)
{
    uint8_t data[8] = {static_cast<uint8_t>(index), static_cast<uint8_t>(index >> 8U)};
    return Send(17U, data);
}
bool Class_RobStride_Motor::WriteParameter(uint16_t index, uint32_t bits)
{
    if (!CanConfigure() || index == 0x7005U || index == 0x7006U || index == 0x700AU || index == 0x7016U)
        return false;
    return WriteBits(index, bits);
}
bool Class_RobStride_Motor::SetActiveReporting(bool enabled)
{
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, static_cast<uint8_t>(enabled), 0};
    return Send(24U, data);
}
bool Class_RobStride_Motor::SaveParameters()
{
    if (!CanConfigure())
        return false;
    const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    return Send(22U, data);
}
