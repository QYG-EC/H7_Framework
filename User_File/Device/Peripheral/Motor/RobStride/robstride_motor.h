#ifndef ROBSTRIDE_MOTOR_H
#define ROBSTRIDE_MOTOR_H

#include "bsp_can.h"
#include "daemon.h"

enum class Enum_RobStride_Model : uint8_t
{
    EL05,
    RS00,
    RS01,
    RS02,
    RS03,
    RS04,
    RS05,
    RS06
};
enum class Enum_RobStride_Mode : uint8_t
{
    MOTION = 0,
    POSITION_PP = 1,
    SPEED = 2,
    CURRENT = 3,
    POSITION_CSP = 5
};

struct Struct_RobStride_Config
{
    FDCAN_HandleTypeDef* hfdcan = nullptr;
    uint8_t can_id = 127U;
    uint8_t host_id = 0xFDU;
    Enum_RobStride_Model model = Enum_RobStride_Model::EL05;
    bool reverse = false;
    // 旧版 RS 固件需按对应手册改为 12.5 rad。
    float position_max_rad = 12.57f;
};

struct Struct_RobStride_Snapshot
{
    float position_rad = 0.0f, velocity_rad_s = 0.0f, torque_nm = 0.0f, temperature_c = 0.0f;
    float total_position_rad = 0.0f; // 0x7019 原生多圈位置，不猜测状态帧回绕。
    bool position_valid = false, control_configured = false;
    uint8_t faults = 0U, state = 0U, configuration_step = 0U;
    uint32_t fault_code = 0U, warning_code = 0U;
    bool online = false, requested_enabled = false, ready = false;
    bool mode_confirmed = false, identity_received = false;
    Enum_RobStride_Mode mode = Enum_RobStride_Mode::MOTION;
    uint8_t uid[8]{};
    uint16_t parameter_index = 0U;
    uint32_t parameter_bits = 0U, parameter_sequence = 0U;
    bool parameter_ok = false;
};

/** 灵足私有扩展帧协议。对象须为静态生命周期，不可复制。
 *  所有 true 只表示软件入队；模式须由参数回读确认，ready 须由新鲜运动反馈确认。
 *  操作接口由同一个任务调用；ServiceAll 在 StatusTask，反馈在 ISR，内部短临界区同步。
 */
class Class_RobStride_Motor
{
  public:
    Class_RobStride_Motor() = default;
    Class_RobStride_Motor(const Class_RobStride_Motor&) = delete;
    Class_RobStride_Motor& operator=(const Class_RobStride_Motor&) = delete;
    bool Init(const Struct_RobStride_Config& config);
    // 异步准备原生速度模式，依次停机、开启反馈、切模式并回读限制；不会使能。
    bool ConfigureSpeed(float current_limit_a, float torque_limit_nm,
                        float acceleration_rad_s2, uint32_t timeout_ms, float position_speed_limit_rad_s = 10.0f);
    // 在已请求的配置基础上安全切换速度/CSP 位置模式；接受请求不等于切换完成。
    bool RequestControlMode(Enum_RobStride_Mode mode);
    bool RequestEnabled(bool enabled);
    bool SetMode(Enum_RobStride_Mode mode); // 仅在线、已确认失能时允许，ServiceAll 回读确认。
    bool SetMotion(float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm);
    bool SetTorque(float torque_nm);
    bool SetSpeed(float velocity_rad_s);
    bool SetCurrent(float current_a);
    bool SetPosition(float position_rad); // PP/CSP，多圈位置由原生位置模式处理。
    bool SetCurrentLimit(float current_a);
    bool SetTorqueLimit(float torque_nm);
    bool SetCanTimeoutMs(uint32_t timeout_ms); // 电机侧看门狗；0 关闭，须按固件核对支持。
    bool SetSpeedLimit(float velocity_rad_s);
    bool SetAcceleration(float acceleration_rad_s2);
    bool ClearError();
    bool SetZeroPosition();
    bool QueryIdentity();
    bool ReadParameter(uint16_t index);
    // 仅失能时写配置参数；run_mode 和运动目标必须使用上面的专用接口。
    bool WriteParameter(uint16_t index, uint32_t bits);
    bool SetActiveReporting(bool enabled);
    bool SaveParameters();
    Struct_RobStride_Snapshot GetFeedbackSnapshot() const;
    bool IsOnline() const;
    bool IsHealthy() const;
    const Daemon& GetDaemon() const
    {
        return feedback_daemon;
    }
    static void ServiceAll();

  private:
    static void FeedbackCallback(FDCAN_HandleTypeDef*, uint32_t, uint8_t*, uint32_t, void*);
    bool Send(uint8_t type, const uint8_t* data = nullptr, uint16_t extra = 0U);
    bool WriteFloat(uint16_t index, float value);
    bool WriteBits(uint16_t index, uint32_t value);
    bool CanConfigureUnlocked() const;
    bool CanConfigure() const;
    void ServiceControlConfiguration(uint32_t now);
    bool ControlConfigurationParameter(uint8_t step, uint16_t& index, uint32_t& bits, bool& is_float) const;
    Struct_RobStride_Config config{};
    Struct_RobStride_Snapshot feedback{};
    float velocity_max = 50.0f, torque_max = 6.0f, current_max = 11.0f;
    float kp_max = 500.0f, kd_max = 5.0f;
    bool initialized = false, registration_attempted = false;
    bool lifecycle_requested = false, stop_pending = false;
    uint32_t stop_request_seq = 0U;
    bool clear_error_pending = false;
    bool mode_pending = false;
    uint8_t pending_mode = 0U;
    uint32_t mode_query_ms = 0U;
    uint32_t mode_request_ms = 0U;
    bool control_configuration_requested = false, configuration_waiting = false;
    bool base_parameters_configured = false;
    uint8_t configuration_step = 0U;
    float speed_current_limit = 0.0f, speed_torque_limit = 0.0f, speed_acceleration = 0.0f;
    float position_speed_limit = 10.0f;
    Enum_RobStride_Mode control_mode = Enum_RobStride_Mode::SPEED;
    uint32_t speed_timeout_ms = 0U, configuration_query_ms = 0U;
    uint32_t status_query_ms = 0U;
    bool position_received = false;
    uint32_t position_rx_ms = 0U;
    Daemon feedback_daemon{100U};
    Class_RobStride_Motor* next = nullptr;
    static Class_RobStride_Motor* head;
};
#endif
