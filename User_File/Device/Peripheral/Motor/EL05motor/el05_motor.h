#ifndef EL05_MOTOR_H
#define EL05_MOTOR_H

#include "bsp_can.h"
#include "daemon.h"

struct Struct_EL05Motor_Snapshot
{
    float position_rad = 0;
    float velocity_rad_s = 0;
    float torque_nm = 0;
    float temperature_c = 0;
    uint8_t faults = 0;
    uint8_t mode = 0;
    bool online = false;
    bool requested_enabled = false;
    bool ready = false;
};

/** 静态实例，启动阶段注册；一个应用任务独占命令。 */
class Class_EL05Motor
{
  public:
    Class_EL05Motor() = default;
    Class_EL05Motor(const Class_EL05Motor&) = delete;
    Class_EL05Motor& operator=(const Class_EL05Motor&) = delete;
    bool Init(FDCAN_HandleTypeDef* bus, uint8_t motor_id, uint8_t master_id,
              uint32_t timeout_ms = 100);
    bool RequestEnabled(bool enabled);
    bool SetMotion(float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm);
    Struct_EL05Motor_Snapshot GetSnapshot() const;
    static void ServiceAll();

  private:
    static void Receive(FDCAN_HandleTypeDef*, uint32_t, uint8_t*, uint32_t, void*);
    bool SubmitAction(uint32_t type);
    void Stop();
    bool Ready() const;

    FDCAN_HandleTypeDef* bus_ = nullptr;
    uint8_t motor_id_ = 0;
    uint8_t master_id_ = 0;
    uint32_t timeout_ms_ = 0;
    uint32_t enable_started_ms_ = 0;
    bool requested_ = false;
    bool confirmed_ = false;
    bool stop_pending_ = false;
    bool locked_ = false;
    Daemon daemon_{100};
    Struct_EL05Motor_Snapshot feedback_{};
    Class_EL05Motor* next_ = nullptr;
    static Class_EL05Motor* head_;
};

#endif
