#include "el05_motor.h"
#include "cmsis_os2.h"
#include "vesc_motor.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <vector>

static FDCAN_GlobalTypeDef registers[3]{};
FDCAN_HandleTypeDef hfdcan1{0, &registers[0]}, hfdcan2{1, &registers[1]}, hfdcan3{2, &registers[2]};
static FDCAN_HandleTypeDef* buses[] = {&hfdcan1, &hfdcan2, &hfdcan3};
static uint32_t primask;
static uint64_t now_us;
static void (*on_unmask)();
static bool busy[3], fail[3];
static uint32_t extended_filter[3];
struct Frame
{
    FDCAN_TxHeaderTypeDef header;
    std::array<uint8_t, 8> data{};
};
struct RxFrame
{
    FDCAN_RxHeaderTypeDef header;
    std::array<uint8_t, 64> data{};
};
static std::vector<Frame> sent[3];
static std::deque<RxFrame> received[3];
struct Queue
{
    uint32_t capacity, size;
    std::deque<std::vector<uint8_t>> items;
};
static Queue queues[3];
static unsigned queue_count;

extern "C"
{
    uint32_t __get_PRIMASK()
    {
        return primask;
    }
    void __disable_irq()
    {
        primask = 1;
    }
    void __enable_irq()
    {
        primask = 0;
        if (on_unmask)
        {
            auto hook = on_unmask;
            on_unmask = nullptr;
            hook();
        }
    }
    void __set_PRIMASK(uint32_t state)
    {
        if (state)
            primask = state;
        else
            __enable_irq();
    }
    void __DMB()
    {
    }
    void Error_Handler()
    {
        std::abort();
    }
    uint64_t SYS_Timestamp_Get_Microsecond()
    {
        return now_us;
    }
    HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef*, FDCAN_FilterTypeDef* filter)
    {
        assert(filter->IdType == FDCAN_STANDARD_ID && filter->FilterID2 == 0);
        return HAL_OK;
    }
    HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef* bus, uint32_t standard,
                                                   uint32_t extended, uint32_t remote_std, uint32_t remote_ext)
    {
        assert(standard == FDCAN_REJECT && remote_std == FDCAN_REJECT_REMOTE && remote_ext == FDCAN_REJECT_REMOTE);
        extended_filter[bus->bus] = extended;
        return HAL_OK;
    }
    HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef*, uint32_t, uint32_t)
    {
        return HAL_OK;
    }
    HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef*)
    {
        return HAL_OK;
    }
    uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef* bus, uint32_t)
    {
        return received[bus->bus].size();
    }
    HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef* bus, uint32_t,
                                             FDCAN_RxHeaderTypeDef* header, uint8_t* data)
    {
        auto& queue = received[bus->bus];
        if (queue.empty())
            return HAL_ERROR;
        *header = queue.front().header;
        std::memcpy(data, queue.front().data.data(), 64);
        queue.pop_front();
        return HAL_OK;
    }
    uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef* bus)
    {
        return busy[bus->bus] ? 0 : 8;
    }
    HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef* bus,
                                                    const FDCAN_TxHeaderTypeDef* header, const uint8_t* data)
    {
        if (fail[bus->bus])
            return HAL_ERROR;
        assert(!busy[bus->bus] && header->DataLength <= 8);
        Frame frame{*header, {}};
        std::memcpy(frame.data.data(), data, header->DataLength);
        sent[bus->bus].push_back(frame);
        return HAL_OK;
    }
    osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size, const osMessageQueueAttr_t* attr)
    {
        assert(queue_count < 3 && attr->mq_size >= count * size);
        auto& queue = queues[queue_count++];
        queue.capacity = count;
        queue.size = size;
        return &queue;
    }
    osStatus_t osMessageQueuePut(osMessageQueueId_t id, const void* data, uint8_t, uint32_t timeout)
    {
        assert(timeout == 0);
        auto& queue = *static_cast<Queue*>(id);
        if (queue.items.size() == queue.capacity)
            return osErrorResource;
        const auto* bytes = static_cast<const uint8_t*>(data);
        queue.items.emplace_back(bytes, bytes + queue.size);
        return osOK;
    }
    osStatus_t osMessageQueueGet(osMessageQueueId_t id, void* data, uint8_t*, uint32_t timeout)
    {
        assert(timeout == 0);
        auto& queue = *static_cast<Queue*>(id);
        if (queue.items.empty())
            return osErrorResource;
        std::memcpy(data, queue.items.front().data(), queue.size);
        queue.items.pop_front();
        return osOK;
    }
}

static void Inject(FDCAN_HandleTypeDef* bus, uint32_t id, uint32_t type,
                   std::initializer_list<uint8_t> bytes, uint32_t frame_type = FDCAN_DATA_FRAME,
                   uint32_t format = FDCAN_CLASSIC_CAN)
{
    RxFrame frame{};
    frame.header = {id, type, frame_type, static_cast<uint32_t>(bytes.size()), format};
    std::copy(bytes.begin(), bytes.end(), frame.data.begin());
    received[bus->bus].push_back(frame);
    HAL_FDCAN_RxFifo0Callback(bus, FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
}
static void Count(FDCAN_HandleTypeDef*, uint32_t, uint8_t*, uint32_t, void* ctx)
{
    ++*static_cast<int*>(ctx);
}
static Struct_CAN_Tx_Msg Message(FDCAN_HandleTypeDef* bus, uint32_t id, Enum_CAN_Id_Type type)
{
    Struct_CAN_Tx_Msg result{};
    result.hfdcan = bus;
    result.id = id;
    result.id_type = type;
    result.len = 4;
    return result;
}
static uint32_t Bits(const Frame& frame)
{
    return (uint32_t(frame.data[0]) << 24) | (uint32_t(frame.data[1]) << 16) |
           (uint32_t(frame.data[2]) << 8) | frame.data[3];
}
static Struct_CAN_Tx_Msg replacement;
static int owner;
static void ReplaceSnapshot()
{
    assert(CAN_Tx_PerformOwned(&replacement, &owner));
}

static void Routing(unsigned mask)
{
    int standard[3]{}, extended[3]{};
    for (unsigned i = 0; i < 3; ++i)
    {
        auto* bus = buses[i];
        bool enabled = (mask & (1U << i)) != 0;
        assert(BSP_CAN_ExtendedEnabled(bus) == enabled);
        assert(extended_filter[i] == (enabled ? FDCAN_ACCEPT_IN_RX_FIFO0 : FDCAN_REJECT));
        assert(BSP_CAN_RegisterCallback(0x117, bus, Count, &standard[i]));
        assert(BSP_CAN_RegisterCallbackEx(0x117, CAN_ID_EXTENDED, bus, Count, &extended[i]) == enabled);
        Inject(bus, 0x117, FDCAN_STANDARD_ID, {1});
        Inject(bus, 0x117, FDCAN_EXTENDED_ID, {1});
        Inject(bus, 0x117, FDCAN_EXTENDED_ID, {1}, FDCAN_REMOTE_FRAME);
        Inject(bus, 0x117, FDCAN_EXTENDED_ID, {1}, FDCAN_DATA_FRAME, FDCAN_FD_CAN);
        assert(standard[i] == 1 && extended[i] == int(enabled));
        auto msg = Message(bus, 0x117, CAN_ID_STANDARD);
        assert(CAN_Tx_Perform(&msg));
        msg.id_type = CAN_ID_EXTENDED;
        assert(CAN_Tx_Perform(&msg) == enabled);
        assert(CAN_Tx_Submit(&msg) == enabled);
        msg.id = 0x20000000U;
        assert(!CAN_Tx_Perform(&msg));
        msg.id_type = CAN_ID_STANDARD;
        msg.id = 0x800;
        assert(!CAN_Tx_Submit(&msg));
    }
    assert(!BSP_CAN_ExtendedEnabled(nullptr));
    assert(!CAN_Tx_Perform(nullptr));
    assert(!BSP_CAN_RegisterCallback(1, nullptr, Count, nullptr));
    busy[0] = true;
    assert(!BSP_CAN_SendPer());
    BSP_CAN_SendAsync();
    assert(sent[0].empty());
    assert(!sent[1].empty() && !sent[2].empty());
    busy[0] = false;
    assert(BSP_CAN_SendPer());
    BSP_CAN_SendAsync();
    for (unsigned i = 0; i < 3; ++i)
    {
        assert(sent[i].size() == ((mask & (1U << i)) ? 3U : 1U));
        sent[i].clear();
    }
    // Publish a different ID for the same owner after the consumer takes its snapshot.
    auto msg = Message(&hfdcan1, 0x301, CAN_ID_STANDARD);
    assert(CAN_Tx_PerformOwned(&msg, &owner));
    replacement = msg;
    replacement.id = 0x101;
    on_unmask = ReplaceSnapshot;
    (void) BSP_CAN_SendPer();
    assert(BSP_CAN_SendPer());
    assert(sent[0].size() == 1 && sent[0][0].header.Identifier == 0x101);
    assert(primask == 0);
    __disable_irq();
    assert(CAN_Tx_Perform(&msg));
    assert(primask == 1);
    __enable_irq();
}

static Struct_VESCMotor_Init_Config Config(FDCAN_HandleTypeDef* bus, uint8_t id = 23)
{
    Struct_VESCMotor_Init_Config config{};
    config.hfdcan = bus;
    config.can_id = id;
    config.pole_pairs = 7;
    config.current_limit_a = 100;
    config.speed_limit_rad_s = 1000;
    config.duty_limit = 1;
    config.relative_current_limit = 1;
    config.allow_position = true;
    return config;
}
static void Motion(FDCAN_HandleTypeDef* bus, uint8_t node = 23)
{
    Inject(bus, 0x900 | node, FDCAN_EXTENDED_ID, {0, 0, 1, 0xA4, 0xFF, 0x9C, 1, 0xF4}); // 420 erpm, -10 A, 0.5 duty
}
static void Motor(unsigned mask)
{
    static Class_VESCMotor motors[3];
    for (unsigned i = 0; i < 3; ++i)
    {
        auto* bus = buses[i];
        const bool enabled = (mask & (1U << i)) != 0;
        assert(motors[i].Init(Config(bus)) == enabled);
        if (!enabled)
        {
            assert(!motors[i].RequestEnabled(true) && !motors[i].Send());
            continue;
        }
        auto& motor = motors[i];
        assert(!motor.SetCurrent(1));
        assert(motor.RequestEnabled(true));
        assert(!motor.GetSnapshot().ready);
        // Other bus / wrong node / short DLC / standard format must not feed this motor.
        Motion(buses[(i + 1) % 3], 24);
        Motion(bus, 24);
        Inject(bus, 0x917, FDCAN_EXTENDED_ID, {1, 2, 3});
        Inject(bus, 0x917, FDCAN_STANDARD_ID, {0, 0, 1, 0xA4, 0, 0, 0, 0});
        assert(!motor.GetSnapshot().online);
        Motion(bus);
        auto snap = motor.GetSnapshot();
        assert(snap.ready && std::fabs(snap.speed_rad_s - 6.2831853f) < 0.00001f);
        assert(snap.current_a == -10 && snap.duty == 0.5f);
        Inject(bus, 0x1017, FDCAN_EXTENDED_ID, {1, 0x90, 1, 0xF4, 0, 0x64, 0x23, 0x28});
        Inject(bus, 0x1B17, FDCAN_EXTENDED_ID, {0xFF, 0xFF, 0xFF, 0xFA, 1, 0xE0, 0, 0});
        snap = motor.GetSnapshot();
        assert(snap.status4_valid && snap.status5_valid);
        assert(snap.fet_temperature_c == 40 && snap.motor_temperature_c == 50);
        assert(snap.input_current_a == 10 && std::fabs(snap.pid_position_rad - 3.14159265f) < 0.00001f);
        assert(snap.tachometer_raw == -6 && snap.input_voltage_v == 48);

        struct Vector
        {
            Enum_VESCMotor_Mode mode;
            float value;
            int32_t raw;
        };
        const Vector vectors[] = {
            {Enum_VESCMotor_Mode::DUTY, -0.5f, -50000},
            {Enum_VESCMotor_Mode::CURRENT, 51, 51000},
            {Enum_VESCMotor_Mode::BRAKE_CURRENT, -2, -2000},
            {Enum_VESCMotor_Mode::SPEED, -6.2831853f, -420},
            {Enum_VESCMotor_Mode::POSITION, 3.14159265f, 180000000},
            {Enum_VESCMotor_Mode::CURRENT_REL, -0.25f, -25000},
            {Enum_VESCMotor_Mode::BRAKE_CURRENT_REL, 0.25f, 25000},
            {Enum_VESCMotor_Mode::HANDBRAKE_CURRENT, 2, 2000},
            {Enum_VESCMotor_Mode::HANDBRAKE_CURRENT_REL, 1, 100000}};
        for (const auto& v : vectors)
        {
            sent[i].clear();
            assert(motor.SetTarget(v.mode, v.value) && motor.Send());
            assert(BSP_CAN_SendPer());
            assert(sent[i].size() == 1);
            const auto& frame = sent[i].back();
            assert(frame.header.IdType == FDCAN_EXTENDED_ID && frame.header.DataLength == 4);
            assert(frame.header.Identifier == ((uint32_t(v.mode) << 8) | 23));
            assert(Bits(frame) == uint32_t(v.raw));
        }
        assert(!motor.SetCurrent(NAN) && !motor.SetCurrent(INFINITY));
        assert(!motor.SetCurrent(101) && !motor.SetDuty(1.01f) && !motor.SetPosition(-1));
        assert(!motor.SetTarget(static_cast<Enum_VESCMotor_Mode>(99), 0));
        sent[i].clear();
        busy[i] = true;
        assert(motor.SetSpeed(100) && motor.Send());
        assert(!BSP_CAN_SendPer());
        assert(motor.SetBrakeCurrent(3) && motor.Send());
        busy[i] = false;
        assert(BSP_CAN_SendPer());
        assert(sent[i].size() == 1 && sent[i].back().header.Identifier == 0x217);
        assert(Bits(sent[i].back()) == 3000);
        // Disable supersedes pending control, HAL errors retry only the safe value.
        sent[i].clear();
        assert(motor.SetSpeed(100) && motor.Send());
        assert(motor.RequestEnabled(false));
        fail[i] = true;
        assert(!BSP_CAN_SendPer());
        fail[i] = false;
        assert(BSP_CAN_SendPer());
        assert(sent[i].size() == 1 && sent[i].back().header.Identifier == 0x117 && Bits(sent[i].back()) == 0);
        assert(motor.RequestEnabled(true));
        assert(motor.Send() && BSP_CAN_SendPer());
        assert(Bits(sent[i].back()) == 0);
        assert(motor.SetCurrent(10) && motor.Send());
        now_us += 100000;
        assert(!motor.GetSnapshot().ready);
        DaemonManager::CheckAll();
        Class_VESCMotor::ServiceAll();
        assert(BSP_CAN_SendPer());
        assert(Bits(sent[i].back()) == 0);
        Motion(bus);
        assert(motor.Send() && BSP_CAN_SendPer());
        assert(Bits(sent[i].back()) == 0);
        // Receive recovery before StatusTask runs must invalidate old targets as well.
        assert(motor.SetCurrent(10) && motor.Send());
        now_us += 100000;
        Motion(bus);
        // CAN task can run before StatusTask after the recovery ISR.
        assert(BSP_CAN_SendPer() && Bits(sent[i].back()) == 0);
        Class_VESCMotor::ServiceAll();
        now_us += 500000;
        assert(!motor.GetSnapshot().status4_valid && !motor.GetSnapshot().status5_valid);
    }
    assert(primask == 0);
}

static void Race()
{
    auto msg = Message(&hfdcan1, 0x301, CAN_ID_STANDARD);
    assert(CAN_Tx_PerformOwned(&msg, &owner));
    replacement = msg;
    replacement.id = 0x101;
    on_unmask = ReplaceSnapshot; // Only slot 0 exists; fires exactly after its snapshot copy.
    assert(!BSP_CAN_SendPer());
    assert(sent[0].empty());
    assert(BSP_CAN_SendPer());
    assert(sent[0].size() == 1 && sent[0].back().header.Identifier == 0x101);
}

static void Capacity(unsigned mask)
{
    int contexts[17]{};
    auto* bus = &hfdcan1;
    for (unsigned i = 0; i < 32; ++i)
    {
        auto msg = Message(bus, i, CAN_ID_STANDARD);
        assert(CAN_Tx_Perform(&msg));
    }
    auto msg = Message(bus, 32, CAN_ID_STANDARD);
    assert(!CAN_Tx_Perform(&msg));
    for (unsigned i = 0; i < 16; ++i)
        assert(CAN_Tx_Submit(&msg));
    assert(!CAN_Tx_Submit(&msg));
    for (unsigned i = 0; i < 15; ++i)
        assert(BSP_CAN_RegisterCallback(i, bus, Count, &contexts[i]));
    // Failed multi-frame registration must release the successful prefix.
    for (unsigned i = 0; i < 3; ++i)
    {
        if (!(mask & (1U << i)))
            continue;
        static Class_VESCMotor failed;
        assert(!failed.Init(Config(buses[i])));
        assert(BSP_CAN_RegisterCallbackEx(0x917, CAN_ID_EXTENDED, buses[i], Count, &contexts[15]));
        BSP_CAN_UnregisterCallbackEx(0x917, CAN_ID_EXTENDED, buses[i], &contexts[15]);
        break;
    }
    assert(BSP_CAN_RegisterCallback(15, bus, Count, &contexts[15]));
    assert(!BSP_CAN_RegisterCallback(16, bus, Count, &contexts[16]));
    Struct_CAN_Tx_Stats stats{};
    BSP_CAN_GetTxStats(&stats);
    assert(stats.submit_queue_full_count == 1 && stats.periodic_slot_full_count == 1);
}

static void EL05(unsigned mask)
{
    static Class_EL05Motor motor;
    auto* bus = buses[0];
    assert(motor.Init(bus, 1, 0xFD) == bool(mask & 1));
    if (!(mask & 1)) return;
    assert(motor.RequestEnabled(true));
    Class_EL05Motor::ServiceAll();
    assert(motor.GetSnapshot().requested_enabled && !motor.GetSnapshot().ready);
    assert(BSP_CAN_SendPer());
    assert(sent[0].back().header.Identifier == 0x0300FD01U);
    Inject(buses[1], 0x028001FDU, FDCAN_EXTENDED_ID, {0, 0, 0, 0, 0, 0, 1, 44});
    assert(!motor.GetSnapshot().ready);
    Inject(bus, 0x028001FDU, FDCAN_EXTENDED_ID, {0, 0, 0, 0, 0, 0, 1, 44});
    assert(motor.GetSnapshot().ready && motor.GetSnapshot().temperature_c == 30.0f);
    assert(motor.SetMotion(12.57f, -50.0f, 500.0f, 0.0f, 6.0f));
    assert(BSP_CAN_SendPer());
    assert(sent[0].back().header.Identifier == 0x01FFFF01U);
    assert(Bits(sent[0].back()) == 0xFFFF0000U);
    assert(motor.SetMotion(0, 0, 0, 0, 0));
    now_us += 101000;
    Class_EL05Motor::ServiceAll();
    assert(BSP_CAN_SendPer());
    assert(sent[0].back().header.Identifier == 0x0400FD01U);
    assert(!motor.GetSnapshot().ready && !motor.RequestEnabled(true));
}

static void Recovery()
{
    registers[0] = {FDCAN_PSR_BO, FDCAN_CCCR_INIT};
    for (auto* bus : buses)
    {
        Struct_CAN_Tx_Msg msg{};
        msg.hfdcan = bus;
        msg.id = 0x123;
        msg.len = 8;
        assert(CAN_Tx_Submit(&msg));
    }
    BSP_CAN_SendAsync();
    assert(sent[0].empty() && sent[1].size() == 1 && sent[2].size() == 1);
    now_us = 99999;
    BSP_CAN_Recover();
    assert(registers[0].CCCR == FDCAN_CCCR_INIT);
    now_us = 100000;
    BSP_CAN_Recover();
    assert(registers[0].CCCR == 0);
    BSP_CAN_SendAsync();
    assert(sent[0].empty()); // 硬件尚未结束 Bus-Off 恢复。
    registers[0].PSR = 0;
    BSP_CAN_SendAsync();
    assert(sent[0].size() == 1);
    registers[0] = {FDCAN_PSR_BO, FDCAN_CCCR_INIT};
    registers[1] = {FDCAN_PSR_BO, FDCAN_CCCR_INIT};
    now_us = 150000;
    BSP_CAN_Recover();
    assert(registers[0].CCCR == FDCAN_CCCR_INIT && registers[1].CCCR == 0);
    now_us = 200000;
    BSP_CAN_Recover();
    assert(registers[0].CCCR == 0);
    registers[2].CCCR = FDCAN_CCCR_INIT; // 主动停止，不应被恢复服务启动。
    BSP_CAN_Recover();
    assert(registers[2].CCCR == FDCAN_CCCR_INIT);
}

int main(int argc, char** argv)
{
    assert(argc == 3);
    unsigned mask = std::strtoul(argv[1], nullptr, 10);
    BSP_CAN_ConfigInit();
    if (std::strcmp(argv[2], "routing") == 0)
        Routing(mask);
    else if (std::strcmp(argv[2], "motor") == 0)
        Motor(mask);
    else if (std::strcmp(argv[2], "race") == 0)
        Race();
    else if (std::strcmp(argv[2], "recovery") == 0)
        Recovery();
    else if (std::strcmp(argv[2], "el05") == 0)
        EL05(mask);
    else
        Capacity(mask);
    std::printf("PASS mask=%u %s\n", mask, argv[2]);
}
