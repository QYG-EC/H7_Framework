#!/usr/bin/env python3
"""Exercise current production function bodies with host HAL fault injection."""
from pathlib import Path
import subprocess
import re

root = Path(__file__).resolve().parents[2]
out = root / 'build/host-reliability'
out.mkdir(parents=True, exist_ok=True)

def function(path, signature):
    source = (root / path).read_text()
    start = source.index(signature)
    brace = source.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]

common = '''
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <cmath>
static uint32_t irq;
uint32_t __get_PRIMASK() { return irq; }
void __disable_irq() { irq = 1; }
void __enable_irq() { irq = 0; }
void __set_PRIMASK(uint32_t x) { irq = x; }
void __DMB() {}
'''

def run(name, source):
    path = out / (name + '.cpp')
    path.write_text(common + source)
    binary = out / name
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    str(path), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
    print('PASS', name, flush=True)

uart = 'User_File/Middleware/BSP/UART/bsp_uart.cpp'
run('uart', '''
#define HAL_OK 0
#define UART_CLEAR_IDLEF 0
#define HAL_UART_ERROR_NONE 0
#define __HAL_UART_CLEAR_FLAG(h, f) ((void)0)
struct UART_HandleTypeDef { void* hdmarx; int ErrorCode; };
struct Struct_UART_Manage_Object {
 UART_HandleTypeDef* UART_Handler;
 bool Rx_Restart_Pending;
 uint8_t *Rx_Buffer_Active, *Rx_Buffer_0;
 unsigned Rx_Restart_Count, Rx_Restart_Failure_Count;
};
static Struct_UART_Manage_Object* active;
static bool inject;
static int abort_status, start_status;
int HAL_UART_AbortReceive(UART_HandleTypeDef*) { return abort_status; }
int UART_Start_Receive_ToIdle(Struct_UART_Manage_Object*) {
 if (inject) { active->Rx_Restart_Pending = true; }
 return start_status;
}
''' + function(uart, 'static bool UART_Restart_Receive(Struct_UART_Manage_Object *manage)\n{') + '''
int main() {
 uint8_t buffer[8]; UART_HandleTypeDef handle{buffer, 0};
 Struct_UART_Manage_Object m{&handle,true,nullptr,buffer,0,0}; active=&m;
 inject=true; assert(UART_Restart_Receive(&m)); assert(m.Rx_Restart_Pending);
 inject=false; assert(UART_Restart_Receive(&m)); assert(!m.Rx_Restart_Pending);
 abort_status=1; assert(!UART_Restart_Receive(&m)); assert(m.Rx_Restart_Pending);
 abort_status=0; start_status=1; assert(!UART_Restart_Receive(&m));
 assert(m.Rx_Restart_Pending && m.Rx_Buffer_Active==nullptr);
 irq=1; start_status=0; assert(UART_Restart_Receive(&m)); assert(irq==1);
}
''')

timestamp = 'User_File/System/Timestamp/sys_timestamp.cpp'
run('timestamp', '''
#define RESET 0
#define TIM_FLAG_UPDATE 1
struct Timer { uint32_t CNT, flag; } timer;
struct TIM_HandleTypeDef { Timer* Instance; } htim5{&timer};
#define __HAL_TIM_GET_FLAG(h, f) ((h)->Instance->flag)
class Class_Timestamp { public:
 TIM_HandleTypeDef* TIM_Handler=&htim5;
 uint32_t TIM_Overflow_Count=0;
 static constexpr uint64_t TIM_PERIOD_US=3600000000ULL;
 uint64_t Calculate_Timestamp() const;
} clock_source;
static bool blocked_preemption;
void HAL_TIM_IRQHandler(TIM_HandleTypeDef* h) {
 h->Instance->flag=0;
 blocked_preemption=irq==1; // HAL clear/update window cannot be observed by a maskable ISR.
 ++clock_source.TIM_Overflow_Count;
}
''' + function(timestamp, 'uint64_t Class_Timestamp::Calculate_Timestamp() const') +
function('Core/Src/stm32h7xx_it.c', 'void TIM5_IRQHandler(void)') + '''
int main() {
 timer={3599999999U,0}; assert(clock_source.Calculate_Timestamp()==3599999999ULL);
 timer={3,1}; assert(clock_source.Calculate_Timestamp()==3600000003ULL);
 TIM5_IRQHandler(); assert(blocked_preemption && irq==0);
 assert(clock_source.Calculate_Timestamp()==3600000003ULL);
 irq=1; assert(clock_source.Calculate_Timestamp()==3600000003ULL && irq==1);
}
''')

flash = 'User_File/Device/Onboard/W25Q64JV/bsp_w25q64jv.cpp'
flash_header = 'User_File/Device/Onboard/W25Q64JV/bsp_w25q64jv.h'
run('flash', '''
#define SEGGER_RTT_printf(...) ((void)0)
#define HAL_OK 0
using HAL_StatusTypeDef=int;
struct Clock { uint64_t now=0; uint64_t Get_Current_Timestamp() { return now; } } SYS_Timestamp;
class Class_W25Q64JV { public:
 bool Initialized=true, Busy_Flag=true, Auto_Polling_Pending=false;
 bool Suppress_AutoPolling=false, Write_Enable_Activated_Flag=false;
 uint64_t Busy_Timestamp=0, Current_Auto_Polling_Timeout=100;
 unsigned Auto_Polling_Error_Count=0, submissions=0;
 int Current_Instruction=0, status=0;
 void OSPI_RxCallback(); void OSPI_TxCallback();
 void TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback();
''' + function(flash_header, 'bool Is_Busy()') +
function(flash_header, 'bool Check_Transfer_Status(HAL_StatusTypeDef status)') + '''
 bool Auto_Polling_With_Timeout() { ++submissions; return Check_Transfer_Status(status); }
};
''' + function(flash, 'void Class_W25Q64JV::OSPI_RxCallback()') +
function(flash, 'void Class_W25Q64JV::OSPI_TxCallback()') +
function(flash, 'void Class_W25Q64JV::TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback()') + '''
int main() {
 Class_W25Q64JV f;
 f.OSPI_TxCallback(); assert(f.Auto_Polling_Pending && f.submissions==0);
 f.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback(); assert(f.submissions==1);
 f.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback(); assert(f.submissions==1);
 f.OSPI_RxCallback(); f.status=1;
 f.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback();
 assert(!f.Busy_Flag && f.Auto_Polling_Error_Count==1);
 f.Busy_Flag=true; f.OSPI_RxCallback(); SYS_Timestamp.now=101;
 f.TIM_1ms_AutoPollingTimeout_PeriodElapsedCallback();
 assert(!f.Busy_Flag && !f.Auto_Polling_Pending && f.Auto_Polling_Error_Count==2);
 assert(f.submissions==2);
}
''')

for board in ['single', 'gimbal', 'chassis']:
    body = function(f'User_Config/Board/board_tasks_{board}.c', 'void Board_CreateTasks(void)')
    tasks = re.findall(r'(\w+)Handle = osThreadNew\((\w+),', body)
    declarations = '\n'.join(f'void* {name}Handle; int {name}_attributes; void {entry}(void*) {{}}'
                             for name, entry in tasks)
    run('tasks_' + board, '''
static int calls, fail_at;
void* osThreadNew(void (*)(void*), void*, const int*) {
 return ++calls==fail_at ? nullptr : reinterpret_cast<void*>(1);
}
void Error_Handler() { throw 1; }
''' + declarations + body + f'''
int main() {{
 for (int i=0; i<={len(tasks)}; ++i) {{
  calls=0; fail_at=i; bool failed=false;
  try {{ Board_CreateTasks(); }} catch (int) {{ failed=true; }}
  assert(failed==(i!=0));
 }}
}}
''')

adc = 'User_File/Middleware/BSP/ADC/bsp_adc.cpp'
power = 'User_File/Device/Onboard/Power/bsp_power.h'
heater = 'User_File/Device/Onboard/BMI088/Accel/bsp_bmi088_accel.cpp'
run('power', '''
#define HAL_OK 0
#define RESET 0
#define HAL_ADC_ERROR_NONE 0
#define ADC_FLAG_OVR 1
#define HAL_DMA_STATE_BUSY 1
#define HAL_ADC_STATE_ERROR_INTERNAL 2
#define HAL_ADC_STATE_ERROR_DMA 4
#define CLEAR_BIT(r,b) ((r)&=~(b))
#define __HAL_ADC_GET_FLAG(h,f) ((h)->overrun)
struct Clock { uint64_t now=0; uint64_t Get_Current_Timestamp() { return now; } } SYS_Timestamp;
struct ADC_HandleTypeDef { int error=0,overrun=0,State=0; int* DMA_Handle; };
struct Struct_ADC_Manage_Object {
 ADC_HandleTypeDef* ADC_Handler=nullptr; uint16_t ADC_Data[128]{};
 bool Sample_Valid=false; uint64_t Sample_Timestamp=0;
} ADC1_Manage_Object;
Struct_ADC_Manage_Object* ADC_Get_Manage_Object(ADC_HandleTypeDef* h) {
 return h==ADC1_Manage_Object.ADC_Handler ? &ADC1_Manage_Object : nullptr;
}
int HAL_ADC_GetError(ADC_HandleTypeDef* h) { return h->error; }
int HAL_DMA_GetState(int* h) { return *h; }
static int starts, restarts, start_result;
int HAL_ADC_Stop_DMA(ADC_HandleTypeDef*) { return 0; }
int HAL_ADC_Start_DMA(ADC_HandleTypeDef* h,uint32_t*,int) {
 ++restarts; h->error=0; h->overrun=0; return 0;
}
int HAL_ADC_Start(ADC_HandleTypeDef*) { ++starts; return start_result; }
class Class_Power { public:
 Struct_ADC_Manage_Object* ADC_Manage_Object=&ADC1_Manage_Object;
 uint16_t ADC_Buffer_Offset=0;
 float Get_Power_Voltage() const;
} BSP_Power;
''' + function(adc, 'extern "C" void HAL_ADC_ConvCpltCallback') +
function(adc, 'extern "C" void HAL_ADC_ErrorCallback') +
function(adc, 'void ADC_TIM_10ms_PeriodElapsedCallback(void)') +
function(power, 'inline float Class_Power::Get_Power_Voltage() const') + '''
#define HEATER_PREHEAT_BASE_TEMPERATURE 30
#define HEATER_PREHEAT_POWER 1000
#define HEATER_TARGET_TEMPERATURE 40
#define HEATER_NOMINAL_VOLTAGE 24
static unsigned pwm;
#define __HAL_TIM_SET_COMPARE(h,c,v) (pwm=(v))
struct Struct_BMI088_Accel_Temperature_State { bool Data_Valid; float Temperature; };
bool Basic_Math_Is_Invalid_Float(float x) { return !std::isfinite(x); }
void Basic_Math_Constrain(float* x,float lo,float hi) { if (*x<lo) *x=lo; if (*x>hi) *x=hi; }
class Class_BMI088_Accel { public:
 bool Heater_Enable=true,Temperature_Stale_Latched=false,Heater_Preheat_Finished_Flag=false;
 unsigned BMI088_Temperature_Stale_Counter=0,Heater_PWM_Compare=0;
 struct PID {
  float integral=1;
  void Set_Integral_Error(float x) { integral=x; }
  void Set_Now(float) {} void Set_Target(float) {}
  void TIM_Calculate_PeriodElapsedCallback() {} float Get_Out() { return 1000; }
 } PID_Temperature;
 Struct_BMI088_Accel_Temperature_State Get_Temperature_State() { return {true,20}; }
 void TIM_128ms_Heater_PID_PeriodElapsedCallback();
};
''' + function(heater, 'void Class_BMI088_Accel::TIM_128ms_Heater_PID_PeriodElapsedCallback()') + '''
int main() {
 int dma=HAL_DMA_STATE_BUSY; ADC_HandleTypeDef h{0,0,0,&dma};
 auto& m=ADC1_Manage_Object; m.ADC_Handler=&h;
 Class_BMI088_Accel heater;
 heater.TIM_128ms_Heater_PID_PeriodElapsedCallback();
 assert(pwm==0 && heater.PID_Temperature.integral==0);
 m.ADC_Data[0]=45000; HAL_ADC_ConvCpltCallback(&h);
 heater.TIM_128ms_Heater_PID_PeriodElapsedCallback(); assert(pwm>0);
 SYS_Timestamp.now=100000; assert(BSP_Power.Get_Power_Voltage()>10);
 SYS_Timestamp.now=100001; assert(BSP_Power.Get_Power_Voltage()==0);
 heater.PID_Temperature.integral=4;
 heater.TIM_128ms_Heater_PID_PeriodElapsedCallback();
 assert(pwm==0 && heater.PID_Temperature.integral==0);
 m.ADC_Data[0]=15000; HAL_ADC_ConvCpltCallback(&h);
 heater.TIM_128ms_Heater_PID_PeriodElapsedCallback(); assert(pwm==0);
 ADC_TIM_10ms_PeriodElapsedCallback(); assert(starts==1);
 start_result=1; ADC_TIM_10ms_PeriodElapsedCallback(); assert(!m.Sample_Valid);
 h.error=1; ADC_TIM_10ms_PeriodElapsedCallback(); assert(restarts==1 && !m.Sample_Valid);
 HAL_ADC_ConvCpltCallback(&h); assert(m.Sample_Valid);
 HAL_ADC_ErrorCallback(&h); assert(!m.Sample_Valid);
 irq=1; BSP_Power.Get_Power_Voltage(); assert(irq==1);
}
''')
