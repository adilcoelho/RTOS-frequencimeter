#include <stdbool.h>
#include <stdint.h>
#include "system_tm4c1294.h" // CMSIS-Core
#include "driverleds.h" // device drivers
#include "cmsis_os2.h" // CMSIS-RTOS
#include "inc/tm4c1294ncpdt.h" // CMSIS-Core
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h" // driverlib
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#define MSGQUEUE_OBJECTS      4                                   // number of Message Queue Objects
#define FUNDO_DE_ESCALA_HZ 10000
#define FUNDO_DE_ESCALA_kHZ 1e6 


osThreadId_t PWM_thread1_id, PWM_thread2_id, PWM_thread3_id, PWM_thread4_id, threadControladora_id, Timer1_thread_id, UART_thread_id, VUmeter_thread_id;
osMessageQueueId_t mid_MsgQueue;                                   // message queue id
osMutexId_t LEDmutex_id;
osEventFlagsId_t freqAcquired;

void ConfigureUART(void);


typedef struct {                                                   // object data type
  uint8_t LED;
  uint8_t DC;
} MSGQUEUE_OBJ_t;



uint32_t freq;

void Timer1_ISR() 
{
  freq = 0xFFFFFF - TimerValueGet(TIMER0_BASE, TIMER_A);
  TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);
  HWREG(TIMER0_BASE+0x50)=0xFFFFFF; // reset timer
  TimerEnable(TIMER0_BASE, TIMER_A);
  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  osThreadFlagsSet(Timer1_thread_id, 0x1);
}


static void
PortJ_IntHandler(void)
{
  GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  // envia mensagem (ou ativa flag) para informar a troca de escala
}
 
int Init_MsgQueue (void) {
  
  mid_MsgQueue = osMessageQueueNew(MSGQUEUE_OBJECTS, sizeof(MSGQUEUE_OBJ_t), NULL);
  if (!mid_MsgQueue) {
    
  }
  return(0);
}
 
void Timer1_thread(void* arg) {
  while(1){
    osThreadFlagsWait(0x1, osFlagsWaitAny, osWaitForever);
    // aqui deve acontecer o teste de mudança de escala solicitada, vai mudar o load do Timer1 e vai alterar a escala de acordo com o necessário
    // a mudança devera ser efetivada somente quando uma nova Timer1_ISR for chamada
    osEventFlagsSet(freqAcquired, 0x1);
  }
}

void UART_thread(void* arg) {
  ConfigureUART();
  while(1){
    osEventFlagsWait(freqAcquired, 0x1, osFlagsNoClear, osWaitForever);
    UARTprintf("frequencia: %d\n", freq); // deve permitir mudar de escala no futuro
  }
}

void VUmeter_thread(void* arg) {
  uint32_t nivelAtual = 0;
  MSGQUEUE_OBJ_t msg;
  while(1){
    osEventFlagsWait(freqAcquired, 0x1, osFlagsWaitAny, osWaitForever);
    // faz a conta para saber o que mandar para cada led

    uint32_t LEDS = freq * 4 / (FUNDO_DE_ESCALA_HZ / 10); // tem que permitir mudar de escala no futuro
    
    if (LEDS >= 10){
      msg.DC = 10;
    } else {
      msg.DC = LEDS;
    }
    msg.LED = LED1;
    osMessageQueuePut(mid_MsgQueue, &msg, 0, osWaitForever);
    if (LEDS >= 20){
      msg.DC = 10;
    } else if (LEDS > 10){
      msg.DC = LEDS % 10;
    } else {
      msg.DC = 0;
    }
    msg.LED = LED2;
    osMessageQueuePut(mid_MsgQueue, &msg, 0, osWaitForever);
    if (LEDS >= 30){
      msg.DC = 10;
    } else if (LEDS > 20){
      msg.DC = LEDS % 10;
    } else {
      msg.DC = 0;
    }
    msg.LED = LED3;
    osMessageQueuePut(mid_MsgQueue, &msg, 0, osWaitForever);
    if (LEDS >= 40){
      msg.DC = 10;
    } else if (LEDS > 30){
      msg.DC = LEDS % 10;
    } else {
      msg.DC = 0;
    }
    msg.LED = LED4;
    osMessageQueuePut(mid_MsgQueue, &msg, 0, osWaitForever);
  }
}

void PWM_thread(void *arg){
  uint32_t tick;
  MSGQUEUE_OBJ_t msg;
   osMessageQueueGet(mid_MsgQueue, &msg, NULL, osWaitForever);
  while(1){
    osMessageQueueGet(mid_MsgQueue, &msg, NULL, 0);
    tick = osKernelGetTickCount();
    int atraso = msg.DC;
    osMutexAcquire(LEDmutex_id, osWaitForever);
    if(atraso != 0)
      LEDOn(msg.LED);
    osMutexRelease(LEDmutex_id);
    osDelayUntil(tick + atraso);
    tick = osKernelGetTickCount();
    osMutexAcquire(LEDmutex_id, osWaitForever);
    if(atraso != 10)
      LEDOff(msg.LED);
    osMutexRelease(LEDmutex_id);
    osDelayUntil(tick + 10 - atraso);
  } 
} 



void GPIOInit() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)); // Aguarda final da habilita��o

  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // configure button pins with pull ups7
  GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1, GPIO_RISING_EDGE);
  GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
  GPIOIntRegister(GPIO_PORTJ_BASE, PortJ_IntHandler);

  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)); // Aguarda final da habilita��o
  GPIOPinConfigure(GPIO_PL4_T0CCP0);
  GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_4);
}

void CounterTimerInit() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)); // Aguarda final da habilita��o
  TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_COUNT);
  TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
  TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff);
}

void Timer1Init() {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); 
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)); // Aguarda final da habilita��o
  TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
  TimerLoadSet(TIMER1_BASE, TIMER_A, SystemCoreClock);
  TimerIntRegister(TIMER1_BASE, TIMER_A, Timer1_ISR);
  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}


void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    // Enable the GPIO Peripheral used by the UART.


    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);    // Enable UART0.


    GPIOPinConfigure(GPIO_PA0_U0RX);                // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);


    UARTStdioConfig(0, 115200, SystemCoreClock);    // Initialize the UART for console I/O.
}

void main(void){
  SystemInit();
  LEDInit(LED1|LED2|LED3|LED4);
  GPIOInit();
  CounterTimerInit();
  Timer1Init();
  ConfigureUART();

  osKernelInitialize();
  const osMutexAttr_t LEDmutex_attr = {
    "LEDMutex",
    osMutexRecursive | osMutexPrioInherit,
    NULL,
    0U
  };
  LEDmutex_id = osMutexNew(&LEDmutex_attr);

  Init_MsgQueue();
  //threadControladora_id = osThreadNew(threadControladora, NULL, NULL);
  
  VUmeter_thread_id = osThreadNew(VUmeter_thread, NULL, NULL);
  PWM_thread1_id = osThreadNew(PWM_thread, NULL, NULL);
  osThreadSetPriority(PWM_thread1_id, osPriorityAboveNormal);
  PWM_thread2_id = osThreadNew(PWM_thread, NULL, NULL);
  osThreadSetPriority(PWM_thread2_id, osPriorityAboveNormal);
  PWM_thread3_id = osThreadNew(PWM_thread, NULL, NULL);
  osThreadSetPriority(PWM_thread3_id, osPriorityAboveNormal);
  PWM_thread4_id = osThreadNew(PWM_thread, NULL, NULL);
  osThreadSetPriority(PWM_thread4_id, osPriorityAboveNormal);
  UART_thread_id = osThreadNew(UART_thread, NULL, NULL);
  
  Timer1_thread_id = osThreadNew(Timer1_thread, NULL, NULL);
  osThreadSetPriority(Timer1_thread_id, osPriorityHigh);

  
  freqAcquired = osEventFlagsNew(NULL);
  
  if(osKernelGetState() == osKernelReady)
  {
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);
    osKernelStart();
  }

  while(1);
} // main
