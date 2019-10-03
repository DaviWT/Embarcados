//******************************************************************************
//  LAB 2 SISTEMAS EMBARCADOS - UTFPR 2019/2
//
//  Alunos:
//  -Adriano Ricardo de Abreu Gamba
//  -Davi Wei Tokikawa
//
//  Demodulator PWM
//
//  Outputs: Duty-Cycle, Period and Frequency (Serial).
//  Inputs: PWM waveform (PIN PL4).
//
//  Obs:
//  1) Based on TivaWare_C_Series-2.1.4.178;
//  2) Uses a serial terminal with 115,200.
//******************************************************************************

//
// Includes
//
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "system_TM4C1294.h" 

//
// Defines
//

extern void UARTStdioIntHandler(void);
int flagInterrTimerA0;
//******************************************************************************
//
// UART Configuration
//
//******************************************************************************
void UARTInit(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    
    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    
    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, SystemCoreClock);
} // UARTInit

void UART0_Handler(void)
{
    UARTStdioIntHandler();
} // UART0_Handler

//******************************************************************************
//
// TIMER Configuration
//
//******************************************************************************
void TimerA0Isr(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Clear the timer interrupt
    UARTprintf("Interrupcao do Timer A0\n");
    flagInterrTimerA0 = 1;
    //digitalWrite(RED_LED, digitalRead(RED_LED) ^ 1);              // toggle LED pin
}

void TIMERInit()
{
    // Enable the Timer0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    // Wait for the Timer0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){}

    // Configure TimerA as a half-width one-shot timer, and TimerB as a
    // half-width edge capture counter.
    TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT |
    TIMER_CFG_B_CAP_COUNT));
    
    // Set the count time for the the one-shot timer (TimerA).
    TimerLoadSet(TIMER0_BASE, TIMER_A, 3000);
    
    // Configure the counter (TimerB) to count both edges.
    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_BOTH_EDGES);
    
    // Registering ISR
    TimerIntRegister(TIMER0_BASE, TIMER_A, TimerA0Isr);
    
    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    
    // Enable timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

//******************************************************************************
//
// GPIO Configuration
//
//******************************************************************************
void GPIOInit()
{
    // Configure the device pins and enable Ports
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Set GPIO N (PN4 - PWM)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)); // Wait until done
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); // Set GPIO L (PL4 - PWM)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)); // Wait until done
    
    // Set pin PN4 to in for the PWM signal
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_4);
    GPIOPadConfigSet( GPIO_PORTN_BASE,
                      GPIO_PIN_4,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPU );
    
    // Set pin PL4 to timer capture for signal in
    GPIOPinConfigure(GPIO_PL4_T0CCP0);  
    GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_4);
}

//******************************************************************************
//
// Main
//
//******************************************************************************
void main(void)
{
    // Variaveis
    int count = 0;
  
    // Inicializa periféricos
    GPIOInit();
    UARTInit();
    TIMERInit();
    
    UARTprintf("Hello World!\n");
    
    flagInterrTimerA0 = 0;
    count = 0;
    while(1)
    {
        while(flagInterrTimerA0 == 0)
        {
            count++;
        }
        UARTprintf("count = %i\n",count);
        flagInterrTimerA0 = 0;
        count = 0;
        while(count < 50000)
          count++;
        count = 0;
        TimerEnable(TIMER0_BASE, TIMER_A);
    }
} // main