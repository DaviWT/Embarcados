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
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "system_TM4C1294.h" 

//
// Defines
//
#define TIMER_MS 2      //up to 2.7

//
// Global Variables
//
extern void UARTStdioIntHandler(void);
int flagInterrTimerA0;
int PIN_N5_STATE=0;

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
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  // Clear timer interrupt
//    UARTprintf("Interrupcao do Timer A0\n");
//    flagInterrTimerA0 = 1;
    PIN_N5_STATE ^= GPIO_PIN_5;
    //digitalWrite(RED_LED, digitalRead(RED_LED) ^ 1);         // toggle LED pin
    TimerEnable(TIMER0_BASE, TIMER_A);
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
    TimerLoadSet(TIMER0_BASE, TIMER_A, SystemCoreClock*TIMER_MS/1000);   //1ms
//    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFF);   //x ms
    
    // Configure the counter (TimerB) to count both edges.
    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_BOTH_EDGES);
    
    // Registering ISR
    TimerIntRegister(TIMER0_BASE, TIMER_A, TimerA0Isr);
    
    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
    
    // Enable processor interrupts.
//    IntMasterEnable();
    
    // Enable the Timer0B interrupt on the processor (NVIC).
//    IntEnable(INT_TIMER0B);
    
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
    
    // Configure output pin for debug purposes
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_5); // PIN N5 como saída
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, 0); // PIN N5 apagados
    GPIOPadConfigSet(GPIO_PORTN_BASE, 
                     GPIO_PIN_5, 
                     GPIO_STRENGTH_12MA, 
                     GPIO_PIN_TYPE_STD);        //Configuration
    
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
  
    // Inicializa periféricos
    GPIOInit();
    TIMERInit();
    UARTInit();
    
    UARTprintf("Hello World do Adriano e do Davi!\n");
    
    while(1)
    {
      GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, PIN_N5_STATE); // Blink PIN N4
    }
    
} // main