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
//  Inputs: PWM waveform (PIN N4).
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
#include "system_TM4C1294.h" 

//
// Defines
//

extern void UARTStdioIntHandler(void);

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
    UARTStdioConfig(0, 921600, SystemCoreClock);
} // UARTInit

void UART0_Handler(void)
{
    UARTStdioIntHandler();
} // UART0_Handler

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
    
    // Set pin PN4 to in for the PWM signal
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_4);
    GPIOPadConfigSet( GPIO_PORTN_BASE,
                      GPIO_PIN_4,
                      GPIO_STRENGTH_2MA,
                      GPIO_PIN_TYPE_STD_WPU );
}

//******************************************************************************
//
// Main
//
//******************************************************************************
void main(void)
{
    // Inicializa periféricos
    GPIOInit();
    UARTInit();
    
} // main