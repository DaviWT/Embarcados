/*
LAB 1 SISTEMAS EMBARCADOS - UTFPR 2019/2
Alunos:
-Adriano Ricardo de Abreu Gamba
-Davi Wei Tokikawa

Demodulator PWM
Output: Duty-Cycle, Period and Frequency.
Input: PWM waveform (PIN N4)
*/

#include <stdint.h>
#include <stdbool.h>
// includes da biblioteca driverlib
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#include "system_TM4C1294.h" 

#define NUM ((24000000/2)/8)
#define SAMPLES 10

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

void GPIOInit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION); // Habilita GPIO N (LED D1 = PN1, LED D2 = PN0)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)); // Aguarda final da habilitação
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTN_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void main(void)
{
    // Inicializa periféricos
    UARTInit();
    GPIOInit();
    
    int n = 0;
    int t_on[SAMPLES];
    int t_off[SAMPLES];
    
    //PORTAR PARA FUNÇÃO CLEAR VECTOR
    short i;
    for(i=0;i<SAMPLES;i++)
    {
      t_on[i]=0;
      t_off[i]=0;
    }
    
    while(1)
    {
        //sincronização
        while(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4) == GPIO_PIN_4);
        while(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4) == 0);
        
        do
        {
          //medição
          while(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4) == GPIO_PIN_4)
              t_on[n]++;
          while(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4) == 0)
              t_off[n]++;
          n++;
        }while(n<SAMPLES-1);
        
        n = 0;
        //CALCULO DAS MEDIAS DOS TON e TOFF
        //CONVERSAO PARA SEGUNDOS
        //CALCULO DE DUTY-CYCLE, PERIODO e FREQUENCIA
        //ENVIO PELA UART
        //LIMPA TON e TOFF
        
    } // while
} // main