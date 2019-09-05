//******************************************************************************
//  LAB 1 SISTEMAS EMBARCADOS - UTFPR 2019/2
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
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "system_TM4C1294.h" 

//
// Defines
//
#define NUM ((24000000/2)/8)
#define SAMPLES 10           // Number of samples used to get the measure
#define NCYCLESTON 5         // Number of cycles corresponding to ton count
#define NCYCLESTOFF 5        // Number of cycles corresponding to toff count

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
    UARTInit();
    GPIOInit();
    
    // Inicializa as variaveis
    int n = 0;
    int t_on[SAMPLES];
    int t_off[SAMPLES];
    int tonMed=0, toffMed=0;
    int T=0, f=0, D=0;
    
    //PORTAR PARA FUNÇÃO CLEAR VECTOR
    short i;
    for(i=0; i<SAMPLES; i++)
    {
        t_on[i]=0;
        t_off[i]=0;
    }
    
    // Main loop
    while(1)
    {
        // Sincronização
        while(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4) == GPIO_PIN_4);
        while(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4) == 0);
        
        // Medição
        do
        {
            while(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4) == GPIO_PIN_4)
                t_on[n]++;
            while(GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4) == 0)
                t_off[n]++;
            n++;
        }while(n < (SAMPLES-1));
        
        // Reseta n para nova contagem de amostras
        n = 0;
        
        // Calculo dos valores medios
        tonMed = 0;
        for(i=0; i<SAMPLES; i++)
        {
            tonMed = tonMed + t_on[i];
            toffMed = toffMed + t_off[i];
        }
        tonMed = tonMed/SAMPLES;
        toffMed = toffMed/SAMPLES;
        
        // Conversao para segundos @TODO verificar se SystemCoreClock funciona e ajustar
        tonMed = NCYCLESTON*tonMed/SystemCoreClock;
        toffMed = NCYCLESTOFF*toffMed/SystemCoreClock;
        
        // Calculo dos parametros do sinal
        T = tonMed + toffMed;
        f = 1/T;
        D = 100*tonMed/T;
        
        // Envia dos parametros por UART
        UARTprintf("T = %i s | f = %i Hz | D = %i\r\n",T,f,D);
        
        // Limpa ton e toff
        for(i=0; i<SAMPLES; i++)
        {
            t_on[i]=0;
            t_off[i]=0;
        }
        
    } // while
} // main