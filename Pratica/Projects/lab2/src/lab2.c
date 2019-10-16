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
//  Inputs: PWM waveform (PIN PL5).
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
#include "inc/hw_types.h"
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
#include "driverlib/pwm.h"

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
int timerCount = 0; // Get current timer value obtained by capture mode
int timerCountLast = 0;
int ton = 0, toff = 0;
int T = 10;

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
//  Configure the PWM peripheral
//
//******************************************************************************
void PWMInit (void)
{
    // Enable the PWM0 peripheral.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)); // Wait until done 
    
    // Set the PWM clock as an division of the system clock. In this example, 
    // the PWM clock mathces the system clock.
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);
    
    // Configure the PWM generator for count down mode
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    
    // Set the Period (expressed in clock ticks). For Example, in order to make
    // a PWM clock with 10kHZ, is used 12000 clock ticks.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1500);
    
    // Set the pulse width of PWM0 for a 30% duty cycle.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 450);

    // Set the pulse width of PWM1 for a 30% duty cycle.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 450);
    
    // Enable the PWM generator
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    
    // Enable the outputs.
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);    
}
//******************************************************************************
//
//  TIMER Configuration
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

void TimerB0Isr(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);  // Clear timer interrupt
    HWREG(TIMER0_BASE + 0x050) = 0xFFFF;  // Reset Timer0A counting
    PIN_N5_STATE ^= GPIO_PIN_5;
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, PIN_N5_STATE); // Blink PIN N4
    timerCount = TimerValueGet(TIMER0_BASE, TIMER_B);
}

void TIMERInit()
{
    // Enable the Timer0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    // Wait for the Timer0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){}

    // Configure TimerA as a half-width one-shot timer, and TimerB as a
    // half-width edge capture counter.
    TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_ONE_SHOT_UP |
    TIMER_CFG_B_CAP_TIME_UP));
    
    // Set the count time for the the one-shot timer (TimerA).
    TimerLoadSet(TIMER0_BASE, TIMER_A, SystemCoreClock*TIMER_MS/1000);   //1ms
    
    //TimerLoadSet(TIMER0_BASE, TIMER_B, 0xFFFF);
    //TimerMatchSet(TIMER0_BASE, TIMER_B, 0x0);
    
    // Configure the counter (TimerB) to count both edges.
    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_BOTH_EDGES);
    
    // Registering ISRs
    TimerIntRegister(TIMER0_BASE, TIMER_A, TimerA0Isr);
    TimerIntRegister(TIMER0_BASE, TIMER_B, TimerB0Isr);
    
    // Enable timer interrupt
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);
    
    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL); // Set GPIO L (PL5 - PWM)
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
    
    // Set pin PL5 to timer capture for signal in
    GPIOPinConfigure(GPIO_PL5_T0CCP1);  
    GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_5);
    
    // Set pins PF0 and PF1 to out for PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Set GPIO F (PWM)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)); // Wait until done
    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

//******************************************************************************
//
// Main
//
//******************************************************************************
void main(void)
{
    // Inicializa periféricos
    FPUEnable();
    FPULazyStackingEnable();
    IntMasterEnable();
    GPIOInit();
    TIMERInit();
    UARTInit();
    PWMInit();
    
    // Variaveis
    float ton_f=0, toff_f=0;    // Tempo ligado e tempo desligado
    float Tsec=10, fHz=0, D=0;  // Parametros a serem exibidos na tela
    char T_str[10], f_str[10], D_str[10];
    //float teste = 0;          // Para testes
    //char teste_str[10];       // Para testes
    
    // Mensagem de Inicio
    UARTprintf("Hello World do Adriano e do Davi!\n");
    
    while(1)
    {        
        // Sincronizacao
        while(GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_5) == GPIO_PIN_5){}
        while(GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_5) == 0){}
        timerCountLast = timerCount;
        
        // Detecta a borda de subida
        while(GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_5) == GPIO_PIN_5){}
        if (timerCount > timerCountLast)
            ton = timerCount - timerCountLast;
        else
            ton = timerCount + 65536 - timerCountLast;
      
        // Detecta borda de descida
        while(GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_5) == 0){}
        if (timerCount > timerCountLast)
            T = timerCount - timerCountLast;
        else
            T = timerCount + 65536 - timerCountLast;
        
        // Calculo do tempo desligado
        toff = T - ton;
        
        // Conversao de ton e toff para MICRO segundos
        ton_f = (float)((1000000*(float)ton)/SystemCoreClock);
        toff_f = (float)((1000000*(float)toff)/SystemCoreClock);
        
        // Calculo dos parametros
        Tsec = ton_f + toff_f;
        fHz = (float)1000000/Tsec;
        D = (float)100*ton/T;
        
        // Envia os parametros por UART
        sprintf(T_str,"%.2f",Tsec);
        sprintf(f_str,"%.2f",fHz);
        sprintf(D_str,"%.2f",D);
        UARTprintf("T = %s us | f = %s Hz | D = %s \n",T_str,f_str,D_str);
        while( UARTBusy(UART0_BASE) ){}
        
        // Para testes
        //teste = D;
        //sprintf(teste_str,"%.2f",teste);
        //UARTprintf("teste_str = %s \n",teste_str);
        //while( UARTBusy(UART0_BASE) ){}
        
    }
} // main
