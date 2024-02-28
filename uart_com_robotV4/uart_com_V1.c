#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#define "utils/uartstdio.h"

int32_t g_ui32SysClock;
char data[20]="";
uint32_t w_left = 0;
uint32_t w_right = 0;
uint32_t pulses = 0;
uint32_t pulses2 = 0;
int32_t error = 0;
int32_t error2 = 0;
uint8_t w = 0;
uint8_t s = 0;
uint8_t l = 0;
uint8_t r = 0;

void
GPIOEIntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = MAP_GPIOIntStatus(GPIO_PORTE_BASE, true);

    MAP_GPIOIntClear(GPIO_PORTE_BASE, ui32Status);
    UARTCharPut(UART0_BASE, 'M');
    pulses++;

    if(pulses >= w_left){
            //MOTOR 1 
         

            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0x00);
            
        }
}

void
GPIOGIntHandler(void)
{
    uint32_t ui32Status;

    ui32Status = MAP_GPIOIntStatus(GPIO_PORTG_BASE, true);

    MAP_GPIOIntClear(GPIO_PORTG_BASE, ui32Status);

    pulses2++;

    if(pulses2 >= w_right){
            //MOTOR 2


            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x00);
            
        } 
}

void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);
	uint8_t idx=0;
    MAP_UARTIntClear(UART0_BASE, ui32Status);
    while(MAP_UARTCharsAvail(UART0_BASE))
    {
	data[idx]=MAP_UARTCharGetNonBlocking(UART0_BASE);//tomamos el dato
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);
        SysCtlDelay(g_ui32SysClock / (1000 * 3));
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0);
        idx++;
    }
        
        
        if(data[0]=='U' && data[1]=='P') 
        {
            
            pulses = 0;
            pulses2 = 0;

            w_left = 100000;
            w_right = 100000;
            SysCtlDelay(g_ui32SysClock / (1000));
            //MOTOR 1      
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, GPIO_PIN_2);
            //MOTOR 2
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7);
            
            

            
        }
        if(data[0]=='D' && data[1]=='O'&& data[2]=='W'&& data[3]=='N') 
        { 
            pulses = 0;
            pulses2 = 0;

            w_left = 100000;
            w_right = 100000;
            SysCtlDelay(g_ui32SysClock / (1000));
            //MOTOR 1      
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, GPIO_PIN_1);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0x00);
            //MOTOR 2
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x00);
            
        }
        if(data[0]=='L' && data[1]=='E'&& data[2]=='F'&& data[3]=='T') 
        { 
            pulses = 0;
            pulses2 = 0;

            w_left = 100000;
            w_right = 100000;
            SysCtlDelay(g_ui32SysClock / (1000));
            //MOTOR 1      
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, GPIO_PIN_2);
            //MOTOR 2
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, GPIO_PIN_6);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x00);
            
        }
        if(data[0]=='R' && data[1]=='I'&& data[2]=='G'&& data[3]=='T'&& data[4]=='H')  {
            pulses = 0;
            pulses2 = 0;

            w_left = 100000;
            w_right = 100000;
            SysCtlDelay(g_ui32SysClock / (1000));
            //MOTOR 1      
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, GPIO_PIN_1);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0x00);
            //MOTOR 2
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, GPIO_PIN_7);
            
        }
        if(data[0]=='S' && data[1]=='T'&& data[2]=='O'&& data[3]=='P')  {
        	//MOTOR 1      
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0x00);
            //MOTOR 2
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0x00);
            MAP_GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0x00);
            SysCtlDelay(g_ui32SysClock / (1000 * 3));
            w_left = 0;
            w_right = 0;
            
        }
}
void pwm_config()
{
    uint32_t ui32PWMClockRate;

    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);
    // The PWM peripheral must be enabled for use.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //
    // Enable the GPIO port that is used for the PWM output.

    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    MAP_GPIOPinConfigure(GPIO_PK4_M0PWM6);
    MAP_GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4); //pulses
    MAP_GPIOPinConfigure(GPIO_PK5_M0PWM7);
    MAP_GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_5); //pulses2
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);

    ui32PWMClockRate = g_ui32SysClock / 8;

    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);


    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, (ui32PWMClockRate / 250));

    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,
                         MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 4);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,
                         MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_3) / 4);

    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}


int main(void){
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_240), 120000000);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_IntMasterEnable();
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    MAP_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	
    pwm_config();

    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_1);//HABILITAMOS PK1
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_2);//HABILITAMOS PK2
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_6);//HABILITAMOS PK6
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_7);//HABILITAMOS PK7

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //motor1
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1,
                    GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);//motor2
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTG_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTG_BASE, GPIO_PIN_0,
                    GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOIntTypeSet(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_FALLING_EDGE); //pulses ++
    GPIOIntRegister(GPIO_PORTE_BASE,&GPIOEIntHandler);
    MAP_IntEnable(INT_GPIOF);
    MAP_GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_1); //pulses ++

    GPIOIntTypeSet(GPIO_PORTG_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE); //pulses2 ++
    GPIOIntRegister(GPIO_PORTG_BASE,&GPIOGIntHandler);
    MAP_IntEnable(INT_GPIOG);
    MAP_GPIOIntEnable(GPIO_PORTG_BASE, GPIO_INT_PIN_0); //pulses2 ++
    uint32_t kp = 50;
    

    uint32_t u = 0;
    uint32_t y = 0;
 
    while(1){
        
        error = pulses2 - pulses;
        error2 = pulses - pulses2;
        
        u = 20000+kp*error;
        y = 20000+kp*error2;


        if(u<=0){
            u = -1*u;
        }
        if(u>=59999){
            u = 59999;
        }
        
        if(y<=0){
            y = -1*y;
        }
        if(y>=59999){
            y = 59999;
        }
        //y = kp+kp*error1;

        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,u); //pulses
        MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7,y); //pulses2
        
    }
}