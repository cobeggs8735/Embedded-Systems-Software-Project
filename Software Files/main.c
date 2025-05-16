#include "msp.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

void portInit(void);    //
void UARTInit(void);    //
void uprint(char *x);   //
void commandLCD(unsigned char);   //
void writeLCD(unsigned char);   //
void LCDInit(void);     //
void Timer_A0Init(void);    //
void Timer_A2Init(void);    //
void ADCInit(void);     //
void clearLCD(int line);    //

unsigned int servoMin = 3000;   // Minimum signal value
unsigned int servoCenter = 4500;   // Center of Servo value
unsigned int servoMax = 6000;  // Maximum signal value
unsigned int PWM_Load;  // Value loaded to PWM
unsigned int PWM_Max = 60000;   // Period control of PWM
unsigned int NADC;  // ADC Raw Value

int choiceMenu;     // Main menu choice

unsigned char line1 = 0x80;     // Home Address of line 1
unsigned char line2 = 0xC0;     // Home address of line 2

unsigned char project[] = "Term Project";   //
unsigned char names[] = "Preston&Coleman";    //
unsigned char date[] = "04/21/2021";    //
unsigned char center[] = "Servo Center";    //
unsigned char left[] = "Servo Left";    //
unsigned char right[] = "Servo Right";  //
unsigned char control[] = "Servo Control";  //
unsigned char automatic[] = "Automatic";    //
unsigned char tcap_string[6];    //

char menu[] = "\n\r\tESET 369 Term Project\n\r"
                    "1. Your Names?\n\r2. Date?\n\r"
                    "3. Servo (Center)\n\r"
                    "4. Servo (Left)\n\r"
                    "5. Servo (Right)\n\r"
                    "6. Control Servo\n\r"
                    "7. Ultrasonic Sensor\n\r"
                    "8. Automatic\n\r"
                    "9. Menu\n\n\r";     //
char choice[2];    //

volatile unsigned int tcap = 0;     //
volatile unsigned int tcapFlag = 0;     //
volatile unsigned int tcapCov = 0;  //

void main(void)
{
    int i;  //
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	// Configures
	UARTInit();     // UART Configure
	portInit();     // Port Configures
	LCDInit();  // LCD Configure
	Timer_A0Init();     // Timer_A0 Configure
	Timer_A2Init();     // Timer_A2 Configure
	ADCInit();  // ADC14 Configure
	// End Configures

	// Interrupts Configure
	// ADC14
	ADC14->IER0 |= ADC14_IER0_IE0;  // ADC14 ISR Set Up

	// TIMER_A0
	TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;     // TA0.0 Enable interrupt
	TIMER_A0->CCTL[1] = TIMER_A_CCTLN_CCIE;     // TA0.1 Enable interrupt

	// TIMER_A2
	TIMER_A2->CTL |= TIMER_A_CTL_IE;    // TA2 Interrupt Enable

	NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 24);   // ADC NVIC Set Up
    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);   // TA0.0 NVIC Set Up
    NVIC->ISER[0] |= 1 << ((TA0_N_IRQn) & 31);   // TA0.1 NVIC Set Up
    NVIC->ISER[0] = 1 << ((TA2_N_IRQn) & 31);   // TA2 NVIC Set Up
	__enable_irq();   // Enable Global Interrupt
	// End Interrupts Configure

	for(i = 0; i < strlen(project); i++)
	{
	    commandLCD(line1+i);    // Get character position
	    writeLCD(project[i]);   // Print character on LCD
	}

	// Print original menu
	uprint(menu);   //

	while(1)
	{
	    i = 0;  // Reset iteration variable

	    // Get menu choice
	    while(1)
	    {
            if ((EUSCI_A0->IFG & 0x01) != 0)
            {
                choice[i] = EUSCI_A0->RXBUF; // Extract the byte
                EUSCI_A0->TXBUF = choice[i]; // Echo the byte

                while ((EUSCI_A0->IFG & 0x02) == 0)
                {
                    // Wait
                }
                if (choice[i] == '\r')
                {
                    choice[i] = '\0';  // Terminator character
                    break; // Must Terminate the loop
                }
                else
                {
                    i++;
                }
            }
	    }

	    choiceMenu = atoi(choice);     // Convert the string to an int

	    switch(choiceMenu)
	    {
            case(1):   // Display Names on LCD
                    clearLCD(1);    // clear line 1
                    uprint(names);  // print names to the console
                    uprint("\n\r");     //
                    for (i = 0; i < strlen(names); i++)
                    {
                        commandLCD(line1+i);    // position of character
                        writeLCD(names[i]);     // write character of name
                    }
                    break;  //

            case(2):    // Display Date to LCD
                    clearLCD(1);    // clear line 1

                    uprint(date);   // Print date to console
                    uprint("\n\r");     //

                    for (i = 0; i < strlen(date); i++)
                    {
                        commandLCD(line1+i);    // Positon of character
                        writeLCD(date[i]);  // write character of the date
                    }
                    break;  //

            case(3):    // Servo Center
                    clearLCD(1);    // clear LCD line 1

                    uprint(center);     // Print center to console
                    uprint("\n\r");     //

                    for (i = 0; i < strlen(center); i++)
                    {
                        commandLCD(line1+i);    // character position
                        writeLCD(center[i]);    // write character
                    }

                    TIMER_A0->CCR[1] = servoCenter;    //
                    break;  //

            case(4):    // Servo Left
                    clearLCD(1);    // clear LCD line 1

                    uprint(left);   // print to console
                    uprint("\n\r");     //

                    for (i = 0; i < strlen(left); i++)
                    {
                        commandLCD(line1+i);    // character position
                        writeLCD(left[i]);  // write character
                    }

                    TIMER_A0->CCR[1] = servoMax;    //
                    break;  //

            case(5):    // Servo Right
                    clearLCD(1);    // clear LCD line 1

                    uprint(right);  // print right to the console
                    uprint("\n\r");     //

                    for (i = 0; i < strlen(right); i++)
                    {
                        commandLCD(line1+i);    // character position
                        writeLCD(right[i]);     // wirte character
                    }

                    TIMER_A0->CCR[1] = servoMin;    //
                    break;  //

            case(6):    // Control Servo with ADC
                    clearLCD(1);    // clear LCD line 1

                    uprint(control);    // pritn to console
                    uprint("\n\r");     //

                    for (i = 0; i < strlen(control); i++)
                    {
                        commandLCD(line1+i);    // character position
                        writeLCD(control[i]);   // write character
                    }

                    while((P1->IN & 0x02) != 0)
                    {
                        ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;  // Start ADC Conversion
                        // Change Servo Position
                        TIMER_A0->CCR[1] = PWM_Load;    // Potentiometer Position
                    }
                    break;  //

            case(7):    // Ultrasonic Sensor
                    clearLCD(1);    // clear line 1
                    __delay_cycles(100);    // delay

                    while((P1->IN & 0x02) != 0)
                    {
                        tcapFlag = 0;   // clear flag
                        tcapCov = 0;    // clear Cov

                        TIMER_A2->CTL |= TIMER_A_CTL_CLR;   // clear TA2 CCR

                        P3->OUT |= 0x20;    // Set P3.5
                        __delay_cycles(30);     // delay for trigger signal
                        P3->OUT &= ~0x20;   // clear P3.5
                        __delay_cycles(100);    // delay

                        while(tcapFlag == 0)
                        {
                            // wait for flag to be set
                        }

                        if (tcapCov == 0)
                        {
                            sprintf(tcap_string, "%d", tcap);   // Convert tcap to string

                            for (i = 0; i < 5; i++)   // strlen(tcap_string)
                            {
                                commandLCD(line1+i);    // display on line 2
                                writeLCD(tcap_string[i]);    // write character on line 2

                                // Less than 5 digits
                                if (tcap < 10000)
                                {
                                    commandLCD(line1+4);    // 5th digit position
                                    writeLCD(' ');  // write a blank
                                }

                                // Less than 4 digits
                                if (tcap < 1000)
                                {
                                    commandLCD(line1+3);    // 4th digit position
                                    writeLCD(' ');  // write a blank
                                }
                            }
                        }
                        else
                        {
                            // Do Nothing
                        }
                    }
                    break;  //

            case(8):    // Automatic Ultrasonic
                    // Display Automatic on Terminal
                    clearLCD(1);    // clear line 1

                    uprint(automatic);  //
                    uprint("\n\r");    //

                    for (i = 0; i < strlen(automatic); i++)
                    {
                        commandLCD(line1+i);    // character position
                        writeLCD(automatic[i]);     // write character
                    }

                    while((P1->IN & 0x02) != 0)
                    {
                        // Automatic
                        tcapFlag = 0;   // clear flag
                        tcapCov = 0;    // clear Cov

                        TIMER_A2->CTL |= TIMER_A_CTL_CLR;   // clear TA2 CCR

                        P3->OUT |= 0x20;    // Set P3.5
                        __delay_cycles(30);     // delay for trigger signal
                        P3->OUT &= ~0x20;   // clear P3.5
                        __delay_cycles(100);    // delay

                        while(tcapFlag == 0)
                        {
                            // wait for flag to be set
                        }

                        if (tcapCov == 0)
                        {
                            // Object within 2-3 inches
                            if (tcap < 5000)
                            {
                                TIMER_A0->CCR[1] = servoMin;    // Tilt servo
                            }

                            // Object outside 2-3 inches
                            else
                            {
                                TIMER_A0->CCR[1] = servoCenter;     // Center Servo
                            }
                        }
                        else
                        {
                            // Do Nothing
                        }
                    }
                    break;  //

            case(9):    //
                    uprint(menu);   // display menu on console

                    clearLCD(1);    //

                    for (i = 0; i < strlen(project); i++)
                    {
                        commandLCD(line1+i);     // line 1 position
                        writeLCD(project[i]);  // write position index
                    }
                    break;  //

            default:    //
                // Do Nothing
                break;  //
	    }
	}
}

void portInit(void)
{
    // Port 1 Configure
    // Inputs
    P1->SEL0 &= ~0x02;  //
    P1->SEL1 &= ~0x02;  // GPIO
    P1->DIR &= ~0x02;    // P1.1 Input
    P1->REN |= 0x02;    //
    P1->OUT |= 0x02;    //

    // UART
    P1->SEL0 |= 0x0C;  //
    P1->SEL1 &= ~0x0C;   // UART P1.2, P1.3
    // End Port 1 Configure

    // Port 2 Configure
    P2->SEL0 &= ~0xFF;  //
    P2->SEL1 &= ~0xFF;  // GPIO
    P2->DIR |= 0xFF;    // Port 2 Outputs
    P2->OUT &= ~0xFF;   // Clear Outputs
    // End Port 2 Configure

    // Port 3 Configure
    // Outputs
    P3->SEL0 &= ~0x21;  //
    P3->SEL1 &= ~0x21;  // GPIO P3.5, 3.0
    P3->DIR |= 0x21;    // P3.5, 3.0 Output
    P3->OUT &= ~0x21;    // Clear the Outputs
    // End Port 3 Configure

    // Port 4 Configure
    P4->SEL0 |= 0x04;   //
    P4->SEL1 |= 0x04;   // ADC14
    // End Port 4 Configure

    // Port 5 Configure
    // Timer_A
    P5->DIR &= ~0x80;    //
    P5->SEL0 |= 0x80;   //
    P5->SEL1 &= ~0x80;   // P5.7 = TA2.2
    //P5->DIR &= ~0x80;    //
    // End Port 5 Configure

    // Port 6 Configure
    P6->SEL0 &= ~0xC1;  //
    P6->SEL1 &= ~0xC1; // GPIO
    P6->DIR |= 0xC1;    // P6.7, 6.6, 6.0
    P6->OUT &= ~0xC1;   // Clear Outputs
    // End Port 6 Configure

    return;     //
}

void UARTInit(void)
{
    // Configure UART
    EUSCI_A0->CTLW0 |= 0x01; // Go to reset state
    EUSCI_A0->MCTLW = 0x00; // Set Baud Rate error
    EUSCI_A0->CTLW0 |= 0x80; // 1 stop bit, no parity, system clock, 8-bit
    EUSCI_A0->BRW = 26; // Baud Rate: 26

    // Disable Reset State
    EUSCI_A0->CTLW0 &= ~0x01; // Out of Reset State

    return;     //
}

void uprint(char *x)
{
    int n; // Loop Variable
    for(n = 0; n < strlen(x); n++)
    {
        EUSCI_A0 -> TXBUF = x[n]; // Char of x to print
        while ((EUSCI_A0 -> IFG & 0x02) == 0) // Print that Char
        {
            // Wait to exit the buffer
        }
    }

    return ;
}

void Timer_A0Init(void)
{
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_1 | TIMER_A_CTL_CLR;  //Timer_A2 Set Up

    TIMER_A0->CCR[0] = PWM_Max;    // PWM Period Control
    TIMER_A0->CCR[1] = 60000;    // Software PWM

    return;     //
}

void TA0_0_IRQHandler(void)
{
    if ((TIMER_A0->CCTL[0] & TIMER_A_CCTLN_CCIFG) != 0)
    {
        P3->OUT |= 0x01;    // set P3.0 Servo
        TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  // clear IFG Flag
    }

    return;     //
}
void TA0_N_IRQHandler(void)
{
    if((TIMER_A0->CCTL[1] & TIMER_A_CCTLN_CCIFG) != 0)
    {
        P3->OUT &= ~0x01;   // clear P3.0 Servo
        TIMER_A0->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;  // clear IFG Flag
    }

    return;     //
}

void Timer_A2Init(void)
{
    TIMER_A2->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC_2 | TIMER_A_CTL_CLR;  // TA2 - SMCLK, Continuous, Enable
    TIMER_A2->CCTL[2] = TIMER_A_CCTLN_CM_2 | TIMER_A_CCTLN_CCIS_0 | TIMER_A_CCTLN_CCIE | TIMER_A_CCTLN_CAP | TIMER_A_CCTLN_SCS;  // TA2 CCTL[2] Set Up

    return;     //
}

void TA2_N_IRQHandler (void)
{
    if ((TIMER_A2->CTL & TIMER_A_CTL_IFG) != 0)
    {
        tcapCov = 1;    // Set tcap overflow flag
        tcapFlag = 1;   // Set tcap flag
        TIMER_A2->CTL &= ~TIMER_A_CTL_IFG;     // clear TA2 IFG flag
    }

    if ((TIMER_A2->CCTL[2] & TIMER_A_CCTLN_CCIFG) != 0)
    {
        tcap = TIMER_A2->CCR[2];    // Store TA CCR[2] in tcap
        tcapFlag = 1;   // set tcap flag
        TIMER_A2->CCTL[2] &= ~TIMER_A_CCTLN_CCIFG;   // claer TA2 IFG flag
    }

    return;     //
}

void ADCInit(void)
{
    ADC14->CTL0 = ADC14_CTL0_SHT0_6 | ADC14_CTL0_SHP | ADC14_CTL0_ON;  // ADC14 Set Up
    ADC14->CTL1 = ADC14_CTL1_RES_3;     // 14-bit Resolution
    ADC14->MCTL[0] = ADC14_MCTLN_INCH_11;  // Input Channel 11

    return;     //
}

void ADC14_IRQHandler(void)
{
    NADC = ADC14->MEM[0];   // Update PWM_Load with ADC
    PWM_Load = ((float)NADC*((float)servoMax - (float)servoMin)/(float)16383) + (float)servoMin;   // Do some interpolation for PWM_Load

    return;     //
}

void commandLCD(unsigned char in)
{
    P2->OUT = in;   //

    P6->OUT &= ~0x80;   // Clear RS
    P6->OUT &= ~0x40;   // Clear R/W

    P6->OUT |= 0x01;    // Set E
    __delay_cycles(2000);   //
    P6->OUT &= ~0x01;    // Clear E

    return;     //
}

void writeLCD(unsigned char in)
{
    P2->OUT = in;   //

    P6->OUT |= 0x80;    // Set RS
    P6->OUT &= ~0x40;   // Clear R/W

    P6->OUT |= 0x01;    // Set E
    __delay_cycles(2000);    //
    P6->OUT &= ~0x01;   // Clear E

    return;     //
}

void LCDInit(void)
{
    P6->OUT &= ~0x01;   // Clear E
    __delay_cycles(3000);   //

    commandLCD(0x30);  // Wake up
    __delay_cycles(400);    //
    commandLCD(0x30);  // Wake up
    __delay_cycles(400);    //
    commandLCD(0x30);  // Wake up
    __delay_cycles(400);    //

    commandLCD(0x38);  // function set: 8-bit / 2 line
    commandLCD(0x10);  // Set cursor
    commandLCD(0x0C);  // display on, cursor off
    commandLCD(0x06);  // Entry Mode Set
    commandLCD(0x01);  // Clear Display
    __delay_cycles(5000);   //

    return;     //
}

void clearLCD(int line)
{
    int i;  //
    unsigned int base;  //

    if (line == 1)
    {
        base = 0x80;    // base address of line 1
    }
    else
    {
        base = 0xC0;    // base address of line 2
    }

    for (i = 0; i < 0x0F; i++)
    {
        commandLCD(base+i);    // LCD index
        writeLCD(' ');  // clear LCD position
    }

    return;     //
}
