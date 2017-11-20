#include <xc.h>
#include <stdlib.h>
#define _XTAL_FREQ 8000000



/* 4450 PINOUT
 * AN0  = POTENTIOMETER (pin 2)
 * RA1  = SWITCH (pin 3)
 * RE0  = RELAY (pin 8)
 * RC0  = BLUE LED (pin 15)
 * RC2  = RED LED1 (pin 17)
 * RD2  = RED LED2 (pin 21)
 * RD3  = RED LED3 (pin 22)
 * RC6  = RED LED4 (pin 25)
 * RC7  = RED LED5 (pin 26)
 * RB0  = DATA0 (pin 33)
 * RB1  = DATA1 (pin 34)
 * RB2  = DATA2 (pin 35)
 * RB3  = DATA3 (pin 36)
 * RB4  = DATA4 (pin 37)
 * RB5  = DATA5 (pin 38)
 * RD6  = DATA6 (pin 29)
 * RD7  = DATA7 (pin 30)
 * RD0  = RS (pin 19)
 * RD1  = EN (pin 20)
 */


/*marker for block */
#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSC_HS// Oscillator Selection bits (Internal oscillator, port function on RA6, EC used by USB (INTIO))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

#define ROWS 2
#define COLS 16
#define RS LATD0
#define EN LATD1
#define ONLED LATCbits.LC0
#define RELAY LATEbits.LE0

#define SWITCH PORTAbits.RA1
#define PRESSED 0

#define LEDBAR1 LATCbits.LC2
#define LEDBAR2 LATDbits.LD2
#define LEDBAR3 LATDbits.LD3
#define LEDBAR4 LATCbits.LC6
#define LEDBAR5 LATCbits.LC7

#define MYTIMERLO 0
#define MYTIMERHI 0
#define LCDPSTDEL 750

// burn lengths (mS / 10)
// yeah, I flubbed these numbers. Sue me.
#define BRNTIME0 0
#define BRNTIME1 3450
#define BRNTIME2 6900
#define BRNTIME3 10350
#define BRNTIME4 13800
#define BRNTIME5 17250


int ISRUNNING = 0;
int CUR_POTVAL = 0;
int OLD_POTVAL = 0;
volatile int PRSCOUNT = 0;
volatile int OLDPRESS = 0;
int BURN_TIME = 0;
int volatile COOK_COUNT = 0;
int volatile DISP_TIMER1 = 0;
int volatile DISP_TIMER2 = 0;


void led_bar(int value)
{
    if (value <= 40)
        BURN_TIME = BRNTIME0;
    if (value > 40)
    {
        BURN_TIME = BRNTIME1;
        LEDBAR1 = 1;
    }
    else
    {
        LEDBAR1 = 0;
    }
    if (value > 80)
    {
        BURN_TIME = BRNTIME2;
        LEDBAR2 = 1;
    }
    else
    {
        LEDBAR2 = 0;
    }
    if (value > 120)
    {
        BURN_TIME = BRNTIME3;
        LEDBAR3 = 1;
    }
    else
    {
        LEDBAR3 = 0;
    }
    if (value > 160)
    {
        BURN_TIME = BRNTIME4;
        LEDBAR4 = 1;
    }
    else
    {
        LEDBAR4 = 0;
    }
    if (value > 200)
    {
        BURN_TIME = BRNTIME5;
        LEDBAR5 = 1;
    }
    else
    {
        LEDBAR5 = 0;
    }
    
}

void write_command(char data)
{
    LATB = data;
    LATD = (PORTD & ~0xc0) | (data & 0xc0);
    RS = 0;
    EN = 1;
    Nop();
    EN = 0;
    __delay_us(LCDPSTDEL);
}

void write_char(char data)
{
    LATB = data;
    LATD = (PORTD & ~0xc0) | (data & 0xc0);
    RS = 1;
    EN=1;
    Nop();
    EN=0;
    __delay_us(LCDPSTDEL);
}

void proc_string(const char *string)
{
    while((*string) != 0)
    {
        write_char(*string);
        string++;
    }
}

void string2screen(unsigned char row, unsigned char pos, const char *string)
{
    unsigned char location = 0;
    if( row <= 1 )
    {
        location = (0x80) | ((pos) & 0x0f);
        write_command(location);
    }
    else
    {
        location = (0xc0) | ((pos) & 0x0f);
        write_command(location);
    }
    proc_string(string);
}


void clear_line(unsigned char row, unsigned char pos)
{
    unsigned char location = 0;
    if( row <= 1 )
    {
        location = (0x80) | ((pos) & 0x0f);
        write_command(location);
    }
    else
    {
        location = (0xc0) | ((pos) & 0x0f);
        write_command(location);
    }
    
    for(int i=0; i < COLS; i++)
        write_char(' ');
}


void update_lcd_timer()
{
    char buf[16];
    clear_line(2, 7);
    string2screen(2,0,"TIME: ");
    itoa(buf, COOK_COUNT,10);//((COOK_COUNT*100)/60), 10);
    string2screen(2,6, buf);
}


void clear_screen()
{
    write_command(0x01);
}

void initialize()
{
    OSCCON = 0xf2;          // internal oscillator, 8MHz
    TRISA = 0b00000011;     // RA0/AN0 as input
    TRISB = 0b00000000;
    TRISD = 0b00000000;
    TRISC = 0b00000000;
    TRISE = 0b00000000;
    
    LATA = 0;
    LATB = 0;
    LATC = 0;
    LATD = 0;
    LATE = 0;
    
    ONLED = 0;
    RELAY = 0;
    
    __delay_ms(40);           // allow time for display to boot
    __delay_ms(40);           // allow time for display to boot

    write_command(0x38);    // setup for 2-line operation
    write_command(0x06);    // shift cursor
    write_command(0x0c);    // turn display on
    
    
    // initialize ADC
    ADCON0 = 0x00;          //clear ADCON0 to select channel 0 (AN0)
    ADCON1 = 0b00001110;    // AN0 analog input
    ADCON2 = 0b00110110;    // left justified, 16TAD
    ADCON0bits.ADON = 0x01; //Enable A/D module
    PIE1bits.ADIE = 1;      // enable a/d interrupt
    
    // initialize Timer0 (10mS)
    T0CONbits.PSA = 0;      // enable prescaler
    T0CONbits.T0PS2 = 0;    // 1:8 prescaler
    T0CONbits.T0PS1 = 1;    //
    T0CONbits.T0PS0 = 0;    //
    T0CONbits.T0CS = 0;     // internal clock
    T0CONbits.T08BIT = 0;   // 16 bit counter
    INTCONbits.TMR0IE = 1;  // enable interrupt
    INTCONbits.TMR0IF = 0;  // clear flag
    TMR0H = 0xB1;
    TMR0L = 0xE0;
    T0CONbits.TMR0ON = 1;   // enable timer
    
    // initialize Timer1
    T1CONbits.T1CKPS1 = 1;      // bits 5-4  Prescaler Rate Select bits 8:1
    T1CONbits.T1CKPS0 = 1;
    T1CONbits.T1OSCEN = 0;      // bit 3 Timer1 Oscillator Enable Control: bit 1=on
    T1CONbits.TMR1CS  = 0;      // bit 1 Timer1 Clock Source Select bit: 0=Internal clock (FOSC/4) / 1 = External clock from pin T1CKI (on the rising edge)
    TMR1H = MYTIMERHI;       // preset for timer1 MSB register
    TMR1L = MYTIMERLO;       // preset for timer1 LSB register
    PIR1bits.TMR1IF = 0;        // interrupt flag status
    PIE1bits.TMR1IE = 1;        // Enable timer1 interrupt
    T1CONbits.TMR1ON = 1;      //enable timer1
    
    // enable interrupts
    INTCONbits.PEIE = 1;        // enable peripheral interrupt
    INTCONbits.GIE = 1;         // Enable global interrupts
    
}


void control_relay(BYTE value)
{
    char buf[16];
    
    if (value > 0)
    {
        ONLED = 1;
        ISRUNNING = 1;
        RELAY = 1;
        clear_screen();
        string2screen(1,0,"HEATER ON");
        string2screen(2,0,"TIME: ");
        itoa(&buf, ((COOK_COUNT*100)/60), 10);
        string2screen(2,6, buf);
    }
    else
    {
        ONLED = 0;
        ISRUNNING = 0;
        RELAY = 0;
        clear_screen();
        string2screen(1,0,"HEATER OFF");
        string2screen(2,0,"TIME: 0");
    }
}


void interrupt isr()
{
    static BYTE running = 0;
    int status;
    char buf[16];
    
    // Timer0 interrupt
    if (INTCONbits.TMR0IF)
    {
        TMR0H = 0xB1;
        TMR0L = 0xE0;
        
        led_bar(CUR_POTVAL);
        
        // check if we're running & control relay
        // (only if burn time is off)
        if ((ISRUNNING) && (BURN_TIME > 0))
        {
            if (COOK_COUNT == 0)
            {
                control_relay(0);
            }
            
            if (COOK_COUNT >= 0)
            {
                COOK_COUNT--;
            }    
        }
        
        // start ADC conversion (if not running)
        if (!ISRUNNING)
            ADCON0bits.GO = 1;
        
        INTCONbits.TMR0IF = 0;
    }
    
    // Timer1 interrupt
    if (PIR1bits.TMR1IF)
    {
        
        TMR1L = MYTIMERLO;
        TMR1H = MYTIMERHI;
        
        if((ISRUNNING) && (BURN_TIME > 0))
            update_lcd_timer();
        
        if (DISP_TIMER1 >= 0)
            DISP_TIMER1--;
        else if ((DISP_TIMER1 < 0) && (DISP_TIMER2 >= 0))
            DISP_TIMER2--;
        
        // display timers
        if (DISP_TIMER1 == 0)
        {
            clear_screen();
            string2screen(1,0,"The Current");
            string2screen(2,0,"Source");
        }
        else if (DISP_TIMER2 == 0)
        {
            control_relay(0);
        }
        
                
        // switch debounce
        if ((SWITCH == PRESSED) && (OLDPRESS == PRESSED))
        {
            PRSCOUNT++;
            if (PRSCOUNT == 3)
            {
                running = !running;
                control_relay(running);
                
                // set cook time if button on, else reset
                if (BURN_TIME > 0)
                {
                    if (ISRUNNING)
                        COOK_COUNT = BURN_TIME;
                    else
                        COOK_COUNT = 0;
                }
                PRSCOUNT = 0;
            }
        }
        else
        {
            PRSCOUNT = 0;
            OLDPRESS = !PRESSED;
        }
        OLDPRESS = SWITCH;     
        PIR1bits.TMR1IF = 0; 
    }
    
    // ADC interrupt
    if (PIR1bits.ADIF) 
    {
        CUR_POTVAL = ADRESH;
        PIR1bits.ADIF = 0;
    }
}



void main(void)
{	    
    initialize();
    
    DISP_TIMER1 = 6;
    DISP_TIMER2 = 6;
    
    clear_screen();
    string2screen(1,0,"Nichrome Wire");
    string2screen(2,0,"Acrylic Bending");
          
    while(1)
    {

    }
     
}
