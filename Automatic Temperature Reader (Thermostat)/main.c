/* ELEC 315 FINAL PROJECT
 * MSP430G2553-UsluKukla
 *
 *  Created on: 08 Jun 2018
 *  Author: Mustafa Ayberk Kale 50359
 *
 *
 *  Reference ADC10 Code: coder-tronics.com/msp430-adc-tutorial , Example 3 code
 */
#include <msp430.h>
#include <math.h>

// Global Variables
unsigned int adc[6] = {0};  // This will hold the adc values from A5 to A0
unsigned int ADCVAL3 = 0; //adc A3 value
unsigned int ADCVAL5 = 0;//adc A5 value

unsigned int ADC_value=0;//adc a3 value
int ADC_res_value=0;//ntc resistance value on p1.3
int ADC_volt_value=0;//ntc volt value on p1.3
unsigned int ADC_temp_value=0;//ntc temp value from ntc characterists formula p1.3

unsigned int ADC_set_temp_value=0;//acd a5 value 0-1023 mapped btw 15-25


//Function
void Setup_HW(void);        // Setup watchdog timer, clockc, ADC ports
void Read_Acc(void);    // This function reads the ADC and stores them to adc array
unsigned char str[7];//disp value int to str temperature value string
unsigned char str_set[7];//disp value int to str setted temperature value string

void ConfigureAdc(void);// configure adc to measure INCH 5 to 0

void main(void)
{// main function
    {
        WDTCTL = WDTPW + WDTHOLD;       // Stop WDT
        BCSCTL1 = CALBC1_1MHZ;          // Set range   Basic Clock System Control Register
        BCSCTL2 &= ~(DIVS_3);           // SMCLK = DCO = 1MHz SUBSYSTEM MASTER CLK  Divider for SMCLK

        P1DIR = BIT0+BIT6;                   // P1.0(LED) AND P1.6(BUZZER) output
        //Disp Related
        DCOCTL = CALDCO_1MHZ; //DCO control register FREQUENCY


        P2DIR |= 0x39;                  // Set P2.0(SER), P2.3(SCK) P2.4(RCK) to output direction , P2.5 motor as output


        initlcd();// Lcd initiliazer
        //
        Setup_HW(); // adc setup

        //PushButton interrupt
        P2IE |= BIT1; // P2.1 interrupt enabled

        P2IFG &= ~BIT1; // P2.1 IFG cleared

        //
        __enable_interrupt();           // Enable interrupts.


        while(1)
        {//main loop
            //__delay_cycles(1000);               // Wait for ADC Ref to settle
            Read_Acc(); // This function reads the ADC and stores them to adc
           // ADC_value = ADC10MEM;        // Assigns the value held in ADC10MEM to the integer called ADC_value
            ADC_res_value=((3.3*10000*1023)/(3*ADCVAL3))-10000; //adc1.3 res value
            ADC_volt_value=ADCVAL3*3/1023;//adc1.3 volt value
            ADC_temp_value=(1/(((log(ADC_res_value/10000.0))/3528.0)+(1/298.0)))-273.0;//NTC adc 1.3 value to temp

            float Range= (30.0-15.0)/(1023.0-0.0);//MAP 0 1023 TO 15C 25C trimpot p1.5 adc value
            ADC_set_temp_value=((ADCVAL5-0)*Range)+15.0;//trimpot p1.5 adc value to temperature


            //control
            if(ADC_temp_value>ADC_set_temp_value){//control ambient temp vs set temp
                P1OUT = 0x41;                    //  reset ,turn on LED and Buzzer
                P2OUT=0x20;//turn on motor
                DatatoLCDtemp(ADC_temp_value);//write temperature to LCD
            }else{
                P1OUT = 0x00;                    //  reset ,turn off LED and buzzer
                P2OUT= 0x00;                    //  reset ,turn off motor
                DatatoLCDtemp(ADC_temp_value);//write temperature to LCD
            }
        }
    }
}


#pragma vector=ADC10_VECTOR // ADC10 interrupt service routine
__interrupt void ADC10_ISR(void)
{
    __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR) Status Register Exit LPM0
}

void Setup_HW(void) //acd 10 setup
{

    ADC10CTL1 = INCH_5 + CONSEQ_1;            // A5/A4/A3/A2/A1/A0, single sequence
    ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;//16 × ADC10CLKs(samp& hold time), multiple sample
                                                    //& conversion,adc on,adc interrupt enabled
    ADC10DTC1 = 0x06;                         // 6 conversions
    ADC10AE0 |= BIT3+BIT5;                    // Disable digital I/O on P1.3 to P1.5
}
void Read_Acc(void)
{
    ADC10CTL0 &= ~ENC;                      //disable conversion
    while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
    ADC10SA = (unsigned int)adc;            // Copies data in ADC10SA to unsigned int adc array
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start

    ADCVAL5 = adc[0];                        // adc array 0 copied to the variable ADCVAL5
    ADCVAL3 = adc[2];                        // adc array 2 copied to the variable ADCVAL3

    __bis_SR_register(CPUOFF + GIE);        // ENTER LPM0(low power mode off), ADC10_ISR will force exit, GENERAL INTER. EN.
}




#pragma vector=PORT2_VECTOR// Port 2 interrupt service routine
__interrupt void Port_2(void)
{
    DatatoLCDsetTemp(ADC_set_temp_value);//write set value to screen
    __delay_cycles(700000);               // show time in display

    P2IFG &= ~BIT1; // P2.1 IFG cleared, input flag clear
    P2IES ^= BIT1; // toggle the interrupt edge,
    // the interrupt vector will be called
    // when P2.1 goes from HitoLow as well as
    // LowtoHigh
}


void DatatoLCDtemp(unsigned int val){// temperature daya to lcd


    writedata(0x0001, 0);//clear display


    __delay_cycles(2000);//wait

    char j;
    int temp=1;
    for(j=0;j<7;j++)//val to string conversion
    {
        str[j] = (((val/temp)%10)+48);//48 is ascii '0' making integer to string
        temp*=10;
    }


    int k,i,l;
    char input[15]= "Sicaklik(C): ";
    for(i=0;i<strlen(input);i++){
        writedata(input[i],1);//write sicaklik
    }

    for(k=0;k<strlen(str)-4;k++){
        writedata(str[2-k],1);//write str to display ch by ch
    }


    __delay_cycles(160000);

}

void DatatoLCDsetTemp(unsigned int val){// set temperature data to lcd

    writedata(0x0001, 0);//clear display

    __delay_cycles(2000);

    char j;
    int temp=1;
    for(j=0;j<7;j++)//val to string conversion
    {
        str_set[j] = (((val/temp)%10)+48);//48 is ascii '0' making integer to string
        temp*=10;
    }


    int k,i,l;
    char input[10]= "Set(C): ";
    for(i=0;i<strlen(input);i++){
        writedata(input[i],1);//write set c
    }

    for(k=0;k<strlen(str_set);k++){
        writedata(str_set[6-k],1);//write str to display ch by ch
    }

    __delay_cycles(160000);

}


void initlcd(){// initialize LCD screen

    volatile unsigned int ptr;
    volatile unsigned int tmp;
    volatile unsigned int upper=240;//0x00F0;

    __delay_cycles(16000);

    ptr=0x0030; //init
    tmp=upper&ptr;
    tmp=tmp>>4;
    sendlcddata(tmp, 0, 0);
    pulseLCD(0,tmp);
    __delay_cycles(4200);

    ptr=0x0030; //init
    tmp=upper&ptr;
    tmp=tmp>>4;
    sendlcddata(tmp, 0, 0);
    pulseLCD(0,tmp);
    __delay_cycles(110);

    ptr=0x0030; //init
    tmp=upper&ptr;
    tmp=tmp>>4;
    sendlcddata(tmp, 0, 0);
    pulseLCD(0,tmp);
    __delay_cycles(45);

    ptr=0x0020; //init
    tmp=upper&ptr;
    tmp=tmp>>4;
    sendlcddata(tmp, 0, 0);
    pulseLCD(0,tmp);
    __delay_cycles(45);

    ptr=0x0028; //function set
    writedata(ptr, 0);
    __delay_cycles(45);

    ptr=0x0006;  //entry mode set
    writedata(ptr, 0);
    __delay_cycles(45);

    ptr=0x000F; //display on
    writedata(ptr, 0);
    __delay_cycles(45);

    ptr=0x0001; //clear display
    writedata(ptr, 0);
    __delay_cycles(2000);

    ptr=0x0002; //return home
    writedata(ptr, 0);
    __delay_cycles(2000);

    ptr=0x0080; //set DDRAM address
    writedata(ptr, 0);
    __delay_cycles(2000);

}

//both IC is full of data (16-bit) and the data is sent to LCD
//data is written using pin 2.0, pin 2.3 and pin 2.4 of MSP430 LaunchPAD
void sendlcddata(int data, int I3, int I4){
    volatile unsigned int sck=8;//0x0008;    //SCK is on
    volatile unsigned int temp=1;//0x0001;
    int i=0;
    volatile unsigned int tmp=0;

    if(ADC_temp_value>ADC_set_temp_value){//control ambient temp vs set temp if larger
                                          // change screen writing routine add BLDC port 2.5(0x20)
        for(i=0;i<4;i++){
            tmp=temp << 3-i;
            tmp=tmp&data;
            tmp=tmp >> 3-i;
            P2OUT=tmp|0x20; //PORT 2.5 ADDED
            __delay_cycles(1);
            P2OUT=sck|tmp|0x20;    //SCK is on(shift data)
            __delay_cycles(1);
            P2OUT=tmp|0x20;    //SCK is off
            __delay_cycles(1);
        }

        P2OUT=I4|0x20;
        __delay_cycles(1);
        P2OUT=sck|I4|0x20;    //SCK is on
        __delay_cycles(1);
        P2OUT=I4|0x20;    //SCK is off
        __delay_cycles(1);

        P2OUT=I3|0x20;
        __delay_cycles(1);
        P2OUT=sck|I3|0x20;    //SCK is on
        __delay_cycles(1);
        P2OUT=I3|0x20;    //SCK is off
        __delay_cycles(1);

        P2OUT=0|0x20;
        __delay_cycles(1);
        P2OUT=sck|0x20;    //SCK is on
        __delay_cycles(1);
        P2OUT=0|0x20;    //SCK is off
        __delay_cycles(1);

        P2OUT=0|0x20;
        __delay_cycles(1);
        P2OUT=sck|0x20;    //SCK is on
        __delay_cycles(1);
        P2OUT=0|0x20;    //SCK is off
        __delay_cycles(1);

        //send 1 byte zeros
        for(i=0;i<8;i++){
            P2OUT=0|0x20;
            __delay_cycles(1);
            P2OUT=sck|0x20;    //SCK is on
            __delay_cycles(1);
            P2OUT=0|0x20;    //SCK is off
            __delay_cycles(1);
        }

        P2OUT=0b00010000|0x20;  //RCK is on(shift data to storage register)
        __delay_cycles(1);
        P2OUT=0|0x20;    //RCK is off
        __delay_cycles(1);
    }else{// if temperature is normal do normal writing routine
        for(i=0;i<4;i++){
            tmp=temp << 3-i;
            tmp=tmp&data;
            tmp=tmp >> 3-i;
            P2OUT=tmp;
            __delay_cycles(1);
            P2OUT=sck|tmp;    //SCK is on
            __delay_cycles(1);
            P2OUT=tmp;    //SCK is off
            __delay_cycles(1);
        }

        P2OUT=I4;
        __delay_cycles(1);
        P2OUT=sck|I4;    //SCK is on
        __delay_cycles(1);
        P2OUT=I4;    //SCK is off
        __delay_cycles(1);

        P2OUT=I3;
        __delay_cycles(1);
        P2OUT=sck|I3;    //SCK is on
        __delay_cycles(1);
        P2OUT=I3;    //SCK is off
        __delay_cycles(1);

        P2OUT=0;
        __delay_cycles(1);
        P2OUT=sck;    //SCK is on
        __delay_cycles(1);
        P2OUT=0;    //SCK is off
        __delay_cycles(1);

        P2OUT=0;
        __delay_cycles(1);
        P2OUT=sck;    //SCK is on
        __delay_cycles(1);
        P2OUT=0;    //SCK is off
        __delay_cycles(1);

        //send 1 byte zeros
        for(i=0;i<8;i++){
            P2OUT=0;
            __delay_cycles(1);
            P2OUT=sck;    //SCK is on
            __delay_cycles(1);
            P2OUT=0;    //SCK is off
            __delay_cycles(1);
        }

        P2OUT=0b00010000;  //RCK is on
        __delay_cycles(1);
        P2OUT=0;    //RCK is off
        __delay_cycles(1);
    }

}

//if operation is zero ==> command ; if operation is one ==> data
//writes 8-bit data
void writedata(unsigned int data, int operation){

    volatile unsigned int upper=240;//0x00F0;
    volatile unsigned int lower=15;//0x000F;

    volatile unsigned int tmp=upper&data;
    tmp=tmp>>4;// bits that send
    sendlcddata(tmp, operation, 0);
    pulseLCD(operation,tmp);
    __delay_cycles(45);
    tmp=lower&data;//lower nibbles
    sendlcddata(tmp, operation, 0);
    pulseLCD(operation,tmp);
    __delay_cycles(45);

}

void pulseLCD(int operation,unsigned int data){// pulse lcd
    __delay_cycles(2);
    sendlcddata(data, operation, 1);
    __delay_cycles(2);
    sendlcddata(data, operation, 0);
    __delay_cycles(2);
}




