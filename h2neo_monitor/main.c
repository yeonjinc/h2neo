// Written by Jenny Cho
// Modified in May 11th, 2020

// Project h2neo

// The following project is for an electric flow rate monitor for gravity-assisted
// IV therapy equipment. A LCD screen is integrated into a MSP430F5529 Launchpad,
// interfaced via SPI communication. Other user interface features a rotary encoder
// that is used to adjust and input setting by the user.
// The flow rate sensing is done using an optical system consisting of an infrared
// LED and a photodiode.

#include <math.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include <msp430f5529.h>
#include "nokia5110.h"
#include "rotary_encoder.h"
#include "test.h"
#include "convertNprint.h"

#define MEMSIZE         8                              // size of memory buffer used for flow rate calculations
#define GTT_FACTOR      20                              // factor specified in tubing packaging (used to calculate # drops/min)
#define GTT_FACTOR_STR  "20"                            // ^ in string format... not sure if it'll work lol
#define SIGNAL_LENGTH   1000                            // 2 * 20ms **Eric: Changed this to 1000ms to limit double drop counting (so max rate we can go to is around 350ml/hr)

// tic - number of times the Timer ISR is entered after x clock cycles
//          tic will be programmed to be 1ms long
// sec - seconds (tic * clock cycles)
// min - minutes (sec / 60)
unsigned short int dropStopwatch = SIGNAL_LENGTH + 1;   // Length of time between each drop used to check if 1 drop has occurred ( > 20ms) value primed to enter if() the first time
unsigned long int tic = 0;                              // (data type short can only go up to 65,535 ms which is only ~1m5sec)
unsigned short int msec = 0, sec = 0, min = 0;
unsigned short int oMsec = 0, oSec = 0, oMin = 0;       // old sec; old min
unsigned char dropFLG = 0;                              // presence of a drop

// save last 5 time interval values and average to find more accurate flow rate
unsigned long int ticMem[MEMSIZE];                      // global var auto initialized to 0
unsigned short int index = 0;
char str[6];                                            // used to convert each integer to string

float flowRate = 0.0; // mL/hr
float oldRate = 0.0;  // old value used to determine if rate should be printed again

unsigned char isPrompting = 1;                          // initially set to YES
unsigned char alarmTriggered = 0;
unsigned short desiredRate = 0;

// interrupt flags
char rotKnobIFG = 0;                                    // rotary encoder knob turned
char rotButIFG = 0;                                     // rotary encoder button pressed
char s2IFG = 0;                                         // on-board P1.1 (S2) pressed

short i = 0, yCursor = 1;                               // yCursor = 0 is taken by the stopwatch display

char refRate[6];                                        // The desired rate but as a string

// Eric's additions
#define SAMPLE_LENGTH 10

int adcValue;
float inSignal[SAMPLE_LENGTH];
unsigned int pos = 0;

// **--CHANGE THESE PARAMETERS FOR ALGORITHM--**
unsigned int lag = 5;
float threshold = 100;
float influence = 0.0;
// **-----------------------------------------**

float filteredIn[SAMPLE_LENGTH];
float avgFilter[SAMPLE_LENGTH];
int outSignal[SAMPLE_LENGTH];
int trigger = 0;
int peaks = 0;

int ticMem_isFull = 0;
unsigned short int numDrops = 0;

/********************************************************************************
 * main.c
 ********************************************************************************/
int main(void) {
// -------------------------------------------- **Initialization** --------------------------------------------
    WDTCTL = WDTPW + WDTHOLD;       // stop watchdog timer
    P4DIR |= BIT7;                  // Configure P4.7 as output (for blinking debugging)

    P2DIR |= BIT5;                  // Configure PIN2.5 (IR LED) as output
    P2OUT |= BIT5;                  // Set PIN 2.5 as HIGH

    //Setup Buttons (REMOVE ONCE REPLACED BY SIGNAL)
    P1DIR &= ~BIT1;                 // P1.1 input
    P1REN |= BIT1;                  // Enable pullup resistor of P1.1 (default: GND)
    P1OUT |= BIT1;                  // Set pullup resistor to active (+3.3V) mode

    Clock_Init_1MHz();              // used for TimerA and LCD

    Timer0_A5_Init();               // Initialize Timer A0

    ADC12_0_Init();                 // for analog sensor signal

    SPI_Init();                     // for LCD screen connection
    _delay_cycles(50000);

    RotEnc_Init();              // sets on-board LED to output for debugging

    LCD_Init();
    clearLCD();

    yCursor = 1;

    // P1.1 (Button) Intterupts
    P1IE |= BIT1;                   // P1.1 interrupt enabled
    P1IFG &= ~BIT1;                 // P1.1 interrupt flag cleared

    // ADC12 Init
    ADC12CTL0 &= ~ADC12SC;          // Clear the start bit (precautionary)
    ADC12CTL0 |= ADC12SC;           // Start conversion

    // General interrupts enable
    __bis_SR_register(GIE);

// -------------------------------------------- **Main Loop** --------------------------------------------
    while (1) {
       // If prompting the user and the rotary encoder buttons is not pressed
        /*if (isPrompting && !rotButIFG) {
            int2str(desiredRate, refRate);

            // LCD screen display
            setCursor(0, 0);
            prints("Desired");      // 7 characters             "Desired
            setCursor(0, 1);        //                           flow rate:"
            prints("flow rate:");   // 10 characters

            setCursor(30, 5);
            prints("   ");
            setCursor(30, 5);
            prints(refRate);        // The desired flow rate (changes with turn of encoder)

            setCursor(60, 5);
            prints("mL/h");
        }
        // If rotary encoder button is pressed
        else if (rotButIFG) {
            if (isPrompting) {
                isPrompting = 0;
            }else {
                isPrompting = 1; // **Eric: I only commented this one out because my button is a bit glitchy
            }
            rotButIFG = 0;
            clearLCD();
        }
        // If not prompting anymore, starting detecting drops through the active_monitor() function
        else {*/
            active_monitor();
       // }
    }
 }

// -------------------------------------------- **Drop Detection** --------------------------------------------
// -------------------------------------------- **Std-Dev Based Algo START (Eric)** --------------------------------------------
// Simple function to calculate mean
float calcMean(float data[], int len) {
    float sum = 0.0, mean = 0.0;
    int i;

    for (i = 0; i < len; ++i) {
        sum += data[i];
    }

    mean = sum / len;
    return mean;
}

// The actual drop detection calculations
void thresholding(int i, float inSignal[], int outSignal[], int lag, float threshold, float influence) {
    if (fabsf(inSignal[i] - avgFilter[i - 1]) > threshold) {
        // If the different between input and average is greater than a threshold value, toggle
        if (inSignal[i] < avgFilter[i - 1]) {
           outSignal[i] = -1;
           trigger = 1;
        }

        filteredIn[i] = influence * inSignal[i] +  (1-influence) * filteredIn[i - 1];
    }else {
        outSignal[i] = 0;
        if(outSignal[i] == 0 && trigger){
            peaks++;
            dropFLG = 1; // dropFLG triggers when incrementing # of peaks
            //printf("Drops Detected: %d\n", peaks);
        }
        trigger = 0;

        filteredIn[i] = inSignal[i];
    }

    avgFilter[i] = calcMean(filteredIn + i - lag, lag);
}

void active_monitor(void)
{
   if(pos < SAMPLE_LENGTH){        // Before the array is filled...
        inSignal[pos] = (float) adcValue;   // store ADC value into array

        if(pos == lag){             // When lag value is reached, start peak detection
            // Thresholding Init
            memcpy(filteredIn, inSignal, sizeof(float)*SAMPLE_LENGTH);  // Copy the values of inSignal to filteredIn
            avgFilter[lag - 1] = calcMean(inSignal, lag);               // Initial Mean
            thresholding(pos, inSignal, outSignal, lag, threshold, influence);
        }else if (pos > lag){
            thresholding(pos, inSignal, outSignal, lag, threshold, influence);
        }

        pos++;
    }else{ // When array is full, the new values are getting added to the end and the array is getting shifted, with the first value getting deleted.
        memmove(&inSignal[0], &inSignal[1], sizeof(inSignal) - sizeof(*inSignal));  //Shift function (WORKS)
        inSignal[pos-1] = adcValue;
        memmove(&filteredIn[0], &filteredIn[1], sizeof(filteredIn) - sizeof(*filteredIn));
        memmove(&avgFilter[0], &avgFilter[1], sizeof(avgFilter) - sizeof(*avgFilter));

        thresholding(pos-1, inSignal, outSignal, lag, threshold, influence);
    }

// -------------------------------------------- ** END ** --------------------------------------------
    //Poll Buttons here. Control the Timer. Update LCD Display.
    // If drop is detected (from ADC12 interrupt)
    if (dropFLG && (dropStopwatch > SIGNAL_LENGTH)) { //Get the first value that is below the threshold and ignore all values within 40ms within that value
        // Start timer
        if (!TA0CCR0) { // TIMER IS OFF if !; else not 0, aka timer is ON
            startTimer0_A5();
            dropStopwatch = 0;

            // Display number of drops detected
            char str[2];
            int2strXX(peaks, str);
            setCursor(72, 0);
            prints(str);

        } else {
            stopTimer0_A5();

            P4OUT ^= BIT7;              // toggle LED

            ticMem[index] = tic;        // save measured time to ticMem buffer

            // print to screen ms between drops (for debugging)
            int2str(ticMem[index++], str);
            setCursor(0, yCursor);
            prints("      ");  // 6 blank to clear screen
            setCursor(0, yCursor++);
            prints(str);


            // Display number of drops detected
            char str[2];
            int2strXX(peaks, str);
            setCursor(72, 0);
            prints(str);


            if (index > MEMSIZE-1) {  // memsize - 1 (when memsize = 5)
                index = 0;            // index wraparound
                yCursor = 1;
            }

            if(!ticMem_isFull){
                numDrops += 1;
            }

            if(index == MEMSIZE-1 && !ticMem_isFull){
                ticMem_isFull = 1;
            }

            printf("%lu ", tic);
            printf("%f\n", flowRate);

            startTimer0_A5();
        }

        dropFLG = 0;

    }else{
        P4OUT &= ~BIT7;
    }

    // display desired flow rate
    setCursor(0, 0);
    prints("ref: ");
    prints(refRate);
    prints(" mL/h");

    // display GTT factor
    setCursor(42, 1);
    prints("GTT:");
    setCursor(72, 1);
    prints(GTT_FACTOR_STR);

/** Refreshing display timer everytime a drop is detected */
/*    msec = tic;
    sec = tic / 1000;
    min = tic / 60000;

    if (msec != oMsec) {  // if different
        char str[2];
        msec = msec % 100;
        int2strXX(msec, str);
        setCursor(72, 0);
        prints(str);
    }
    oMsec = msec;

    if (sec != oSec) {  // if different
        char str[2];
        int2strXX(sec%60, str);
        setCursor(54, 0);
        prints(str);
    }
    oSec = sec;

    if (min != oMin) {
        char str[2];
        int2strXX(min, str);
        setCursor(36, 0);
        prints(str);
    }
    oMin = min;*/

    // Calculation of flow rate & display
    if (ticMem[0]) {  // not zero
        // this might be being repeated too many times...
        unsigned long int sum = 0, avgTime_ms = 0;

        // Get total sum of time values that are currently in the ticMem array
        for (i = 0; i <= numDrops; i++) {
            sum += ticMem[i];
        }

        avgTime_ms = (float) sum / numDrops;  // yields average msec

        flowRate = 3600000.0 / ((float) GTT_FACTOR * avgTime_ms);

        // change the flowRate to string
        char buf[80];
        displayFlowRate(&flowRate, buf);
        setCursor(36, 3);
        prints(buf);
        setCursor(60, 3);
        prints(" mLh");
        /*if (flowRate != oldRate) {
			printf("Tic = %d\n", tic);
			printf("Flow Rate %f\n", flowRate);
			oldRate = flowRate;
        }*/

    } else {
        setCursor(36, 3);
        prints("no drops");
        setCursor(36, 4);
        prints("detected");
    }
}
