/*
 * fitnessTracker.c
 *
 *  Created on: 21/03/2022
 *      Author: whu35
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"

#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/uart.h"

#include "../OrbitOLED/OrbitOLEDInterface.h"
#include "utils/ustdlib.h"

#include "acc.h"
#include "i2c_driver.h"
#include "circBufT.h"
#include "buttons4.h"

/*
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
*/

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} vector3_t;

/**********************************************************
 * Constants
 **********************************************************/

#define MAX_STR_LEN         32
#define BUF_SIZE            10
#define NUM_SW_POLLS        3

#define SYSTICK_RATE_HZ     25  // 25Hz results in 40ms resolution timer counter
#define DISPLAY_PERIOD      5   // 200ms period = 5Hz
#define BUTTON_PERIOD       1   // 40ms period = 25Hz
#define ADC_PERIOD          2   // 80ms period = 12.5Hz

//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

// Switch 1
#define SW1_PERIPH SYSCTL_PERIPH_GPIOA
#define SW1_PORT_BASE GPIO_PORTA_BASE
#define SW1_PIN GPIO_PIN_7
#define SW1_NORMAL false

// Switch 2
#define SW2_PERIPH SYSCTL_PERIPH_GPIOA
#define SW2_PORT_BASE GPIO_PORTA_BASE
#define SW2_PIN GPIO_PIN_6
#define SW2_NORMAL false


/**********************************************************
 * Global Variables
 **********************************************************/

static circBuf_t g_xAccBuffer;    // Buffer of size BUF_SIZE values of x acceleration
static circBuf_t g_yAccBuffer;    // Buffer of size BUF_SIZE values of y acceleration
static circBuf_t g_zAccBuffer;    // Buffer of size BUF_SIZE values of z acceleration

static circBuf_t g_goalBuffer;    // Buffer of size BUF_SIZE values read from potentiometer

static uint32_t g_timerCount;     // Timer counter

char statusStr[MAX_STR_LEN + 1];  // String for UART communication

// Buttons states
static uint8_t butStateUp;
static uint8_t butStateDown;
static uint8_t butStateLeft;
static uint8_t butStateRight;
static uint8_t buttonHoldTime;

// State machine
static int8_t displayState;
static bool stepCountUnitToggle;
static bool distTravelUnitToggle;
static bool downButtonHold;

// Switches
enum swNames {SW_LEFT = 0, SW_RIGHT, NUM_SW};
enum swStates {SW_DOWN = 0, SW_UP};
static bool sw_state[NUM_SW];
static bool sw_normal[NUM_SW];
static uint8_t sw_count[NUM_SW];

// Step values
static int32_t stepCount;
static uint16_t stepGoal;
static int32_t goalBufferVal;

/*******************************************
 * Local prototypes
 *******************************************/
void SysTickIntHandler(void);
void ADCIntHandler(void);

void initClock (void);
void initDisplay (void);
void initAccl (void);
void initSwitch (void);
void initSysTick (void);
void initialiseUSB_UART (void);
void initADC (void);

void displayUpdate (char *str1, char *str2, int16_t num, uint8_t charLine);
vector3_t getAcclData (void);
int16_t rollCalc(int16_t g_y, int16_t g_z);
int16_t pitchCalc(int16_t g_x, int16_t g_y, int16_t g_z);
void UARTSend (char *pucBuffer);
void updateSwitches (void);
bool checkSwitch (uint8_t swName);
void displayDistKM(int32_t stepCount);
void displayDistMile(int32_t stepCount);
int32_t getGoalValue(void);
void displayPercentGoal(int32_t stepCount, uint16_t stepGoal);
void displaySteps(int32_t stepCount);
void displayCurrentGoal(uint16_t stepGoal);
void displayNewGoal(int32_t goalBufferVal);
uint32_t getSysTime(void);
void displayTask(void);
void buttonTask(void);

/***********************************************************
 * Interrupt handlers
 ***********************************************************
 * SysTick interrupt
 ***********************************************************/
void
SysTickIntHandler(void)
{
    vector3_t acceleration_raw = getAcclData();   // Read data from accelerometer

    // Write accelerometer data to three separate circular buffers
    writeCircBuf (&g_xAccBuffer, acceleration_raw.x);
    writeCircBuf (&g_yAccBuffer, acceleration_raw.y);
    writeCircBuf (&g_zAccBuffer, acceleration_raw.z);

    // Poll button(s) and switch(es)
    updateButtons();
    updateSwitches();

    // Increment timer
    g_timerCount++;
}

//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Writes to a circular buffer.
//
//*****************************************************************************
void
ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_goalBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

/***********************************************************
 * Initialisation functions
 ***********************************************************
 * Clock
 ***********************************************************/
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
}

/*********************************************************
 * Display
 *********************************************************/
void
initDisplay (void)
{
    // Initialise the Orbit OLED display
    OLEDInitialise ();
}

/*********************************************************
 * Accelerometer
 *********************************************************/
void
initAccl (void)
{
    char    toAccl[] = {0, 0};  // parameter, value

    /*
     * Enable I2C Peripheral
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    /*
     * Set I2C GPIO pins
     */
    GPIOPinTypeI2C(I2CSDAPort, I2CSDA_PIN);
    GPIOPinTypeI2CSCL(I2CSCLPort, I2CSCL_PIN);
    GPIOPinConfigure(I2CSCL);
    GPIOPinConfigure(I2CSDA);

    /*
     * Setup I2C
     */
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    GPIOPinTypeGPIOInput(ACCL_INT2Port, ACCL_INT2);

    //Initialize ADXL345 Acceleromter

    // set +-16g, 13 bit resolution, active low interrupts
    toAccl[0] = ACCL_DATA_FORMAT;
    toAccl[1] = (ACCL_RANGE_16G | ACCL_FULL_RES);
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_PWR_CTL;
    toAccl[1] = ACCL_MEASURE;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);


    toAccl[0] = ACCL_BW_RATE;
    toAccl[1] = ACCL_RATE_100HZ;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_INT;
    toAccl[1] = 0x00;       // Disable interrupts from accelerometer.
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_OFFSET_X;
    toAccl[1] = 0x00;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_OFFSET_Y;
    toAccl[1] = 0x00;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);

    toAccl[0] = ACCL_OFFSET_Z;
    toAccl[1] = 0x00;
    I2CGenTransmit(toAccl, 1, WRITE, ACCL_ADDR);
}

/*********************************************************
 * Initialise test switch SW1
 *********************************************************/
void
initSwitch (void)
{
    SysCtlPeripheralEnable(SW1_PERIPH);
    GPIOPinTypeGPIOInput (SW1_PORT_BASE, SW1_PIN);
    GPIOPadConfigSet (SW1_PORT_BASE, SW1_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    sw_normal[SW_RIGHT] = SW1_NORMAL;

    SysCtlPeripheralEnable(SW2_PERIPH);
    GPIOPinTypeGPIOInput (SW2_PORT_BASE, SW2_PIN);
    GPIOPadConfigSet (SW2_PORT_BASE, SW2_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    sw_normal[SW_LEFT] = SW2_NORMAL;

    int i;

    for (i = 0; i < NUM_SW; i++) {
        sw_state[i] = sw_normal[i];
        sw_count[i] = 0;
    }
}

/*********************************************************
 * SysTick
 *********************************************************/
void
initSysTick (void)
{
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet (SysCtlClockGet () / SYSTICK_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister (SysTickIntHandler);
    //
    // Enable interrupt and device
    SysTickIntEnable ();
    SysTickEnable ();
}

//********************************************************
// initialiseUSB_UART - 8 bits, 1 stop bit, no parity
//********************************************************
void
initialiseUSB_UART (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

/*********************************************************
 * ADC (Potentiometer)
 *********************************************************/
void
initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

/***********************************************************
 * Action functions
 ***********************************************************
 * Read accelerometer
 ***********************************************************/
vector3_t
getAcclData (void)
{
    char    fromAccl[] = {0, 0, 0, 0, 0, 0, 0}; // starting address, placeholders for data to be read.
    vector3_t acceleration;
    uint8_t bytesToRead = 6;

    fromAccl[0] = ACCL_DATA_X0;
    I2CGenTransmit(fromAccl, bytesToRead, READ, ACCL_ADDR);

    acceleration.x = (fromAccl[2] << 8) | fromAccl[1]; // Return 16-bit acceleration readings.
    acceleration.y = (fromAccl[4] << 8) | fromAccl[3];
    acceleration.z = (fromAccl[6] << 8) | fromAccl[5];

    return acceleration;
}

/*********************************************************
 * Update display
 *********************************************************/
void
displayUpdate (char *str1, char *str2, int16_t num, uint8_t charLine)
{
    char text_buffer[17];           //Display fits 16 characters wide.

    // "Undraw" the previous contents of the line to be updated.
    OLEDStringDraw ("                ", 0, charLine);
    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.
    usnprintf(text_buffer, sizeof(text_buffer), "%s %s %3d", str1, str2, num);
    // Update line on display.
    OLEDStringDraw (text_buffer, 0, charLine);
}

/*********************************************************
 * Calculate roll of board in milli-radians
 *********************************************************/
int16_t
rollCalc(int16_t g_y, int16_t g_z)
{
    int16_t roll = atan2(g_y, g_z) * 1000;
    return roll;
}

/*********************************************************
 * Calculate pitch of board in milli-radians
 *********************************************************/
int16_t
pitchCalc(int16_t g_x, int16_t g_y, int16_t g_z)
{
    int16_t pitch = atan2((-g_x), sqrt(g_y * g_y + g_z * g_z)) * 1000;
    return pitch;
}

//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void
UARTSend (char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}


/*********************************************************
 * Update the switches
 *********************************************************/
void
updateSwitches (void)
{
    bool sw_value[NUM_SW];
    int i;

    sw_value[SW_LEFT] = (GPIOPinRead (SW2_PORT_BASE, SW2_PIN) == SW2_PIN);
    sw_value[SW_RIGHT] = (GPIOPinRead (SW1_PORT_BASE, SW1_PIN) == SW1_PIN);

    for (i = 0; i < NUM_SW; i++) {
        if (sw_value[i] != sw_state[i]) {
            sw_count[i]++;
            if (sw_count[i] >= NUM_SW_POLLS) {
                sw_state[i] = sw_value[i];
                sw_count[i] = 0;
            }
        }
        else {
            sw_count[i] = 0;
        }
    }
}

/*********************************************************
 * Check designated switch and return its state
 *********************************************************/
bool
checkSwitch (uint8_t swName)
{
    if (sw_state[swName] == sw_normal[swName]) {
        return SW_DOWN;
    }
    else {
        return SW_UP;
    }
}

/*********************************************************
 * Convert a distance into a km string and display it
 *********************************************************/
void
displayDistKM(int32_t stepCount)
{
    char string[17];
    int32_t distanceCM = stepCount * 90;
    int32_t distanceKMD0 = distanceCM / 100000;
    int32_t distanceKMM0 = distanceCM % 100000;
    int32_t distanceKMD1 = distanceKMM0 / 10000;
    int32_t distanceKMM1 = distanceKMM0 % 10000;
    int32_t distanceKMD2 = distanceKMM1 / 1000;
    usnprintf(string, sizeof(string), "%d.%d%d km", distanceKMD0, distanceKMD1, distanceKMD2);
    OLEDStringDraw (string, 0, 1);
}

/*********************************************************
 * Returns the average value in the goal buffer
 *********************************************************/
int32_t
getGoalValue(void)
{
    uint16_t i;
    int32_t sum = 0;
    for (i = 0; i < BUF_SIZE; i++) {
        sum = sum + readCircBuf (&g_goalBuffer);
    }
    int32_t mean = (2 * sum + BUF_SIZE) / 2 / BUF_SIZE;
    int32_t value = mean * 10000 / 4095 / 100;
    return (value*100);
}

/*********************************************************
 * Convert a distance into a mile string and display it
 *********************************************************/
void
displayDistMile(int32_t stepCount)
{
    char string[17];
    int32_t distanceCM = stepCount * 90;
    int32_t distanceMD0 = distanceCM / 160934;
    int32_t distanceMM0 = distanceCM % 160934;
    int32_t distanceMD1 = distanceMM0 / 16093;
    int32_t distanceMM1 = distanceMM0 % 16093;
    int32_t distanceMD2 = distanceMM1 / 1609;
    usnprintf(string, sizeof(string), "%d.%d%d miles", distanceMD0, distanceMD1, distanceMD2);
    OLEDStringDraw (string, 0, 1);
}

/*********************************************************
 * Convert steps to a percentage of steps goal
 *********************************************************/
void
displayPercentGoal(int32_t stepCount, uint16_t stepGoal)
{
    char string[17];

    int32_t wholeNum = 100 * stepCount / stepGoal;
    int32_t decimal = (100 * stepCount % stepGoal) / (stepGoal/10);

    usnprintf(string, sizeof(string), "%d.%d%% of goal", wholeNum, decimal);
    OLEDStringDraw (string, 0, 1);
}

/*********************************************************
 * Display step count
 *********************************************************/
void
displaySteps(int32_t stepCount)
{
    char string[17];

    usnprintf(string, sizeof(string), "%d steps", stepCount);
    OLEDStringDraw (string, 0, 1);
}

/*********************************************************
 * Display current step goal
 *********************************************************/
void
displayCurrentGoal(uint16_t stepGoal)
{
    char string[17];

    usnprintf(string, sizeof(string), "Current: %d", stepGoal);
    OLEDStringDraw (string, 0, 1);
}

/*********************************************************
 * Display new step goal
 *********************************************************/
void
displayNewGoal(int32_t goalBufferVal)
{
    char string[17];

    usnprintf(string, sizeof(string), "New: %d", goalBufferVal);
    OLEDStringDraw (string, 0, 2);
}

/*********************************************************
 * Get system time based on timer counter
 *********************************************************/
uint32_t
getSysTime(void)
{
    uint32_t temp = 0;
    IntMasterDisable();
    temp = g_timerCount;
    IntMasterEnable();
    return temp;
}

/***********************************************************
 * Tasks
 ***********************************************************
 * Display task
 ***********************************************************/
void
displayTask(void)
{
    // Get goal buffer mean value
    goalBufferVal = getGoalValue();

    switch (displayState) {

        case 0: // number of steps since reset
            OLEDStringDraw ("                ", 0, 0);
            OLEDStringDraw ("Steps Taken", 0, 0);
            if (stepCountUnitToggle) {
                OLEDStringDraw ("                ", 0, 1);
                displayPercentGoal(stepCount, stepGoal);
            }
            else {
                OLEDStringDraw ("                ", 0, 1);
                displaySteps(stepCount);
            }
                        OLEDStringDraw ("                ", 0, 2);
            break;

        case 1: // distance travelled since reset
            OLEDStringDraw ("                ", 0, 0);
            OLEDStringDraw ("Travel Distance", 0, 0);
            if (distTravelUnitToggle) {
                OLEDStringDraw ("                ", 0, 1);
                displayDistMile(stepCount);
            }
            else {
                OLEDStringDraw ("                ", 0, 1);
                displayDistKM(stepCount);
            }
            OLEDStringDraw ("                ", 0, 2);
            break;

        case 2: // set goal state
            OLEDStringDraw ("                ", 0, 0);
            OLEDStringDraw ("Set Goal", 0, 0);
            OLEDStringDraw ("                ", 0, 1);
            displayCurrentGoal(stepGoal);
            OLEDStringDraw ("                ", 0, 2);
            displayNewGoal(goalBufferVal);
            break;
    }

    // long hold indicator bar
    if (buttonHoldTime >= 100) {
        OLEDStringDraw(":-) :-) :-) :-) ", 0, 3);
    }
    else if (buttonHoldTime >= 75) {
        OLEDStringDraw(":-) :-) :-)     ", 0, 3);
    }
    else if (buttonHoldTime >= 50) {
        OLEDStringDraw(":-) :-)         ", 0, 3);
    }
    else if (buttonHoldTime >= 25) {
        OLEDStringDraw(":-)             ", 0, 3);
    }
    else {
        OLEDStringDraw("                ", 0, 3);
    }
}

/*********************************************************
 * Manage the button presses
 *********************************************************/
void
buttonTask(void)
{
    butStateLeft = checkButton(LEFT);
    butStateRight = checkButton(RIGHT);
    butStateUp = checkButton(UP);
    butStateDown = checkButton(DOWN);

    // Adjusting the display state with left and right button presses
    if (butStateLeft == PUSHED) {
        displayState++;
        if (displayState > 2) {
            displayState = 0;
        }
    }
    if (butStateRight == PUSHED) {
        displayState--;
        if (displayState < 0) {
            displayState = 2;
        }
    }

    // Up button press operations
    if (butStateUp == PUSHED) {
        if (checkSwitch(SW_RIGHT) == SW_UP) {   // test state, should increment
            stepCount = stepCount + 100;
        }
        else {                                  // normal state, should toggle units
            if (displayState == 0) {
                stepCountUnitToggle = !stepCountUnitToggle;
            }
            else if (displayState == 1) {
                distTravelUnitToggle = !distTravelUnitToggle;
            }
        }
    }

    // Down button press operations
    if (butStateDown == PUSHED) {
        if (checkSwitch(SW_RIGHT) == SW_UP) {
            stepCount = stepCount - 500;
            if (stepCount < 0) {
                stepCount = 0;
            }
        }
        else {
            downButtonHold = 1;
            if (displayState == 2) {
                stepGoal = goalBufferVal;
                displayState = 0;
            }
        }
    }
    else if ((butStateDown == NO_CHANGE) && downButtonHold && (checkSwitch(SW_RIGHT) == SW_DOWN) && (displayState != 2)) {
        buttonHoldTime++;
    }
    else {
        buttonHoldTime = 0;
        downButtonHold = 0;
    }

    // Down button hold - dedicated code
    if (buttonHoldTime >= 125) {
        stepCount = 0;
        buttonHoldTime = 0;
        downButtonHold = 0;
    }
}

/*********************************************************
 * Main
 *********************************************************/
int
main(void)
{
    // General initialisation
    initClock ();
    initDisplay ();
    initAccl ();
    initSysTick ();
    initButtons ();
    initADC();
    initSwitch();

    // Initialise circular buffer
    initCircBuf (&g_goalBuffer, BUF_SIZE);

    // Initialise USB UART
    initialiseUSB_UART ();

    // Initialised state machine
    displayState = 0;
    stepCountUnitToggle = 0;
    distTravelUnitToggle = 0;

    // Initialise task timers
    uint32_t displayTaskTimer = 0;
    uint32_t buttonTaskTimer = 0;
    uint32_t adcTaskTimer = 0;
    uint32_t currentTime;

    // Arbitrarily chosen initial values
    stepCount = 10;
    stepGoal = 1000;

    // Enable interrupts to the processor.
    IntMasterEnable();

    while (1)
    {
        currentTime = getSysTime();

        if ((currentTime - displayTaskTimer) >= DISPLAY_PERIOD) {
            displayTask();
            displayTaskTimer = currentTime;
        }

        if ((currentTime - buttonTaskTimer) >= BUTTON_PERIOD) {
            buttonTask();
            buttonTaskTimer = currentTime;
        }

        if ((currentTime - adcTaskTimer) >= ADC_PERIOD) {
            ADCProcessorTrigger(ADC0_BASE, 3);
            adcTaskTimer = currentTime;
        }

    }
}
