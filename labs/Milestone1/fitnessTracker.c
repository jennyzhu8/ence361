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

#define SYSTICK_RATE_HZ    10
#define BUF_SIZE           10

#define MAX_STR_LEN 32

//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX

/**********************************************************
 * Global Variables
 **********************************************************/

static circBuf_t g_xAccBuffer;    // Buffer of size BUF_SIZE values of x acceleration
static circBuf_t g_yAccBuffer;    // Buffer of size BUF_SIZE values of y acceleration
static circBuf_t g_zAccBuffer;    // Buffer of size BUF_SIZE values of z acceleration

static uint32_t g_ulSampCnt;    // Counter for the interrupts

char statusStr[MAX_STR_LEN + 1];

/*******************************************
 * Local prototypes
 *******************************************/
void SysTickIntHandler(void);

void initClock (void);
void initDisplay (void);
void initAccl (void);
void initSysTick (void);

void displayUpdate (char *str1, char *str2, int16_t num, uint8_t charLine);
vector3_t getAcclData (void);
int16_t rollCalc(int16_t g_y, int16_t g_z);
int16_t pitchCalc(int16_t g_x, int16_t g_y, int16_t g_z);

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

    g_ulSampCnt++;    // Increment the sampler counter
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
 * Main
 *********************************************************/
int
main(void)
{
    uint16_t i;

    int32_t sum_x;
    int32_t sum_y;
    int32_t sum_z;

    int32_t mean_x;
    int32_t mean_y;
    int32_t mean_z;

    int16_t current_roll;
    int16_t current_pitch;

    uint8_t butState;
    int8_t upPushes = 0;

    // Initialisation
    initClock ();
    initDisplay ();
    initAccl ();
    initSysTick ();
    initButtons ();

    initCircBuf (&g_xAccBuffer, BUF_SIZE);
    initCircBuf (&g_yAccBuffer, BUF_SIZE);
    initCircBuf (&g_zAccBuffer, BUF_SIZE);

    initialiseUSB_UART ();

    // Enable interrupts to the processor.
    IntMasterEnable();

    // Calculate and display reference orientation
    vector3_t ref_acc = getAcclData();
    int32_t ref_x = ref_acc.x;
    int32_t ref_y = ref_acc.y;
    int32_t ref_z = ref_acc.z;
    int16_t ref_roll = rollCalc(ref_y, ref_z) * 180 / M_PI / 1000;
    int16_t ref_pitch = pitchCalc(ref_x, ref_y, ref_z) * 180 / M_PI / 1000;
    OLEDStringDraw ("Reference", 0, 0);
    displayUpdate ("Roll ", "deg", ref_roll, 1);
    displayUpdate ("Pitch", "deg", ref_pitch, 2);
    // Transmit data to the PC terminal
    usprintf (statusStr, "roll = %4d | pitch = %4d \r\n", ref_roll, ref_pitch);
    UARTSend (statusStr);

    // Wait approximately 3 seconds
    SysCtlDelay (SysCtlClockGet ());

    OLEDStringDraw ("                ", 0, 0);
    OLEDStringDraw ("Accelerometer", 0, 0);

    while (1)
    {
        // Calculate the sum of the values in each acceleration buffer
        sum_x = 0;
        sum_y = 0;
        sum_z = 0;
        for (i = 0; i < BUF_SIZE; i++) {
            sum_x = sum_x + readCircBuf (&g_xAccBuffer);
            sum_y = sum_y + readCircBuf (&g_yAccBuffer);
            sum_z = sum_z + readCircBuf (&g_zAccBuffer);
        }

        // Calculate the rounded mean of the values in each acceleration buffer
        mean_x = (2 * sum_x + BUF_SIZE) / 2 / BUF_SIZE;
        mean_y = (2 * sum_y + BUF_SIZE) / 2 / BUF_SIZE;
        mean_z = (2 * sum_z + BUF_SIZE) / 2 / BUF_SIZE;

        // Calculate the current roll and pitch
        current_roll = rollCalc(mean_y, mean_z) * 180 / M_PI / 1000;
        current_pitch = pitchCalc(mean_x, mean_y, mean_z) * 180 / M_PI / 1000;

        // Poll button(s)
        updateButtons ();

        // Check up button and increment counter if pushed
        butState = checkButton (UP);
        if (butState == PUSHED) {
            upPushes++;
            if (upPushes > 2) {
                upPushes = 0;
            }
        }

        // Check down button and perform reference procedure if pushed
        butState = checkButton (DOWN);
        if (butState == PUSHED) {
            ref_x = mean_x;
            ref_y = mean_y;
            ref_z = mean_z;
            ref_roll = current_roll;
            ref_pitch = current_pitch;
            OLEDStringDraw ("                ", 0, 0);
            OLEDStringDraw ("Reference", 0, 0);
            displayUpdate ("Roll ", "deg", ref_roll, 1);
            displayUpdate ("Pitch", "deg", ref_pitch, 2);
            OLEDStringDraw ("                ", 0, 3);
            // Transmit data to the PC terminal
            usprintf (statusStr, "roll = %4d | pitch = %4d \r\n", ref_roll, ref_pitch);
            UARTSend (statusStr);

            SysCtlDelay (SysCtlClockGet ());

            OLEDStringDraw ("                ", 0, 0);
            OLEDStringDraw ("Accelerometer", 0, 0);
        }

        /* // Orientation calculations [IRRELEVANT]
        if (mean_z < ref_z) {                   // raw value of z has decreased
            mean_z = 256 + mean_z - ref_z;      // corresponding negative shift
        } else if (mean_z > ref_z) {            // raw value of z has increased
            mean_z = 256 + ref_z - mean_z;      // corresponding negative shift
        } */

        // Change units of acceleration depending on up button press
        switch (upPushes) {
        case 1: // milli-g
            mean_x = 1000 * mean_x / 256;
            mean_y = 1000 * mean_y / 256;
            mean_z = 1000 * mean_z / 256;
            break;
        case 2: // ms-2
            mean_x = mean_x * 981 / 25600;
            mean_y = mean_y * 981 / 25600;
            mean_z = mean_z * 981 / 25600;
            break;
        }

        // Display mean acceleration values
        displayUpdate ("Accl", "X", mean_x, 1);
        displayUpdate ("Accl", "Y", mean_y, 2);
        displayUpdate ("Accl", "Z", mean_z, 3);

        // Transmit data to the PC terminal
        usprintf (statusStr, "x = %4d | y = %4d | z = %4d \r\n", mean_x, mean_y, mean_z);
        UARTSend (statusStr);

        // Delay for a period such that this loop runs at approximately 2Hz
        SysCtlDelay (SysCtlClockGet () / 6);
    }
}
