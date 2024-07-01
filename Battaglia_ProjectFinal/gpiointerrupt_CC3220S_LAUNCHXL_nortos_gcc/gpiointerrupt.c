/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>

#define DISPLAY(x) UART2_write(uart, &output, x, x)
#define timerPeriod 100
#define numTasks 3
#define checkButtonPeriod 200
#define checkTempPeriod 500
#define updateHeatPeriod 1000

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART2_Handle uart;
void initUART2(void)
{
    UART2_Params uartParams;
    // Init the driver
    // Configure the driver
    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_DataLen_8;
    uartParams.readMode = UART2_DataLen_8;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

// Task structure.
typedef struct task {
    int state;
    unsigned long period;
    unsigned long elapsedTime;
    int (*tickFunction)(int);
} task;

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
                { 0x48, 0x0000, "11X" },
                { 0x49, 0x0000, "116" },
                { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;
// Make sure you call initUART() before calling this function.

void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "));
    // Init the driver
    I2C_init();
    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    // Open the driver

    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"));
                while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"));
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id));

        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"));
                    found = true;
            break;
        }

        DISPLAY(snprintf(output, 64, "No\n\r"));
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress));
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

// Thermostat Global Variables
enum BUTTON_STATES {INCREASE_TEMPERATURE, DECREASE_TEMPERATURE, BUTTON_INIT} BUTTON_STATE;
enum TEMPERATURE_SENSOR_STATES {READ_TEMPERATURE, TEMPERATURE_SENSOR_INIT};
enum HEATING_STATES {HEAT_OFF, HEAT_ON, HEAT_INIT};
int16_t ambientTemp = 0;
int16_t setPoint = 20;
int seconds = 0;

// GPIO button callback to increase set point.
void gpioIncreaseTempCallback(uint_least8_t index)
{
    BUTTON_STATE = INCREASE_TEMPERATURE;
}

void gpioDecreaseTempCallback(uint_least8_t index)
{
    BUTTON_STATE = DECREASE_TEMPERATURE;
}

int adjustSetPoint(int state)
{
    // Check if temperature has been adjusted.
    switch (state)
    {
    case INCREASE_TEMPERATURE:
        if (setPoint < 99)
        {
            setPoint++;
        }
        BUTTON_STATE = BUTTON_INIT;
        break;
    case DECREASE_TEMPERATURE:
        if (setPoint > 0)
        {
            setPoint--;
        }
        BUTTON_STATE = BUTTON_INIT;
        break;
    }
    state = BUTTON_STATE;
    return state;
}

int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction))
    {

        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */

        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    }
    return temperature;
}

int getAmbientTemp(int state)
{
    switch (state)
    {
    case TEMPERATURE_SENSOR_INIT:
        state = READ_TEMPERATURE;
        break;
    case READ_TEMPERATURE:
        ambientTemp = readTemp();
        break;
    }
    return state;
}

int setHeatMode(int state)
{
    if (seconds != 0)
    {
        // Turn heat on?
        if (ambientTemp < setPoint)
        {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            state = HEAT_ON;
        }
        else
        {
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            state = HEAT_OFF;
        }

        //Status report sent to server.
        DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", ambientTemp, setPoint, state, seconds));
    }
    seconds++;
    return state;
}

// Driver Handles - Global variables
Timer_Handle timer0;
volatile unsigned char TimerFlag = 0;

void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = 1000000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}


/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Toggle an LED */
    GPIO_toggle(CONFIG_GPIO_LED_0);
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Toggle an LED */
   //REMOVED!!!!! GPIO_toggle(CONFIG_GPIO_LED_1);
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    task tasks[numTasks] = {
                        //task1 = check button and update set point.
                        {
                         .state = BUTTON_INIT,
                         .period = checkButtonPeriod,
                         .elapsedTime = checkButtonPeriod,
                         .tickFunction = &adjustSetPoint
                        },
                        {
                        //task2 = get temp from sensor.
                        .state = TEMPERATURE_SENSOR_INIT,
                        .period = checkTempPeriod,
                        .elapsedTime = checkButtonPeriod,
                        .tickFunction = &getAmbientTemp
                        },
                        {
                         //task3 = update heat and server.
                         .state = HEAT_INIT,
                         .period = updateHeatPeriod,
                         .elapsedTime = updateHeatPeriod,
                         .tickFunction = &setHeatMode
                        }
    };

    //Call driver init functions.
    initUART2();
    initI2C();

    /* Call GPIO driver init function */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
   //REMOVED!!! GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }
    initTimer();

    // Loop
    while (1)
    {
        unsigned int i = 0;

        for (i = 0; i < numTasks; ++i)
        {
            if (tasks[i].elapsedTime>= tasks[i].period)
            {
                tasks[i].state = tasks[i].tickFunction(tasks[i].state);
                tasks[i].elapsedTime = 0;
            }
            tasks[i].elapsedTime += timerPeriod;
        }
        //wait for timer period.
        while(!TimerFlag){}
        //Set timer flag false.
        TimerFlag = 0;
    }

    return (NULL);
}
