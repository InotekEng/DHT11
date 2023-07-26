/*
 * File: DHT11.c
 * Project: src
 * Author: Omar Messaoud,  messaoudomar715@gmail.com
 * -----
 * Modified By: Omar Messaoud
 * -----
 *
 * Copyright (c) 2023 Inotek Engineering SARL
 *
 * THIS SOFTWARE IS PROVIDED BY Inotek Engineering SARL "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Inotek Engineering SARL OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <Arduino.h>
#include "DHT11.h"
// creating a class.

/**
 * @brief initializing DHT11.
 * initializing DHT11 by setting the pin as output.
 * @param pin the pin that we chose to read data from.
 */
DHT11::DHT11(int pin)
{
    this->pin = pin;
    pinMode(pin, INPUT_PULLUP);
    digitalWrite(pin, 0);
}
/**
 * @brief read the sensor.
 * The sensor reads the data and verify if they are correct then return it.
 * @param pin The pin that we chose to read data from.
 * @return dht11_result
 */
dht11_result DHT11::read()
{
    dht11_result res = {0, 0, DHT_NO_RESPONSE};

    // send start signal
    sendStartSignal();

    // check response
    res.result = ChekResponse();
    if (res.result != DHT_OK)
    {
        return res;
    }

    // read data
    res.result = readData();
    if (res.result != DHT_OK)
    {
        return res;
    }

    //  parse data
    parseData(&res.RH, &res.T);
    return res;
}
/**
 * @brief reading the data.
 *  reading every 8 bits and putting it on table called data and verifing if the data that we read was correct and return an error if something went wrong.
 * @param pin The pin that we chose to read data from.
 * @return DHT11_ERR the function will return an error if a problem happened while reading.
 */
DHT11_ERR DHT11::readData()
{

    for (int i = 0; i < 5; i++)
    {
        int8_t x = 0;
        for (int j = 0; j < 8; j++)
        {
            int tmp = readBit();
            if (tmp != -1)
            {
                x |= tmp << (7 - j);
            }
            else
            { // Error message if a bit is missing.
                return DHT_READ_ERR;
            }
        }

        data[i] = x;
    }
    // verifying if cheksum byte are equal to all other bytes together.
    if (data[0] + data[1] + data[2] + data[3] == data[4])
    {
        return DHT_OK;
    }
    else
    {
        return DHT_CRC_ERR;
        Serial.println("error CRC ");
    }
}
/**
 * @brief sending the start signal.
 * We should send a start signal for the sensor to begin.
 * @param pin The pin that we chose to read data from.
 */
void DHT11::sendStartSignal()
{
    // Sending starting signal.
    pinMode(pin, OUTPUT);
    delay(18);
    pinMode(pin, INPUT_PULLUP);
}
/**
 * @brief reading a single bit.
 * reading just one bit and verifying if it is 1 or 0 and return -1 if there was an error.
 * @param pin The pin that we chose to read data from.
 * @return int :the return will define if the bit sent is 0 or 1 .
 */
int DHT11::readBit()
{
    // Analyzing the time to read the 40 bites bite by bite.
    uint8_t lowTime = 0;
    uint8_t highTime = 0;
    measureBitTime(&highTime, &lowTime);

    if (lowTime < 60)
    {
        if (highTime > 20 && highTime < 30)
        {
            // bit 0
            return (0);
        }
        else if (highTime > 60 && highTime < 80)
        {
            // bit 1
            return (1);
        }
        else
        {
            // error
            return (-1);
        }
    }
    else
    {
        // error
        return (-1);
    }
}

/**
 * @brief Measuring the time
 *Measuring the of high-voltage-level and low-voltage-level to know if the bit is 1 or 0.
 * @param pin The pin that we chose to read data from.
 * @param highTime The total time high-voltage-level.
 * @param lowTime The total time low-voltage-level.
 * @return DHT11_ERR DHT11_ERR the function will return a time out error if a problem happened .
 */
DHT11_ERR DHT11::measureBitTime(uint8_t *highTime, uint8_t *lowTime)
{ // a verifier avec anis plus tard.
    DHT11_ERR result = DHT_OK;

    pinMode(pin, INPUT); //
    int limit = micros();

    while (digitalRead(pin) == 1)
    {
        int limit2 = micros();
        if ((limit2 - limit) >= DHT_READ_TIMEOUT)
        {
            return DHT_TIMEOUT_ERR;
        }
    }

    int timeFallingEdge = micros();
    while (digitalRead(pin) == 0)
    {
        int limit3 = micros();
        if ((limit3 - timeFallingEdge) >= DHT_READ_TIMEOUT)
        {
            return DHT_TIMEOUT_ERR;
        }
    }
    int timeRisingEdge = micros();

    while (digitalRead(pin) == 1)
    {
        int limit4 = micros();
        if ((limit4 - timeRisingEdge) >= DHT_READ_TIMEOUT)
        {
            return DHT_TIMEOUT_ERR;
        }
    }

    int timeFallingEdge2 = micros();

    *lowTime = timeRisingEdge - timeFallingEdge;
    *highTime = timeFallingEdge2 - timeRisingEdge;

    return result;
}
/**
 * @brief Parsing the data.
 * In this function we parsed the data we took every 8 bits and converted it to decimal to give temperature and RH value.
 * @param data a table where we stocked every 8 bits together in one case
 * @param RH The RH value.
 * @param T The temperature value.
 */
void DHT11::parseData(float *RH, float *T)
{
    // initializing variables .
    float rhInteg = data[0];
    float rhDecimal = data[1];
    float tempInteg = data[2];
    float tempDecimal = data[3];
    // verifying the lengh and put it after the intger .
    if (rhInteg > 100)
    {
        *RH = rhInteg + (rhDecimal / 1000);
    }
    else if (rhInteg > 10)
    {
        *RH = rhInteg + (rhDecimal / 100);
    }
    else
    {
        *RH = rhInteg + (rhDecimal / 10);
    }

    if (tempDecimal > 100)
    {
        *T = tempInteg + (tempDecimal / 1000);
    }
    else if (tempDecimal > 10)
    {
        *T = tempInteg + (tempDecimal / 100);
    }
    else
    {
        *T = tempInteg + (tempDecimal / 10);
    }
}
/**
 * @brief checke response
 * After sending the start signal the sensor waits for the response signal to send the 40 bits that we will parse so we checked if the response was send or not.
 * @param pin The pin that we chose to read data from.
 * @return DHT11_ERR returning a check response error if the response wasn't sent.
 */
DHT11_ERR DHT11::ChekResponse()
{

    uint8_t LowTime, HighTime;
    measureBitTime(&LowTime, &HighTime);
    // verifying the time of the check response.
    if ((LowTime < 80) || (HighTime < 80))
    {
        return DHT_NO_RESPONSE;
    }
    else
    {
        return DHT_OK;
    }
}