/*
 * File: DHT11.h
 * Project: src
 * Author: Mohamed Anis Messaoud,  medanis.messaoud@gmail.com
 * -----
 * Modified By: Mohamed Anis Messaoud
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

#ifndef DHT11_H
#define DHT11_H
#include <stdint.h>

#define DHT_READ_TIMEOUT 100 // time out in us when reading bits
typedef enum
{
    DHT_OK,
    DHT_NO_RESPONSE,
    DHT_CRC_ERR,
    DHT_READ_ERR,
    DHT_TIMEOUT_ERR,
} DHT11_ERR;

typedef struct
{
    float T;
    float RH;
    DHT11_ERR result;
} dht11_result;

class DHT11
{
public:
    uint8_t data[5];
    int pin;

    DHT11(int pin);

       /**
     * @brief reads the data from the DHT11 modulef
     *
     * @param pin the number of the pin connected to the dht11 data pin
     * @return dht11_result structure containing the last operation result and parsed data if ok
     */
    dht11_result read();

private:
    void sendStartSignal();
    DHT11_ERR ChekResponse();
    int readBit();
    DHT11_ERR readData();
    DHT11_ERR measureBitTime(uint8_t *HighTime, uint8_t *LowTime);
    void parseData(float *RH, float *T);
};

#endif /* DHT11_H */
