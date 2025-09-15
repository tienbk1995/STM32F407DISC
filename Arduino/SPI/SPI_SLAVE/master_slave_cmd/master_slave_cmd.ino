#include <ESP32SPISlave.h>
#include "helper.h"

/*
    STM32F407 Discovery Board
    PB14 (SPI2_MISO)
    PB15 (SPI2_MOSI)
    PB13 (SPI2_SCK)
    PB12 (SPI2_NSS)
*/

/*
    ESP32 Slave
    GPIO19 (SPI2_MISO)
    GPIO23 (SPI2_MOSI)
    GPIO18 (SPI2_SCK)
    GPIO5  (SPI2_NSS)
*/

#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK  18
#define PIN_SS   5

/*
 * Command definitions for SPI communication
 */
#define COMMAND_LED_CTRL        0x50
#define COMMAND_SENSOR_READ     0x51
#define COMMAND_LED_READ        0x52
#define COMMAND_PRINT           0x53
#define COMMAND_ID_READ         0x54

/*
 * LED control commands
 */
#define LED_ON                   1
#define LED_OFF                  0

/*
 * Analog pin definitions
 */
#define ANALOG_PIN0              0
#define ANALOG_PIN1              1
#define ANALOG_PIN2              2
#define ANALOG_PIN3              3
#define ANALOG_PIN4              4

#define LED_PIN                  9
#define ACK_BYTE                 0xF5


ESP32SPISlave slave;

static constexpr size_t BUFFER_SIZE = 1;
static constexpr size_t QUEUE_SIZE = 1;
size_t received_bytes = 0;
uint8_t tx_buf[BUFFER_SIZE]; // Slave transmit
uint8_t rx_buf[BUFFER_SIZE];// Slave receive (from Master)

void setup()
{
    Serial.begin(115200);

    delay(2000);

    slave.setDataMode(SPI_MODE0);   // default: SPI_MODE0
    slave.setQueueSize(1); // default: 1

    // begin() after setting
    slave.begin(VSPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

    Serial.println("start spi slave");
}

void loop()
{
    // initialize tx/rx buffers
    initializeBuffers(tx_buf, rx_buf, BUFFER_SIZE, 0, 256);

    // start and wait to complete one BIG transaction (same data will be received from slave)
    received_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);
    printf("number of received bytes: %d\n ", received_bytes);


    // verify and dump difference with received data

    /*if (verifyAndDumpDifference("slave", tx_buf, BUFFER_SIZE, "master", rx_buf, received_bytes)) {
         Serial.println("successfully received expected data from master");
     } else {
         Serial.println("unexpected difference found between master/slave data");
    } */
    if (received_bytes)
    {

      for (size_t i = 0; i < received_bytes; i++) 
      {
          printf("data %d:%02X\n ", i, rx_buf[i]);
      }

      if (received_bytes == 1)
      {
        if (COMMAND_LED_CTRL == rx_buf[0])
        {
          tx_buf[0] = ACK_BYTE;
          printf("ACK sent\n");
        }
        else
        {
          printf("Corrupted Data\n");
        }
      }
      else if (received_bytes == 2)
      {
        printf("LED[%d] is :%d\n ", rx_buf[0], rx_buf[1]);
      }
    }
    else
    {
      printf("Transmission failed\n");
    }
    
    clearBuffers(tx_buf, BUFFER_SIZE);
    clearBuffers(rx_buf, BUFFER_SIZE);
}