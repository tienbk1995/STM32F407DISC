#include <ESP32SPISlave.h>
#include "helper.h"

#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SCK  18
#define PIN_SS   5



ESP32SPISlave slave;

static constexpr size_t BUFFER_SIZE = 5;
static constexpr size_t QUEUE_SIZE = 1;
uint8_t tx_buf[BUFFER_SIZE] {6, 7, 8, 9, 10}; // Slave transmit
uint8_t rx_buf[BUFFER_SIZE] {0, 0, 0, 0, 0}; // Slave receive (from Master)

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
    initializeBuffers(tx_buf, rx_buf, BUFFER_SIZE);

    // start and wait to complete one BIG transaction (same data will be received from slave)
    const size_t received_bytes = slave.transfer(tx_buf, rx_buf, BUFFER_SIZE);

    // verify and dump difference with received data

    /*if (verifyAndDumpDifference("slave", tx_buf, BUFFER_SIZE, "master", rx_buf, received_bytes)) {
         Serial.println("successfully received expected data from master");
     } else {
         Serial.println("unexpected difference found between master/slave data");
    } */

    printf("number of received bytes: %d\n ", received_bytes);

    for (size_t i = 0; i < BUFFER_SIZE; i++) {
        printf("data %d:%02X\n ", i, rx_buf[i]);
    }
    
    clearBuffers(tx_buf, BUFFER_SIZE);
    clearBuffers(rx_buf, BUFFER_SIZE);
}