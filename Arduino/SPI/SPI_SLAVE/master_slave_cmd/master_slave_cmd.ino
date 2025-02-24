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

// State machine to track SPI transaction states
enum SpiState {
  WAITING_FOR_COMMAND,
  READY_TO_SEND_ACK,
  WAITING_FOR_ARGS
};

ESP32SPISlave slave;

static constexpr size_t BUFFER_SIZE = 32;
static constexpr size_t QUEUE_SIZE = 4;

SpiState currentState = WAITING_FOR_COMMAND;
uint8_t tx_buf[BUFFER_SIZE]; // Slave transmit
uint8_t rx_buf[BUFFER_SIZE];// Slave receive (from Master)

/* Main setup*/
void setup()
{
  pinMode(PIN_SS, INPUT_PULLUP);  // Add pull-up to SS pin

  // Start serial for debug output
  Serial.begin(115200);
  delay(2000);

  // clear tx/rx buffers initially
  clearBuffers(tx_buf, BUFFER_SIZE);
  clearBuffers(rx_buf, BUFFER_SIZE);
  // initialize tx/rx buffers
  initializeBuffers(tx_buf, rx_buf, BUFFER_SIZE, 0, 256);

  // set SPI parameters
  slave.setDataMode(SPI_MODE0);   // default: SPI_MODE0
  slave.setQueueSize(QUEUE_SIZE); // default: 1

  // begin() after setting
  slave.begin(VSPI, PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
  Serial.println("start spi slave");
}

void loop()
{

size_t received_bytes = 0;

  // Add to loop() before the switch statement
if (digitalRead(PIN_SS) == LOW) {
  Serial.println("SS pin is LOW - STM32 is trying to communicate");
}

  // Process based on current state
  switch(currentState) {
    case WAITING_FOR_COMMAND:
      // Wait for command from master
      received_bytes = slave.transfer(tx_buf, rx_buf, 1);
      
      if (received_bytes == 1) {
        Serial.printf("Received command: 0x%02X\n", rx_buf[0]);
        
        // Check if valid command
        if (rx_buf[0] == COMMAND_LED_CTRL) {
          // Prepare to send ACK
          currentState = READY_TO_SEND_ACK;
          Serial.println("Valid command, ready to send ACK");
        } else {
          Serial.println("Invalid command");
        }
      }
      break;
      
    case READY_TO_SEND_ACK:
      // Send ACK to master
      tx_buf[0] = ACK_BYTE; // Ensure ACK is set
      received_bytes = slave.transfer(tx_buf, rx_buf, 1);
      
      if (received_bytes == 1) {
        Serial.printf("Sent ACK, received dummy: 0x%02X\n", rx_buf[0]);
        currentState = WAITING_FOR_ARGS;
      }
      else{
        Serial.println("Unresponsive master");
      }
      break;
      
    case WAITING_FOR_ARGS:
      // Receive LED arguments
      received_bytes = slave.transfer(tx_buf, rx_buf, 2);
      
      if (received_bytes == 2) {
        Serial.printf("LED Pin: %d, State: %d\n", rx_buf[0], rx_buf[1]);
        
        // Process LED command
        if (rx_buf[0] == LED_PIN) {
          if (rx_buf[1] == LED_ON) {
            Serial.println("LED ON command received");
            // Implement actual LED control here if needed
          } else {
            Serial.println("LED OFF command received");
            // Implement actual LED control here if needed
          }
        }
        
        // Reset to wait for new command
        currentState = WAITING_FOR_COMMAND;
      }
      break;
  }
  // Small delay to prevent tight loops
  delay(1);
}