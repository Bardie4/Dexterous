#include <stdio.h>
#include "driver/i2c.h"

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a BH1750 light sensor(GY-30 module) for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP32 chip.
 *
 * Pin assignment:
 *
 * - slave :
 *    GPIO25 is assigned as the data signal of i2c slave port
 *    GPIO26 is assigned as the clock signal of i2c slave port
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect GPIO18 with GPIO25
 * - connect GPIO19 with GPIO26
 * - connect sda/scl of sensor with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 * - i2c master(ESP32) will write data to i2c slave(ESP32).
 * - i2c master(ESP32) will read data from i2c slave(ESP32).
 */

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        500             /*!< delay time between different test items */

#define I2C_EXAMPLE_SLAVE_SCL_IO           GPIO_NUM_16               /*!<gpio number for i2c slave clock  */
#define I2C_EXAMPLE_SLAVE_SDA_IO           GPIO_NUM_17               /*!<gpio number for i2c slave data */
#define I2C_EXAMPLE_SLAVE_NUM              I2C_NUM_0        /*!<I2C port number for slave dev */
#define I2C_EXAMPLE_SLAVE_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_EXAMPLE_SLAVE_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */

#define ESP_SLAVE_ADDR                     0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/

SemaphoreHandle_t print_mux = NULL;

static void i2c_example_slave_init(){
      i2c_port_t i2c_slave_port = I2C_EXAMPLE_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_EXAMPLE_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_EXAMPLE_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    i2c_driver_install(i2c_slave_port, 
                        conf_slave.mode,
                        I2C_EXAMPLE_SLAVE_RX_BUF_LEN,
                        I2C_EXAMPLE_SLAVE_TX_BUF_LEN, 
                        I2C_NUM_0);
}

static void disp_buf(uint8_t* buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {

        Serial.print(buf[i]);
        Serial.print(" ");
        if (( i + 1 ) % 16 == 0) {
            Serial.println();
        }
    }
}

static void i2c_test_task(void* arg)
{
    uint8_t* data = (uint8_t*) malloc(DATA_LENGTH);

    int size;

    while (1) {

      size = i2c_slave_read_buffer( I2C_EXAMPLE_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);

      disp_buf(data, size);

      vTaskDelay( 1 / portTICK_RATE_MS);
    }
}

void setup()
{
    Serial.begin(115200);
    print_mux = xSemaphoreCreateMutex();
    i2c_example_slave_init();
    
    //i2c_port_t i2c_num,
    //void (*fn)(void *),
    //void *arg,
    //int intr_alloc_flags; 

    //intr_handle_t intr = NULL;

    //i2c_isr_register(I2C_EXAMPLE_SLAVE_NUM, i2c_test_task, NULL, ESP_INTR_FLAG_LEVEL6, &intr);
    xTaskCreatePinnedToCore(i2c_test_task, "printer", 1024 * 2, (void *)7, 1, NULL, 0);
}

void loop(){
  delay(100);
}
