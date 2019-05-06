#ifndef i2c_slave_h
#define i2c_slave_h

#include <stdint.h>
#include <cstdlib>
#include <stdio.h>
#include "driver/i2c.h"


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

struct i2c_packet{
    int size;
    uint8_t* data = new (uint8_t);
};

class i2c_slave
{
private:

public:
    i2c_packet incoming;
    i2c_slave();
    //~i2c_slave();
    i2c_packet read();
};


#endif
