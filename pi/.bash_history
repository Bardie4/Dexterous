ls
cd ..
ls
cd pi
$ sudo apt-get install python-pip libglib2.0-dev
sudo apt-get install python-pip libglib2.0-dev
sudo apt-get update
sudo apt-get install python-pip libglib2.0-dev
sudo pip install bluepy
sudo apt-get install libbluetooth
apt-get install bluez-libs*
sudo apt-get install bluez-libs*
apt-get install libbluetooth3 libbluetooth3-dbg libbluetooth-dev
sudo apt-get install libbluetooth3 libbluetooth3-dbg libbluetooth-dev
mkdir bl
cd bl
sudo nano simplescan.c
ls
nano simplescan.c 
gcc -o simplescan simplescan.c -lbluetooth
ls
./simplescan 
sudo nano client.c
ls
nano client.c 
sudo nano client.c 
gcc -o client client.c -lbluetooth
ls
./client
./simplescan 
sudo nano client.c 
./client
sudo nano client.c 
ls
mkdir i2c
ls
cd i2c/
sudo nano Rpi_i2c.cpp
ls
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo apt-get update
sudo apt-get install pigpio python-pigpio python3-pigpio
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
‘memset’ was not declared in this scope
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
ls
./test
sudo ./test
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo ./test
./test
chmod +x test 
./test 
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
ls
cd i2c/
ls
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo ./test 
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo ./test 
sudo nano Rpi_i2c.cpp
sudo ./test 
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo ./test 
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo ./test 
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo ./test 
ls
sudo nano Rpi_i2cSniff.c
g++ -Wall -pthread -o sniff Rpi_i2cSniff.c -lpigpio -lrt
sudo nano Rpi_i2cSniff.c
sudo ./sniff 
LS
ls
sudo ./sniff 
sudo raspi-config
ls /dev/*i2c*
sudo apt-get install -y i2c-tools
i2cdetect
i2cdetect -a
i2cdetect -a 0
i2cdetect 0
i2cdetect 1
i2cdetect 0
i2cdetect 2
i2cdetect 1
/*
*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include "pigpio.h"
/*
This software reads pigpio notification reports monitoring the I2C signals.
Notifications are pipe based so this software must be run on the Pi
being monitored.
It should be able to handle a 100kHz bus.  You are unlikely to get any
usable results if the bus is running at 400kHz.
gcc -o pig2i2c pig2i2c.c
Do something like
sudo pigpiod -s 2
# get a notification handle, assume handle 0 was returned
pigs no
# start notifications for SCL/SDA
e.g. pigs nb 0 0x3   # Rev. 1 select gpios 0/1
e.g. pigs nb 0 0xC   # Rev. 2 select gpios 2/3
e.g. pigs nb 0 0xA00 # select gpios 9/11 (1<<9|1<<11)
# run the program, specifying SCL/SDA and notification pipe
./pig2i2c SCL SDA </dev/pigpioN # specify gpios for SCL/SDA and pipe N
e.g. ./pig2i2c 1  0 </dev/pigpio0 # Rev.1 I2C gpios
e.g. ./pig2i2c 3  2 </dev/pigpio0 # Rev.2 I2C gpios
e.g. ./pig2i2c 9 11 </dev/pigpio0 # monitor external bus 
*/
#define RS (sizeof(gpioReport_t))
#define SCL_FALLING 0
#define SCL_RISING  1
#define SCL_STEADY  2
#define SDA_FALLING 0
#define SDA_RISING  4
#define SDA_STEADY  8
static char * timeStamp()
{    static char buf[32];    struct timeval now;    struct tm tmp;    gettimeofday(&now, NULL);
}
void parse_I2C(int SCL, int SDA)
{    static int in_data=0, byte=0, bit=0;    static int oldSCL=1, oldSDA=1;    int xSCL, xSDA;    if (SCL != oldSCL);    {       oldSCL = SCL;       if (SCL) xSCL = SCL_RISING;
sudo i2cdetect -y 0
sudo i2cdetect -y 1
lsmod|grep i2c
sudo i2cdetect -y 1
lsmod|grep i2c
sudo i2cdetect -y 1
sudo nano /etc/modprobe.d/raspi-blacklist.conf
ls
cd ..
ls
sudo nano /etc/modprobe.d/raspi-blacklist.conf
sudo nano /etc/modprobe.d/raspi-blacklist.conf 
ls
cd etc/modprobe.d/
ls
sudo nano raspi-blacklist.conf 
cd ..
ls
sudo nano modules
sudo apt-get install i2c-tools
sudo adduser pi i2c
sudo i2cdetect -y 1
cd ..
ls
cd home/
ls
cd pi/
ls
sudo nano pi2c
ls
mkdir pi2c
sudo nano /pi2c/main.cpp
ls
cd pi2c/
ls
sudo nano main.cpp
ls
g++ pi2c.o test.cpp -o test
g++ pi2c.o main.cpp -o test
g++ pi2c main.cpp -o test
g++ main.cpp pi2c.o -o test
ls
sl
ls
sudo nano pi2c.h
sudo nano pi2c.cpp
g++ pi2c.o main.cpp -o test
sudo nano pi2c.o
g++ pi2c.o main.cpp -o test
git
sudo apt-get install git
ls
cd ..
mkdir PI2C
ls
rm -r PI2C/
ls
git clone https://github.com/JohnnySheppard/Pi2c.git
ls
cd Pi2c/
ls
sudo nano test.cpp
i2cdetect -y 1
g++ pi2c.o test.cpp -o test
ls
./test 
ls
./test 
sudo ./test 
sudo nano test.cpp 
g++ pi2c.o test.cpp -o test
sudo ./test 
sudo nano test.cpp 
cat ~/etc/modprobe.d/raspi-blacklist.conf
cd ..
cat ~/etc/modprobe.d/raspi-blacklist.conf
cd etc/modprobe.d/
ls
cat raspi-blacklist.conf 
sudo nano raspi-blacklist.conf 
cd ..
ls
sudo nano modules
sudo apt-get install i2c-tools
sudo adduser pi i2c
reboot
sudo reboot
i2detect -y 1
i2cdetect -y 1
sudo ./test 
ls
cd Pi2c/
ls
sudo ./test 
sudo nano I2c.cpp
ls
sudo nano test.cpp 
g++ pi2c.o test.cpp -o test
./test 
cd ..
mkdir test2
cd test2/
gpio load i2c
sudo apt-get install gpio
i2cdetect -y 1
sudo apt-get install gpio
i2cdetect -y 1
sudo apt-get install gpio
i2cdetect -y 1
ls
sudo nano main.c
gcc main.c -o main
./main
./main "HI"
./main 9
i2cdetect -y 1
sudo raspi-config
sudo reboot
ls
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade
sudo apt-get install i2c-tools
'sudo nano /etc/modprobe.d/raspi-blacklist.conf
sudo nano /etc/modprobe.d/raspi-blacklist.conf
sudo nano /boot/config.txt 
sudo nano /etc/modules
sudo adduser pi i2c
sudo reboot
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            ls
cd pi2c/
ls
sudo nano main.cpp 
cd ..
cd Pi2c/
ls
sudo nano test.cpp 
cd ..
ls
cd i2c/
ls
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test test.cpp -lpigpio -lrt
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo nano Rpi_i2c.cpp
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
./test 
sudo ./test 
sudo nano Rpi_i2c.cpp
sudo ./test 
g++ -Wall -pthread -o test Rpi_i2c.cpp -lpigpio -lrt
sudo ./test 
ls
cd i2c/
ls
./test 
ls
sudo nano Rpi_i2c.cpp
cd i2c
ls
sudo nano Rpi_i2c.cpp
192.168.2.197192.168.2.197
sudo apt-get install vim
vim Rpi_i2c.cpp
[A
