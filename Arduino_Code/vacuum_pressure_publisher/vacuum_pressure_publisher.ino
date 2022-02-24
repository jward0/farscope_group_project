/*
 * rosserial Publisher with BME280 Pressure sensor test script
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ---SETUP ROS---
// Instantiate the node handle
ros::NodeHandle  nh;

// Define the publisher
std_msgs::Float32 msg;
ros::Publisher vacuum_pressure("vacuum_pressure", &msg);

// ---SETUP VACUUM CONTROLLER---
//This Macro definition decide whether you use I2C or SPI
//When USEIIC is 1 means use I2C interface, When it is 0,use SPI interface
//Note that the default I2C address is 0x77 but for the FARSCOPE group project this needs
//to be changed to 0x76 in the Adafruit_BME280.h library
#define USEIIC 1

#if(USEIIC)
  Adafruit_BME280 bme;
#else
  #define SPI_SCK 13
  #define SPI_MISO 12
  #define SPI_MOSI 11
  #define SPI_CS 10
  Adafruit_BME280 bme(SPI_CS, SPI_MOSI, SPI_MISO, SPI_SCK);
#endif

unsigned long delayTime;

#define relay_pin 2

void setup()
{
    bool rslt;
    rslt = bme.begin(0x76);  
    if (!rslt) {
        while (1);
    }
  
    delayTime = 40;

  //Initiate the relay pin
  pinMode(relay_pin, OUTPUT);

  // Initiate the ROS node and advertise the node name
  nh.initNode();
  nh.advertise(vacuum_pressure);
}

void loop()
{
  msg.data = bme.readPressure();
  vacuum_pressure.publish( &msg );
  nh.spinOnce();
  delay(1000);
}
