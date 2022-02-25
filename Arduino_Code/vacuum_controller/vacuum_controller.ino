/*
 * Vacuum controller sketch for the arduino vacuum controller module.
 * This sketch is used to:
 * 1) Publish pressure readings from inside the vacuum to the 
 *    /vacuuum_pressure topic.
 * 2) Respond to service requests from central controller to turn the vaccum
 *    relay on and off
 *
 * Note that there is a dependency - the vacuum_switch.h header file needs to have been
 * generated within the UR10_picking package
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
#include <std_msgs/String.h>
#include <ur10_picking/vacuum_switch.h>

// ---SETUP ROS---
// Instantiate the node handle
ros::NodeHandle  nh;
using ur10_picking::vacuum_switch;

// Define the publisher
std_msgs::Float32 msg;
ros::Publisher vacuum_pressure("vacuum_pressure", &msg);

// Define the service for the vacuum relay
int i;
#define relayPin 2

void callback(const vacuum_switch::Request & req, vacuum_switch::Response & res){
  if((i++)%2){
    digitalWrite(relayPin, HIGH);
    res.output = "relay switched on"; }
  else {
    digitalWrite(relayPin, LOW);
    res.output = "relay switched off"; }
}
ros::ServiceServer<vacuum_switch::Request, vacuum_switch::Response> server("vacuum_switch",&callback);

// ---SETUP VACUUM CONTROLLER---
//This Macro definition decide whether you use I2C or SPI
//When USEIIC is 1 means use I2C interface, When it is 0,use SPI interface
//Note that the default I2C address is 0x77 but this needs to be checked and updated!
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

void setup()
{
  // Initiate the BME sensor
  bool rslt;
  rslt = bme.begin(0x77);  
  if (!rslt) {
      while (1);
  } 
  delayTime = 40;

  // Initiate the relay pin
  pinMode(relayPin, OUTPUT);

  // Initiate the ROS node and advertise the node name
  nh.initNode();
  nh.advertiseService(server);
  nh.advertise(vacuum_pressure);
}

void loop()
{
  msg.data = bme.readPressure();
  vacuum_pressure.publish( &msg );
  nh.spinOnce();
  delay(1000);
}
