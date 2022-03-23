/*
 * Vacuum controller sketch for the arduino vacuum controller module.
 * This sketch is used to:
 * 1) Publish pressure readings from inside the vacuum to the 
 *    /vacuuum_pressure topic.
 * 2) Respond to service requests from central controller to turn the vaccum
 *    relay on and off
 *
 * Note 1 - there is a dependency - the vacuum_switch.h header file needs to have been
 * generated within the UR10_picking package
 * Note 2 - all pressures are in hPa
 */

#include <ros.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ur10_picking/vacuum_switch.h>
#include <ur10_picking/vacuum_calibration.h>

// ---SETUP VACUUM CONTROLLER---
unsigned long pressure;
float baseline_pressure = 880.0; //This is overwritten during calibration.
float sucking_threshold = 30.0;
#define relayPin 2
#define USEIIC 1

//This Macro definition decide whether you use I2C or SPI
//When USEIIC is 1 means use I2C interface, When it is 0,use SPI interface
//Note that the default I2C address is 0x77 but this needs to be checked and updated!
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

// ---SETUP ROS---
// Instantiate the node handle - note reduced settings to save memory:
// Maximum 2 publishers, 2 subscribers, 128 bytes input and 256 output buffers
#define __AVR_ATmega8__
ros::NodeHandle_<ArduinoHardware, 2, 2, 128, 256> nh;

// Define the vacuum_status publisher which publishes to a topic vacuum_status
// Pulishes "true" if sucking object and "false" if not sucking an object
std_msgs::Bool msg;
ros::Publisher vacuum_status("vacuum_status", &msg);

// Define the service for the vacuum controller calibration
// Takes a string request to begin the calibration routine
// Returns a string response when complete
using ur10_picking::vacuum_calibration;

void cali_callback(const vacuum_calibration::Request & req, vacuum_calibration::Response & res)
// NOT WORKING - NEED TO UNDERSTAND SRV STRING INPUTS 
{
  if (String(req.input) == "begin vacuum calibration") {
    vacuum(1);
    delay(2000);
    baseline_pressure = bme.readPressure()/100.0;
    delay(1000);
    vacuum(0);
    res.output = "vacuum calibration complete - please ensure threshold has been correctly set!";
  } else {
    res.output = req.input; }
}
ros::ServiceServer<vacuum_calibration::Request, vacuum_calibration::Response> server_c("vacuum_calibration",&cali_callback);

// Define the service for the vacuum relay
// Takes a boolean request. 1 to start vacuum. 0 to stop vacuum.
using ur10_picking::vacuum_switch;

void switch_callback(const vacuum_switch::Request & req, vacuum_switch::Response & res)
{
  if (req.input == 1) {
    vacuum(1);
    res.output = "vacuum relay switched on"; }
  else if (req.input == 0) {
    vacuum(0);
    res.output = "vacuum relay switched off"; }
  else {
    res.output = "ERROR: input must be 1 or 0 to turn the vacuum on and off respectively"; }

}
ros::ServiceServer<vacuum_switch::Request, vacuum_switch::Response> server_s("vacuum_switch",&switch_callback);


void setup()
{
  // Initiate the relay pin
  pinMode(relayPin, OUTPUT);
    bool rslt;
    rslt = bme.begin(0x76);  
    if (!rslt) {
        while (1);
    }


  // Initiate the ROS node and advertise services and topic
  nh.initNode();
  nh.advertiseService(server_c);
  nh.advertiseService(server_s);
  nh.advertise(vacuum_status);
}

void loop()
{
  msg.data = suck_status();
  vacuum_status.publish( &msg );
  nh.spinOnce();
  delay(1000);
}

void vacuum(bool onoff) 
{
/* Turns vacuum on or off. 
 * Set bool input to 1 to turn vacuum on.
 * Set bool input to 0 to turn vacuum off.
 */
  if(onoff) {
    digitalWrite(relayPin, HIGH); }
  else {
    digitalWrite(relayPin, LOW); }
}

bool suck_status() 
{
/* Function to return the "suck status" of the vacuum
 * Returns true if sucking an object and false if not
*/
  pressure = bme.readPressure()/100.0;
  if ((baseline_pressure - pressure) > sucking_threshold) {
      return true;
  } else {
      return false;
  }
}
