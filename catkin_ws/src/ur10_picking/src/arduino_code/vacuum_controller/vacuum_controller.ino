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

#include <ros.h>#include <ur10_picking/vacuum_switch.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <ur10_picking/vacuum_switch.h>
#include <ur10_picking/vacuum_calibration.h>

// ---SETUP VACUUM CONTROLLER---
float pressure;
float baseline_pressure = 880.0; //This is overwritten during calibration.
float sucking_threshold = 30.0;
#define relayPin 2

//This Macro definition decide whether you use I2C or SPI
//When USEIIC is 1 means use I2C interface, When it is 0,use SPI interface
//Note that the default I2C address is 0x77 but this needs to be checked and updated!
Adafruit_BME280 bme;

// ---SETUP ROS---
// Instantiate the node handle - note reduced settings to save memory:
// Maximum 2 publishers, 2 subscribers, 128 bytes input and 256 output buffers
ros::NodeHandle_<ArduinoHardware, 2, 4, 150, 150> nh;

// Define the vacuum_status publisher which publishes to a topic vacuum_status
// Pulishes "true" if sucking object and "false" if not sucking an object
std_msgs::Bool msg;
ros::Publisher vacuum_status("vacuum_status", &msg);

// Define the service for the vacuum controller calibration
// Takes a string request to begin the calibration routine
// Returns a string response when complete
using ur10_picking::vacuum_calibration;
void cali_callback(const vacuum_calibration::Request & req, vacuum_calibration::Response & res)
{
  if (String(req.input) == "start_vacuum_calibration") {
    vacuum(1);
    delay(2000);
    baseline_pressure = bme.readPressure()/100.0;
    delay(1000);
    vacuum(0);
    res.output = "vacuum calibration complete";
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
    delay(100);
    res.output = "vac_on"; }
  else if (req.input == 0) {
    vacuum(0);
    delay(100);
    res.output = "vac_off"; }
  else {
    res.output = "ERROR: input must be 1 or 0 to turn the vacuum on and off respectively"; }
}
ros::ServiceServer<vacuum_switch::Request, vacuum_switch::Response> server_s("vacuum_switch",&switch_callback);


void setup()
{
  // Initiate the relay pin
  pinMode(relayPin, OUTPUT);
  // Find the I2C pin (see code from I2C scanner Arduino example)
  Wire.begin();
  byte address, error, address_final;
  int nDevices;

  nDevices = 0;
  for(address = 1; address < 127; address++ ){
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
	address_final = address;
        nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  // Initiate the sensor based on address from the above I2C scan
  bool rslt;
  if (address_final == 118) {
    bool rslt;
    rslt = bme.begin(0x76);  
    if (!rslt) {
        while (1);
    } 
  } else if (address_final == 119) {
    rslt = bme.begin(0x77);  
    if (!rslt) {
        while (1);
    } 
  } else {
    Serial.print("Error: I2C address is not within range. Restart arduino and try again");
  } 

  // Initiate the ROS node and advertise services and topic
  nh.initNode();
  nh.advertise(vacuum_status);
  delay(100);
  nh.advertiseService(server_s);
  delay(1000);
  nh.advertiseService(server_c);
  
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
