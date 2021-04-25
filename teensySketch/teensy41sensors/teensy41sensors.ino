/********************************************************************************************
 * This code is for the Teensy 4.1 in the MataMotive system. It reads the data coming from
 * the GPS connected to the teensy pins and sends that data over the CANBUS network. It also
 * reads the data coming from the analong sensors on the vehicle and sends that over the
 * CANBUS network too.
 * 
 * Connect the GPS Power pin to 5V
 * Connect the GPS Ground pin to ground
 * Connect the GPS TX (transmit) pin to Digital pins on teensy
 * Connect the GPS RX (receive) pin to Digital pins on teensy
 *******************************************************************************************/

 // FlexCAN Library github link https://github.com/tonton81/FlexCAN_T4
 // Adafruit GPS github link https://github.com/adafruit/Adafruit_GPS

#include <FlexCAN_T4.h> // FlexCan library for Teensy 4.0 and 4.1
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true
#define GPSSerial Serial2 // Hardware serial port

static CAN_message_t canMsg;
//static uint8_t hex[1] = "0123456789abcdef";
const int X_pin = 23;               // Analog pin connected to X output
const int Y_pin = 22;               // Analog pin connected to Y output
float mapX = 0;
float mapY = 0;
uint32_t timer = millis();
const int ledPin =  LED_BUILTIN;    // The pin number for LED
int ledState = LOW;                 // ledState used to set the LED
unsigned long previousMillis = 0;   // will store last time LED was updated
const long interval = 1000;         // interval at which to blink (milliseconds)

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> canbus; // CAN2 is Teensy4.1 pins 0 & 1. CAN3 pins support regular CAN2.0 and CANFD modes

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);



// ----------------------------------------------------------------------------------------------------
void setup(void)
{
  pinMode(ledPin, OUTPUT); // Set the digital pin as output:
  Serial.begin(9600);
  Serial.println("Adafruit GPS basic test!");
  //canbus.begin();
  //canbus.setBaudRate(500000); // This value has to match the baud rate on the Quasar/Jetson TX2 board
  //pinMode(2, OUTPUT); digitalWrite(2, LOW);
  
  GPS.begin(9600); // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
}


// ------------------------------------------------------------------------------------------------------
void loop(void) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // Save the last time you blinked the LED
    previousMillis = currentMillis;

    // If the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(ledPin, ledState);
  }

  char c = GPS.read();
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    Serial.println("NMEA has been received!");
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.hour, DEC); Serial.print(':');
    if (GPS.minute < 10) { Serial.print('0'); }
    Serial.print(GPS.minute, DEC); Serial.print(':');
    if (GPS.seconds < 10) { Serial.print('0'); }
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  }
  
    canMsg.flags.extended = random(0,2);
    canMsg.flags.remote = 0;
    canMsg.len = 8;
    canMsg.id = 0x34;
    canMsg.buf[0] = GPS.hour;
    canMsg.buf[1] = GPS.minute;
    canMsg.buf[2] = GPS.seconds;
    canMsg.buf[3] = GPS.latitude;
    canMsg.buf[4] = GPS.longitude;
    canMsg.buf[5] = GPS.speed;
    canMsg.buf[6] = GPS.magvariation; 
    canMsg.buf[7] = GPS.angle; //angle is mentioned as course in Adafruit lib header files


    canbus.read(canMsg);
    Serial.print("CAN bus 0: "); hexDump(8, canMsg.buf);
    Serial.print(" ID: 0x"); Serial.print(canMsg.id, HEX);
    Serial.write('\r');
    Serial.write('\n');

    canbus.write(canMsg);
}
