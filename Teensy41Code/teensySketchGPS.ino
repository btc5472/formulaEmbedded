#include <IFCT.h> //Improved FlexCAN Teensy Library
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

FlexCAN CANbus0(500000, 0);

static CAN_message_t msg;
//static uint8_t hex[17] = "0123456789abcdef";

const int X_pin = 23; // analog pin connected to X output
const int Y_pin = 22; // analog pin connected to Y output

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 7

// you can change the pin numbers to match your wiring:
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

float mapX = 0;
float mapY = 0;

// -------------------------------------------------------------
void setup(void)
{
  Serial.begin(115200);
  pinMode(X_pin, INPUT);
  pinMode(Y_pin, INPUT);
  CANbus0.begin();
  //pinMode(2, OUTPUT); digitalWrite(2, LOW);

  

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

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


// -------------------------------------------------------------
void loop(void) {

  char c = GPS.read();
  if ((c) && (GPSECHO))
    Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    if (GPS.hour < 10) { Serial.print('0'); }
    Serial.print(GPS.time, DEC); Serial.print(':');
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
  
    msg.flags.extended = random(0,2);
    msg.flags.remote = 0;
    msg.len = 8;
    msg.id = 0x34;
    msg.buf[0] = GPS.hour;
    msg.buf[1] = GPS.minute;
    msg.buf[2] = GPS.seconds;
    msg.buf[3] = GPS.latitude;
    msg.buf[4] = GPS.longitude;
    msg.buf[5] = GPS.speed;
    msg.buf[6] = GPS.magvariation; 
    msg.buf[7] = GPS.angle; //angle is mentioned as course in Adafruit lib header files


    CANbus0.read(msg);
    //Serial.print("CAN bus 0: "); hexDump(8, msg.buf);
    //Serial.print(" ID: 0x"); Serial.print(msg.id, HEX);
    //Serial.write('\r');
    //Serial.write('\n');

    CANbus0.write(msg);
}
