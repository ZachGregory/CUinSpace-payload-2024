// Hardware required: 
// GPS
// IMU
// Servos

#include <Adafruit_GPS.h>
#include <ICM_20948.h>
#include <Servo.h>

#define WIRE_PORT Wire
#define AD0_VAL 1
#define GPSSerial Serial1

uint32_t timer = millis();

Adafruit_GPS GPS(&GPSSerial); //GPS Object
ICM_20948_I2C myIMU; //IMU Object
Servo servo1; //Servo Objects
Servo servo2;

float magOffsetX = 0; //Calibrated magnetometer offsets
float magOffsetY = 0;
float magOffsetZ = 0;

float targetLat = 45.382718; // Dunton Tower
float targetLong = -75.699446;

int actualBearing; //The bearing the parachute is currently flying

float imuConf = 0.5;  //Confidence levels for the sensors
float gpsConf = 0.5;

int servoNeutral = 135; //Middle position of servo
int correction = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // GPS setup
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  // IMU setup
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  myIMU.begin(WIRE_PORT, AD0_VAL);
  bool initialized = false;
  while (!initialized){
    myIMU.begin(WIRE_PORT, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myIMU.statusString());
    if (myIMU.status != ICM_20948_Stat_Ok){
      Serial.println("Trying again...");
      delay(500);
    }
    else{
      initialized = true;
    }
  }

  // Servo setup
  servo1.attach(6); // Servos on pins 6 & 7
  servo2.attach(7);
  servo1.write(servoNeutral);  // Set servos to a neutral position (range of 0-270deg)
  servo2.write(servoNeutral);
  delay(1000);
}

void loop() {
  char c = GPS.read();
  parseData();
  
  //Wait for data to be received
  while (!GPS.fix){
    if (millis()-timer>2000){
      Serial.println("No fix");
      timer=millis();
    }
    char c = GPS.read();
    parseData();
  }

  
  
  //Set confidence levels using log constrained between 0 and 1
  gpsConf = constrain(log10(GPS.speed+0.1), 0, 1);
  imuConf = 1-gpsConf;

  actualBearing = gpsConf*GPS.angle + imuConf*getImuHeading();
  Serial.println(actualBearing);

  correction = constrain(getHeadingToWaypoint() - actualBearing, 0, 135);
  servo1.write(servoNeutral-correction);
  servo2.write(servoNeutral+correction);
}



//Function to return the magnetic bearing we are currently facing
int getImuHeading(){
  myIMU.getAGMT();  // Read from sensor
  return atan((myIMU.magY()+magOffsetY) / (myIMU.magX()+magOffsetX))*(180/PI);  //Returns angle of the magnetic field vector
}

//Function to return the heading needed to get to the waypoint
int getHeadingToWaypoint(){
  //X2-X1
  float x = targetLong - GPS.longitudeDegrees;
  float y = targetLat - GPS.latitudeDegrees;
  //dot product
  if (x >= 0){ //Right of north
    return acos((-y)/sqrt(sq(x)+sq(y)));
  } else{ //Left of north
    return -1*acos((-y)/sqrt(sq(x)+sq(y)));
  }
}

void parseData(){
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

}
