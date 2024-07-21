// Hardware required: 
// GPS
// IMU

#include <SD.h>
#include <SPI.h>
#include <LoRa.h>
#include <string.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <ICM_20948.h>
#include <Servo.h>
#include <Adafruit_Sensor.h> //Altimeter Library
#include "Adafruit_BMP3XX.h" //Also Altimeter Library

#define WIRE_PORT Wire
#define AD0_VAL 1
#define GPSSerial Serial1
#define SEALEVELPRESSURE_HPA (1012.53)
#define TIMEZONE (-4) //Ottawa
//#define TIMEZONE (-6) //New Mexico

uint32_t timer = millis();
uint32_t runTime = millis();
uint32_t txTime = millis();
int incoming = 0;

String dataString = "";

File myFile;
Adafruit_GPS GPS(&GPSSerial); //GPS Object
ICM_20948_I2C myIMU; //IMU Object
Adafruit_BMP3XX bmp; //Altimeter Object

const int gndHeight = 1335; //0 meters AGL

int pinCS = 10; // Pin 10 on Arduino Uno
const int chipSelect = 1;
long lastTime = 0; //For GPS interval
bool headingWritten = false;
String hour;
String minute;
String second;

float magOffsetX = -12.00; //Calibrated magnetometer offsets
float magOffsetY = -16.47;
float magOffsetZ = 21.98;



float targetLat = 45.382718; // Dunton Tower
float targetLong = -75.699446;

int actualBearing; //The bearing the parachute is currently flying

float imuConf = 0.5;  //Confidence levels for the sensors
float gpsConf = 0.5;

int servoNeutral = 90; //Middle position of servo
int correction = 0;

void setup() {

  Serial.begin(115200);
  delay(2000);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // LoRa setup
  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
  }

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


    //Initialize Altimeter
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //Initialize SD Card
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
  }
  Serial.println("card initialized.");

  delay(1000);
}

void loop() {
  timer = millis();

  // Get GPS Data
  if (runTime+1000 < millis()){
    while (millis()-timer<500){
      char c = GPS.read();
      parseData();
    }
    runTime = millis();
    logData();
  }

  // Send data
  if (txTime+1500 < millis()){
    LoRa.beginPacket();
    LoRa.print(dataString);
    LoRa.endPacket();
    txTime = millis();
  }

  // Receive LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    // read packet
    String myData = "";
    while (LoRa.available()) {
      //Serial.print((char)LoRa.read());
      myData += (char)LoRa.read();
    }

    if (myData == "f") {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    if (myData == "c") {
      digitalWrite(LED_BUILTIN, LOW);
    }

    // print RSSI of packet
    //Serial.print("' with RSSI ");
    //Serial.println(LoRa.packetRssi());
  }
  
  //Set confidence levels using log constrained between 0 and 1
  gpsConf = constrain(log10(GPS.speed+0.1), 0, 1);
  imuConf = 1-gpsConf;

  actualBearing = gpsConf*GPS.angle + imuConf*getImuHeading();
  //Serial.println(getHeadingToWaypoint() - actualBearing);
}


//Function to return the magnetic bearing we are currently facing
int getImuHeading(){
  float sumX = 0;
  float sumY = 0;
  float angle;
  for (int i=0; i<10; i++){
    myIMU.getAGMT();  // Read from sensor
    sumX+=myIMU.magX();
    sumY+=myIMU.magY();
  }
  sumX = sumX/10;
  sumY = sumY/10;
  //Serial.println(sumY);
  //Serial.println();
  if (sumY+magOffsetY>0 && sumX+magOffsetX>0){
    //Serial.println("Quad 1");
    angle = (atan((sumY+magOffsetY) / (sumX+magOffsetX))*(180/PI))*-1;
  } else if (sumY+magOffsetY>0 && sumX+magOffsetX<0){
    //Serial.println("Quad 2");
    angle = -180+(atan((sumY+magOffsetY) / (sumX+magOffsetX))*(180/PI))*-1;
  } else if (sumY+magOffsetY<0 && sumX+magOffsetX<0){
    //Serial.println("Quad 3");
    angle = 180+(atan((sumY+magOffsetY) / (sumX+magOffsetX))*(180/PI))*-1;
  } else if (sumY+magOffsetY<0 && sumX+magOffsetX>0){
    //Serial.println("Quad 4");
    angle = (atan((sumY+magOffsetY) / (sumX+magOffsetX))*(180/PI))*-1;
  }
  return angle;  //Returns angle of the magnetic field vector
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
    return (-1)*acos((-y)/sqrt(sq(x)+sq(y)));
  }
}

void parseData(){
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}

void logData(){
  // Data Logging
  dataString = "";
  dataString += millis();
  dataString += ", ";
  if (GPS.hour+TIMEZONE < 10 && GPS.hour+TIMEZONE > 0) { hour = String('0'); } else if (GPS.hour+TIMEZONE >= 0) {hour = String("");}
  if (GPS.hour+TIMEZONE < 0) {hour = String(GPS.hour+TIMEZONE+24);} else {hour += String(GPS.hour+TIMEZONE);}
  if (GPS.minute < 10) { minute = String('0'); } else {minute = String("");}
  minute += GPS.minute;
  if (GPS.seconds < 10) { second = String('0'); } else {second = String("");}
  second += GPS.seconds;
  dataString += hour + ":" + minute + ":" + second;
  dataString += ", ";
  dataString += bmp.temperature;
  dataString += ", ";
  dataString += bmp.pressure/100;
  dataString += ", ";
  dataString += bmp.readAltitude(SEALEVELPRESSURE_HPA);
  dataString += ", ";
  dataString += String(GPS.latitudeDegrees,6);
  dataString += ", ";
  dataString += String(GPS.longitudeDegrees,6);
  dataString += ", ";
  dataString += String(myIMU.agmt.acc.axes.x) + ", " + String(myIMU.agmt.acc.axes.y) + ", " + String(myIMU.agmt.acc.axes.z) + ", "; //gives raw data
  dataString += String(myIMU.agmt.gyr.axes.x) + ", " + String(myIMU.agmt.gyr.axes.y) + ", " + String(myIMU.agmt.gyr.axes.z) + ", "; //gives raw data
  dataString += String(myIMU.agmt.mag.axes.x) + ", " + String(myIMU.agmt.mag.axes.y) + ", " + String(myIMU.agmt.mag.axes.z); //gives raw data
  



  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
    //File dataFile = SD.open("datalog.csv", FILE_WRITE);


  // Display to serial
  //Serial.println("Time (ms) \tTime (hh:mm:ss) \tTemperature (C) Pressure (hPa) \tAltitude (m) \tLatitude \t\tLongitude \t\tAcc X (mg) \tAcc Y (mg) \tAcc Z (mg) \tGyr X (DPS) \tGyr Y (DPS) \tGyr Z (DPS) \tMag X (uT) \tMag Y (uT) \tMag Z (uT)");
  Serial.println(dataString);

  // if the file is available, write to it:
  /*if (dataFile) {
    if (!headingWritten){
      dataFile.println("Time (ms), Time (hh:mm:ss), Temperature (C), Pressure (hPa), Altitude (m), Latitude, Longitude, Acc X (mg), Acc Y (mg), Acc Z (mg), Gyr X (DPS), Gyr Y (DPS), Gyr Z (DPS), Mag X (uT), Mag Y (uT), Mag Z (uT)");
      headingWritten = true;
    }
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    dataString.replace(", ", "\t\t");
    Serial.println("Time (ms) \tTime (hh:mm:ss) \tTemperature (C) Pressure (hPa) \tAltitude (m) \tLatitude \t\tLongitude \t\tAcc X (mg) \tAcc Y (mg) \tAcc Z (mg) \tGyr X (DPS) \tGyr Y (DPS) \tGyr Z (DPS) \tMag X (uT) \tMag Y (uT) \tMag Z (uT)");
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.csv");
  }*/
}
