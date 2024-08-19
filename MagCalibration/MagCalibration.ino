#include <ICM_20948.h>
#define WIRE_PORT Wire
#define AD0_VAL 1

ICM_20948_I2C myIMU; //IMU Object

float max_x;
float min_x;
float max_y;
float min_y;
float max_z;
float min_z;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  delay(1000);
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

  Serial.println("Running: ");
  //Set initial values
  myIMU.getAGMT();
  max_x = myIMU.magX();
  min_x = myIMU.magX();
  max_y = myIMU.magY();
  min_y = myIMU.magY();
  max_z = myIMU.magZ();
  min_z = myIMU.magZ();
}

void loop() {

  myIMU.getAGMT();
  //X extremes
  if (myIMU.magX() > max_x){
    max_x = myIMU.magX();
  } else if (myIMU.magX() < min_x){
    min_x = myIMU.magX();
  }

  //Y extremes
  if (myIMU.magY() > max_y){
    max_y = myIMU.magY();
  } else if (myIMU.magY() < min_y){
    min_y = myIMU.magY();
  }

  //Z extremes
  if (myIMU.magZ() > max_z){
    max_z = myIMU.magZ();
  } else if (myIMU.magZ() < min_z){
    min_z = myIMU.magZ();
  }

  Serial.println("X Min: "+ String(min_x) +"    X Max: "+ String(max_x));
  Serial.println("Y Min: "+ String(min_y) +"    Y Max: "+ String(max_y));
  Serial.println("Z Min: "+ String(min_z) +"    Z Max: "+ String(max_z));

  //Offset: (Max - Min)/2

  delay(20);

}
