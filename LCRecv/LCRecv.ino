#include <SPI.h>
#include <LoRa.h>

int incoming = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    // read packet
    String myData = "";
    while (LoRa.available()) {
      //Serial.print((char)LoRa.read());
      myData += (char)LoRa.read();
    }
    Serial.println(myData + ", " + LoRa.packetRssi());

    // Send state
    if (Serial.available() > 0){
      incoming = Serial.read();
      if (incoming == 102) { // 102 - f
        //Serial.println("Opening Valve");
        LoRa.beginPacket();
        LoRa.print("f");
        LoRa.endPacket();
      }
      if (incoming == 99) { // 99 - c
        //Serial.println("Closing Valve");
        LoRa.beginPacket();
        LoRa.print("c");
        LoRa.endPacket();
      }
    }

  }
}