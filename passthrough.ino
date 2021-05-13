//Library for AXP20x Power Management
#include <axp20x.h>
AXP20X_Class axp;
HardwareSerial GPSSerial1(1);

void setup() {
  Serial.begin(115200);
  
  Wire.begin(21, 22);
  if (!axp.begin(Wire, AXP192_SLAVE_ADDRESS))   
  {
    Serial.println("[I] AXP192 Begin PASS");
  }
  else  
  {
    Serial.println("[I] AXP192 Begin FAIL");
  }
  axp.setDCDC1Voltage(3300);            //ESP32 3v3
  axp.setLDO3Voltage(3300);            //GPS   3v3
  axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);  //ESP32 ON
  axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);    //GPS   ON
  axp.setPowerOutPut(AXP192_LDO2, AXP202_OFF);  //LORA
  axp.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);
  GPSSerial1.begin(9600, SERIAL_8N1, 34, 12);
}

void loop() {

  if (Serial.available()) {      // If anything comes in Serial (USB),
    GPSSerial1.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
  }

  if (GPSSerial1.available()) {     // If anything comes in Serial1 (pins 0 & 1)
    Serial.write(GPSSerial1.read());   // read it and send it out Serial (USB)
  }
}
