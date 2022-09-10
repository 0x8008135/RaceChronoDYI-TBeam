//Library for AXP20x Power Management
#include <axp20x.h>

//Library for Bluetooth Low Energy
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define RACECHRONO_UUID "00001ff8-0000-1000-8000-00805f9b34fb" //RaceChrono service UUID
#define UBX_ID_NAV_DOP 0x04
#define UBX_ID_NAV_PVT 0x07

using namespace std;

bool deviceConnected = false;
bool oldDeviceConnected = false;

int gpsPreviousDateAndHour = 0;
int dateAndHour;
int timeSinceHourStart;

uint8_t _byte;
uint8_t _parserState;
uint8_t _tempPacket[255];
uint8_t rc_data[20];
uint8_t gpsSyncBits = 0;

int16_t msg_length;

AXP20X_Class axp;

HardwareSerial GPSSerial1(1);

BLEServer *BLE_server = NULL;
BLECharacteristic *BLE_GPS_Main_Characteristic = NULL; //RaceChrono GPS Main characteristic UUID 0x03
BLECharacteristic *BLE_GPS_Time_Characteristic = NULL; //RaceChrono GPS Time characteristic UUID 0x04

String device_name = "RC_DYI_" + String((uint16_t)((uint64_t)ESP.getEfuseMac() >> 32));

struct ublox
{
  uint8_t  message_class;
  uint8_t message_id;
  uint16_t payload_length;
};

struct ublox_NAV_DOP : ublox
{
  uint32_t  iTOW;
  uint16_t  gDOP;
  uint16_t  pDOP;
  uint16_t  tDOP;
  uint16_t  vDOP;
  uint16_t  hDOP;
  uint16_t  nDOP;
  uint16_t  eDOP;
};

struct ublox_NAV_PVT : ublox
{
  uint32_t  iTOW;
  uint16_t  year;
  uint8_t   month;
  uint8_t   day;
  uint8_t   hour;
  uint8_t   min;
  uint8_t   sec;
  uint8_t   valid;
  uint32_t  tAcc;
  int32_t   nano;
  uint8_t   fixType;
  uint8_t   flags;
  uint8_t   flags2;
  uint8_t   numSV;
  int32_t   lon;
  int32_t   lat;
  int32_t   height;
  int32_t   hMSL;
  uint32_t  hAcc;
  uint32_t  vAcc;
  int32_t   velN;
  int32_t   velE;
  int32_t   velD;
  int32_t   gSpeed;
  int32_t   headMot;
  uint32_t  sAcc;
  uint32_t  headAcc;
  uint16_t  pDOP;
  uint16_t  reserved2;
  uint32_t  reserved3;
  int32_t   headVeh;
  int16_t   magDec;
  uint16_t  magAcc;
};

union {
  ublox_NAV_DOP dop;
  ublox_NAV_PVT pvt;
} _validPacket;

//Checksum calculation for UBLOX module
void _calcChecksum(uint8_t *CK, uint8_t *payload, uint16_t length)
{
  CK[0] = 0;
  CK[1] = 0;

  for (uint8_t i = 0; i < length; i++)
  {
    CK[0] += payload[i];
    CK[1] += CK[0];
  }
}

//Bluetooth Low Energy BLEServerCallbacks
class ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *BLE_server) 
  {
    deviceConnected = true;
    Serial.println("[I] Bluetooth client connected!");
  };

  void onDisconnect(BLEServer *BLE_server) 
  {
    deviceConnected = false;
    Serial.println("[I] Bluetooth client disconnected!");
  }
};

//Function to send bytearray to ublox receiver ovcer serial
void ublox_sendPacket(byte *packet, byte len)
{
  for (byte i = 0; i < len; i++)
  {
    GPSSerial1.write(packet[i]);
  }
}

//U-blox receiver disable NMEA messages
void ublox_noNMEA()
{
  byte messages[][2] = {
    {0xF0, 0x0A},
    {0xF0, 0x09},
    {0xF0, 0x00},
    {0xF0, 0x01},
    {0xF0, 0x0D},
    {0xF0, 0x06},
    {0xF0, 0x02},
    {0xF0, 0x07},
    {0xF0, 0x03},
    {0xF0, 0x04},
    {0xF0, 0x0E},
    {0xF0, 0x0F},
    {0xF0, 0x05},
    {0xF0, 0x08},
    {0xF1, 0x00},
    {0xF1, 0x01},
    {0xF1, 0x03},
    {0xF1, 0x04},
    {0xF1, 0x05},
    {0xF1, 0x06},
  };

  byte packet[] = {
    0xB5, //sync 1
    0x62, //sync  2
    0x06, //class
    0x01, //id
    0x08, //length
    0x00, //length
    0x00, //payload (first byte from messages array element)
    0x00, //payload (second byte from messages array element)
    0x00, //payload (not changed in the case)
    0x00, //payload (not changed in the case)
    0x00, //payload (not changed in the case)
    0x00, //payload (not changed in the case)
    0x00, //payload (not changed in the case)
    0x00, //payload (not changed in the case)
    0x00, //CK_A
    0x00, //CK_B
  };

  byte packetSize = sizeof(packet);

  //Offset to the place where payload starts.
  byte payloadOffset = 6;

  //Iterate over the messages array.
  for (byte i = 0; i < sizeof(messages) / sizeof(*messages); i++)   
  {
    //Copy two bytes of payload to the packet buffer.
    for (byte j = 0; j < sizeof(*messages); j++)
    {
      packet[payloadOffset + j] = messages[i][j];
    }

    //Set checksum bytes to the null.
    packet[packetSize - 2] = 0x00;
    packet[packetSize - 1] = 0x00;
    _calcChecksum(&packet[packetSize - 2], &packet[2], (packetSize - 4));

    ublox_sendPacket(packet, packetSize); 
  }
}

//U-blox receiver change baudrate to 115200
void ublox_setBaudrate()
{
  byte packet[] = {
    0xB5, //sync 1
    0x62, //sync 2
    0x06, //class
    0x00, //id
    0x14, //length
    0x00, //length
    0x01, //payload
    0x00, //payload
    0x00, //payload
    0x00, //payload
    0xD0, //payload
    0x08, //payload
    0x00, //payload
    0x00, //payload
    0x00, //payload
    0xC2, //payload
    0x01, //payload
    0x00, //payload
    0x07, //payload
    0x00, //payload
    0x03, //payload
    0x00, //payload
    0x00, //payload
    0x00, //payload
    0x00, //payload
    0x00, //payload
    0xC0, //CK_A
    0x7E, //CK_B
  };
  ublox_sendPacket(packet, sizeof(packet));
}

//U-blox receiver change frequency to 10Hz
void ublox_changeFrequency()
{
  byte packet[] = {
    0xB5, //sync 1
    0x62, //sync 2
    0x06, //class
    0x08, //id
    0x06, //length
    0x00, //length
    0x64, //payload
    0x00, //payload
    0x01, //payload
    0x00, //payload
    0x01, //payload
    0x00, //payload
    0x7A, //CK_A
    0x12, //CK_B
  };
  ublox_sendPacket(packet, sizeof(packet));
}

//U-blox receiver enable NAV-PVT messages
void ublox_enableNavPvt()
{
  //CFG-MSG packet.
  byte packet[] = {
    0xB5, //sync 1
    0x62, //sync 2
    0x06, //class
    0x01, //id
    0x08, //length
    0x00, //length
    0x01, //payload
    0x07, //payload
    0x01, //payload
    0x01, //payload
    0x00, //payload
    0x01, //payload
    0x01, //payload
    0x00, //payload
    0x1B, //CK_A
    0xEC, //CK_B
  };
  ublox_sendPacket(packet, sizeof(packet));
}

//U-blox receiver enable NAV-DOP messages
void ublox_enableNavDop()
{
  //CFG-MSG packet.
  byte packet[] = {
    0xB5, //sync 1
    0x62, //sync 2
    0x06, //class
    0x01, //id
    0x08, //length
    0x00, //length
    0x01, //payload
    0x04, //payload
    0x01, //payload
    0x01, //payload
    0x00, //payload
    0x01, //payload
    0x01, //payload
    0x00, //payload
    0x18, //CK_A
    0xD7, //CK_B
  };
  ublox_sendPacket(packet, sizeof(packet));
}

//U-blox receiver configuration
void configGPS()
{
  GPSSerial1.begin(9600, SERIAL_8N1, 34, 12);
  Serial.println("[I] U-BLOX configuration starting");
  Serial.println("[I] Disabling NMEA messages");
  ublox_noNMEA();
  Serial.println("[I] Switching baudrate to 115200");
  ublox_setBaudrate();
  GPSSerial1.flush();
  delay(100);
  GPSSerial1.end();
  delay(100);
  GPSSerial1.begin(115200, SERIAL_8N1, 34, 12);
  Serial.println("[I] Changing frequency to 10Hz");
  ublox_changeFrequency();
  Serial.println("[I] Enabling NAV-PVT / NAV-DOP messages");
  ublox_enableNavPvt();
  ublox_enableNavDop();
  Serial.println("[I] U-BLOX configuration finished");
}

//BLE configuration
void configBLE()
{
  BLEDevice::init(device_name.c_str());
  BLE_server = BLEDevice::createServer();
  BLE_server->setCallbacks(new ServerCallbacks());
  BLEService *BLE_service = BLE_server->createService(RACECHRONO_UUID);

  //GPS main characteristic definition
  BLE_GPS_Main_Characteristic = BLE_service->createCharacteristic(BLEUUID((uint16_t)0x3), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  BLE_GPS_Main_Characteristic->addDescriptor(new BLE2902());

  //GPS time characteristic definition
  BLE_GPS_Time_Characteristic = BLE_service->createCharacteristic(BLEUUID((uint16_t)0x4), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  BLE_GPS_Time_Characteristic->addDescriptor(new BLE2902());

  BLE_service->start();

  BLEAdvertising *BLE_advertising = BLEDevice::getAdvertising();
  BLE_advertising->addServiceUUID(RACECHRONO_UUID);
  BLE_advertising->setScanResponse(false);
  BLE_advertising->setMinInterval(100);
  BLE_advertising->setMaxInterval(100);
  BLEDevice::startAdvertising();
}

//AXP power management configuration
void configAXP()
{
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
}

//Setup UART/BLE/GPS/AXP
void setup()
{
  //ESP32 UART - 115200
  Serial.begin(115200);
  configBLE();
  configAXP();
  configGPS();

  //LED blink fast when ready
  axp.setChgLEDMode(AXP20X_LED_BLINK_4HZ);
}

//U-blox read incoming messages
bool read_ublox()
{
  uint8_t _checksum[2];
  const uint8_t _ubxHeader[2] = {0xB5, 0x62};
  while (GPSSerial1.available())
  {
    _byte = GPSSerial1.read();
    if (_parserState < 2)
    {
      if (_byte == _ubxHeader[_parserState])
      {
        _parserState++;

      }
      else
      {
        _parserState = 0;

      }
    }
    else
    {
      if (_parserState == 2)
      {
        if (_byte == 1)
        { //NAV
          msg_length = 2;
        }

      }
      if (_parserState == 3)
      {
        if (_byte == UBX_ID_NAV_DOP)
        {
          msg_length =  22; //18+4
        }
        else if (_byte == UBX_ID_NAV_PVT)
        {
          msg_length =  96; //92+4
        }
        else
        {
          msg_length = 0;
        }
      }
      if ((_parserState - 2) < msg_length)
      {
        *((uint8_t *)&_tempPacket + _parserState - 2) = _byte;
      }
      _parserState++;
      //compute checksum
      if ((_parserState - 2) == msg_length)
      {
        _calcChecksum(_checksum, ((uint8_t *)&_tempPacket), msg_length);
      }
      else if ((_parserState - 2) == (msg_length + 1))
      {
        if (_byte != _checksum[0])
        {
          _parserState = 0;
        }
      }
      else if ((_parserState - 2) == (msg_length + 2))
      {
        _parserState = 0;
        if (_byte == _checksum[1])
        {
          memcpy(&_validPacket, &_tempPacket, sizeof(_validPacket));
          return true;  
        }
      }
      else if (_parserState > (msg_length + 4))
      {
        _parserState = 0;
      }
    }   
  }
  return false;
}

//Main loop
void loop()
{
  if (deviceConnected) 
  {
    if (read_ublox())
    {
      if (_validPacket.pvt.message_id == UBX_ID_NAV_PVT)
      {
        dateAndHour = (_validPacket.pvt.year - 2000) * 8928 + (_validPacket.pvt.month - 1) * 744 + (_validPacket.pvt.day - 1) * 24 + _validPacket.pvt.hour;

        /*
         * UBLOX   -> scaling:N/A unit:min name:min  desc:Minute of hour, range 0..59 (UTC)
         *   -> scaling:N/A unit:s   name:sec  desc:Seconds of minute, range 0..60 (UTC)
         *   -> scaling:N/A unit:ns  name:nano desc:Fraction of second, range -1e9 .. 1e9 (UTC)
         * RaceChrono -> time from hour start = (minute * 30000) + (seconds * 500) + (milliseconds / 2)
         */
        timeSinceHourStart = _validPacket.pvt.min * 30000 + _validPacket.pvt.sec * 500 + (_validPacket.pvt.nano / 1000000.0) / 2;

        /*
         * UBLOX   -> scaling:1e-5 unit:deg name:headMot desc:Heading of motion (2-D)
         * RaceChrono -> scaling:1e+2 unit:deg
         */
        int gps_bearing = max(0.0, round((double)_validPacket.pvt.headMot / 1000.0));

        /*
         * UBLOX   -> scaling:N/A unit:mm name:hMSL desc:Height above mean sea level
         * RaceChrono -> scaling:N/A unit:deg
         */
        int gps_altitude = ((double)_validPacket.pvt.hMSL * 1e-3) > 6000.f ? ((int)max(0.0, round(((double)_validPacket.pvt.hMSL * 1e-3) + 500.f)) & 0x7FFF) | 0x8000 : (int)max(0.0, round((((double)_validPacket.pvt.hMSL * 1e-3) + 500.f) * 10.f)) & 0x7FFF;

        /*
         * UBLOX   -> scaling:N/A unit:mm/s name:gSpeed desc:Ground Speed (2-D)
         * RaceChrono -> scaling:N/A unit:km/h
         */
        int gps_speed = ((double)_validPacket.pvt.gSpeed * 0.0036) > 600.f ? ((int)(max(0.0, round(((double)_validPacket.pvt.gSpeed * 0.0036) * 10.f))) & 0x7FFF) | 0x8000 : (int)(max(0.0, round(((double)_validPacket.pvt.gSpeed * 0.0036)  * 100.f))) & 0x7FFF;

        rc_data[0] = ((gpsSyncBits & 0x7) << 5) | ((timeSinceHourStart >> 16) & 0x1F);
        rc_data[1] = timeSinceHourStart >> 8;
        rc_data[2] = timeSinceHourStart;
        rc_data[3] = ((min(0x03, (int)_validPacket.pvt.fixType) & 0x3) << 6) | ((min(0x3F, (int)_validPacket.pvt.numSV)) & 0x3F);
        rc_data[4] = _validPacket.pvt.lat >> 24;
        rc_data[5] = _validPacket.pvt.lat >> 16;
        rc_data[6] = _validPacket.pvt.lat >> 8;
        rc_data[7] = _validPacket.pvt.lat;
        rc_data[8] = _validPacket.pvt.lon >> 24;
        rc_data[9] =  _validPacket.pvt.lon >> 16;
        rc_data[10] =  _validPacket.pvt.lon >> 8;
        rc_data[11] =  _validPacket.pvt.lon;
        rc_data[12] = gps_altitude >> 8;
        rc_data[13] = gps_altitude;
        rc_data[14] = gps_speed >> 8;
        rc_data[15] = gps_speed;
        rc_data[16] = gps_bearing >> 8;
        rc_data[17] = gps_bearing;

      }
      else if (_validPacket.dop.message_id == UBX_ID_NAV_DOP)
      {
        /*
         * UBLOX   -> scaling:1e-2 unit:N/A name:hDOP des:Horizontal DOP
         *   -> scaling:1e-2 unit:N/A name:vDOP des:Vertical DOP
         * RaceChrono -> scaling:1e1  unit:N/A
         */
        rc_data[18] = _validPacket.dop.hDOP / 10;
        rc_data[19] = _validPacket.dop.vDOP / 10;   
      }

      BLE_GPS_Main_Characteristic->setValue(rc_data, 20);
      BLE_GPS_Main_Characteristic->notify();

      if (gpsPreviousDateAndHour != dateAndHour)     
      {
        gpsPreviousDateAndHour = dateAndHour;
        gpsSyncBits++;
        rc_data[0] = ((gpsSyncBits & 0x7) << 5) | ((dateAndHour >> 16) & 0x1F);
        rc_data[1] = dateAndHour >> 8;
        rc_data[2] = dateAndHour;
        BLE_GPS_Time_Characteristic->setValue(rc_data, 3);
        BLE_GPS_Time_Characteristic->notify();
      }
    }
  }
  if (!deviceConnected && oldDeviceConnected)     
  {
    delay(500);
    BLE_server->startAdvertising();
    axp.setChgLEDMode(AXP20X_LED_BLINK_4HZ);
    Serial.println("[I] Bluetooth device discoverable");
    oldDeviceConnected = deviceConnected; 
  }
  if (deviceConnected && !oldDeviceConnected)
  {
    axp.setChgLEDMode(AXP20X_LED_BLINK_1HZ);
    oldDeviceConnected = deviceConnected;   
  }
}
