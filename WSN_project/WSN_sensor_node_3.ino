#include <ArduinoBLE.h>
#include <string.h>
unsigned long previousMillis = 0;  // last time the battery level was checked, in ms
char address_node[][3]={"00","01","02","03","04","05","06","07","08","09","10","11"};
byte data[4] = {0x20,0x20,0x26,0x56};
int RSSI[11];
int Relay_node_indicator=0;
 // Bluetooth® Low Energy Battery Service
BLEService TemperatureService("1809");
// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedIntCharacteristic HealthThermometer("2A1C",  // standard 16-bit characteristic UUID
  BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

  


void Scan_Sensor_node();
void setup() {
  pinMode(2,OUTPUT);
  Serial.begin(9600);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");
    while (1);
  }
  BLE.setAdvertisingInterval(320);
  BLE.setLocalName("02");
  BLE.setAdvertisedService(TemperatureService); // add the service UUID
  BLE.addService(TemperatureService);
  BLE.setManufacturerData(data,4);
  TemperatureService.addCharacteristic(HealthThermometer); // add the battery level characteristic
  BLE.addService(TemperatureService); // Add the battery service
  //Giai đoạn khởi tạo mang (Xác định relay node)
  Serial.println("Giai doan khoi tao mang!");
  Serial.println("BLE Scanning lan 1!");
  BLE.scan();
  do
  {
  Scan_Sensor_node();
  }while(millis()-previousMillis <= 3000);
    BLE.stopScan();
    Serial.println("Sensor node address 02 advertising!");
    BLE.advertise();
    BLE.poll(1000);
    BLE.stopAdvertise();
  Serial.println("Bluetooth® Low Energy Central scan");
  // start scanning for peripheral
  BLE.scan();
  while(millis() - previousMillis >= 4000 && (millis() - previousMillis <= 10000))
  {
  Scan_Sensor_node();
  }
  Serial.print("BLE stop scanning!");
  BLE.stopScan();
  for(int i=0;i<12;i++)
  {
  if(((RSSI[i] + 80) > 0) && (RSSI[i] != 0))
  {
   Relay_node_indicator++;
  }
  }
  Serial.println(Relay_node_indicator);
  if(Relay_node_indicator == 2)
  {
       digitalWrite(2,HIGH);
       Serial.println("Relay node address 02 advertising!");
       BLE.advertise();
       BLE.poll(1000);
       Serial.println("Relay node address 02 stop Advertising!");
  }
}

void loop() {
  // check if a peripheral has been discovered
  // BLEDevice peripheral = BLE.available();

  // if (peripheral) {
  //   // discovered a peripheral
  //   Serial.println("Discovered a peripheral");
  //   Serial.println("-----------------------");

  //   // print address
  //   Serial.print("Address: ");
  //   Serial.println(peripheral.address());

  //   // print the local name, if present
  //   if (peripheral.hasLocalName()) {
  //     Serial.print("Local Name: ");
  //     Serial.println(peripheral.localName());
  //   }

  //   // print the advertised service UUIDs, if present
  //   if (peripheral.hasAdvertisedServiceUuid()) {
  //     Serial.print("Service UUIDs: ");
  //     for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
  //       Serial.print(peripheral.advertisedServiceUuid(i));
  //       Serial.print(" ");
  //     }
  //     Serial.println();
  //   }

  //   // print the RSSI
  //   Serial.print("RSSI: ");
  //   Serial.println(peripheral.rssi());

  //   Serial.println();
  // }
}
void Scan_Sensor_node()
{
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    // discovered a peripheral
    Serial.println("Discovered a peripheral");
    Serial.println("-----------------------");

    // print address
    Serial.print("Address: ");
    Serial.println(peripheral.address());

    // print the local name, if present
    if (peripheral.hasLocalName()) 
    {
      Serial.print("Local Name: ");
      Serial.println(peripheral.localName());
      for(int i=0;i<12;i++)
      {
        if(peripheral.localName() == address_node[i])
        {
              RSSI[i] = peripheral.rssi();
              Serial.print("RSSI: ");
              Serial.println(RSSI[i]);
        }
      }
    }
        // print the advertised service UUIDs, if present
    if (peripheral.hasAdvertisedServiceUuid()) {
      Serial.print("Service UUIDs: ");
      for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
        Serial.print(peripheral.advertisedServiceUuid(i));
        Serial.print(" ");
      }
      Serial.println();
    }
    Serial.println();
  }
}