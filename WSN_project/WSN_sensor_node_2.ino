#include <ArduinoBLE.h>
#include <string.h>
#define Sensor_node_in_network 3  //Số lượng sensor node có trong mạng đặt vào đây
unsigned long previousMillis = 0;  // last time the battery level was checked, in ms
char address_node[][3]={"00","01","02","03","04","05","06","07","08","09","10","11"};
byte data[4] = {0x20,0x20,0x26,0x56};
int RSSI[11];
int Relay_node_indicator=0;
int temp[10]={0,0,0,0,0,0,0,0,0,0};
int power_battery[10]={0,0,0,0,0,0,0,0,0,0};
int address_sensor_node[10]={0,0,0,0,0,0,0,0,0,0};
int data_sensor_node[3] = {00,150,75};
//int relay_node_quantity[11]={1,2,3,4,5,6,7,8,9,10,11};
int packet_per_relay = 0;
int tmp=0;
unsigned long current_millis = 0;
 // Bluetooth® Low Energy Battery Service
BLEService TemperatureService("1809");
// Bluetooth® Low Energy Battery Level Characteristic
BLEUnsignedIntCharacteristic HealthThermometer("2A1C",  // standard 16-bit characteristic UUID
  BLERead | BLEWrite); // remote clients will be able to get notifications if this characteristic changes

  

//Khai báo các function sử dụng:
void Scan_Sensor_node();
void Advertising_relay_node_to_gateway();
void Advertising_sensor_node_to_relay_node();
void relay_node_scan_sensor_node();
void Relay_node_nhan_du_lieu_tu_sensor_node(BLEDevice peripheral);
void setup() {
  pinMode(2,OUTPUT);
  esp_sleep_enable_timer_wakeup(5 * 1000000); //light sleep for 2 seconds
  Serial.begin(9600);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }
  BLE.setAdvertisingInterval(320);
  BLE.setLocalName("01");
  BLE.setAdvertisedService(TemperatureService); // add the service UUID
  BLE.addService(TemperatureService);
  BLE.setManufacturerData(data,4);
  TemperatureService.addCharacteristic(HealthThermometer); // add the battery level characteristic
  BLE.addService(TemperatureService); // Add the battery service
  //Giai đoạn khởi tạo mang (Xác định relay node)
  // BLE.scan();
  // do
  // {
  // Scan_Sensor_node();
  // }while(millis()-previousMillis <= 1000);
  //   BLE.stopScan();
  //   Serial.println("Sensor node address 01 advertising!");
  //   BLE.advertise();
  //   BLE.poll(1000);
  //   BLE.stopAdvertise();
  // Serial.println("Bluetooth® Low Energy Central scan");
  // // start scanning for peripheral
  // BLE.scan();
  // while(millis() - previousMillis >= 2000 && (millis() - previousMillis <= 10000))
  // {
  // Scan_Sensor_node();
  // }
  // BLE.stopScan();
  // for(int i=0;i<11;i++)
  // {
  // if(((RSSI[i] + 80) > 0) && (RSSI[i]!=0))
  // {
  //  Relay_node_indicator++;
  // }
  // }
  // if(Relay_node_indicator == (Sensor_node_in_network - 1))
  // {
  //      digitalWrite(2,HIGH);
  //      Serial.println("Relay node address 00 advertising!");
  //      BLE.advertise();
  //      //Phat advertising để nhận tin nhắn từ gateawy:
  //      Serial.println("Relay node phat advertising de nhan tin nhan tu gateway!");
  //      Advertising_relay_node_to_gateway();
  //      BLE.stopAdvertise();
  //      Serial.println("Relay node address 00 stop Advertising!");
  //      digitalWrite(2, LOW);
  // }
}

void loop() {
  // esp_light_sleep_start(); //Che do light sleep trong 5s sau đó thức dậy
  // Serial.println("Sensor node/Relay node vao che do light sleep!");
  // if(Relay_node_indicator == (Sensor_node_in_network - 1)) //Nếu là Relay node
  // {
  //     Serial.println("Relay node address 00 scanning to other sensor nodes!");
  //     BLE.scan();
  //     relay_node_scan_sensor_node();
      
  // }
  // else // Nếu không là relay node
  // {
  //   Serial.println("Sensor node gui du lieu den relay node!");
  //   BLE.advertise();
  //   Advertising_sensor_node_to_relay_node();
  // }
    Serial.println("Sensor node gui du lieu den relay node!");
    BLE.advertise();
    Advertising_sensor_node_to_relay_node();
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
      for(int i=0;i<10;i++)
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
void Advertising_relay_node_to_gateway()
{
  while(1)
  {
  BLEDevice central = BLE.central();
  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    Serial.println(central.localName());
    if(central.localName()!="00")
    {
     central.disconnect();
     Serial.println("Thiet bi ket noi khong phai la Gateway!"); 
    }
    // turn on the LED to indicate the connection:
    digitalWrite(2, LOW);
    BLECharacteristic TemperatureCharacteristic = central.characteristic("2A1C");
    while (central.connected()) 
    {
     TemperatureCharacteristic.readValue(packet_per_relay);
     Serial.println("Packets per relay: ");
     Serial.println(packet_per_relay);
    }
    // when the central disconnects, turn on the LED:
    digitalWrite(2, HIGH);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    break;
  }
  }
}
void Advertising_sensor_node_to_relay_node()
{
  int i=0;
  while(1)
  {
  BLEDevice central = BLE.central();
  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    Serial.println(central.localName());
    for(int j=0;j<12;j++)
    {
      if(central.localName() != address_node[j])
      {
       central.disconnect();
      }
    }
    // turn on the LED to indicate the connection:
    //digitalWrite(2, LOW);
    BLECharacteristic TemperatureCharacteristic = central.characteristic("2A1C");
    current_millis = millis();
    while (central.connected()) 
    {
        if(millis()-previousMillis>=100)
        {
        TemperatureCharacteristic.writeValue(data_sensor_node[i]);
        i++;
        if(i > 2)
        {
          central.disconnect();
          break;
        }
        current_millis = millis();
        }
    }
    // when the central disconnects, turn on the LED:
    digitalWrite(2, HIGH);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    break;
  }
  }
}
void relay_node_scan_sensor_node()
{
  while(1)
  {
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
    for(int j=0;j<12;j++)
    {
      if(peripheral.localName() != address_node[j])
      {
       return;
      }
    }
    BLE.stopScan();
    Relay_node_nhan_du_lieu_tu_sensor_node(peripheral);
    digitalWrite(2, HIGH);
    Serial.print("Peripheral Disconnected: ");
    Serial.println(peripheral.localName());
    break;
  }
  }
}
void Relay_node_nhan_du_lieu_tu_sensor_node(BLEDevice peripheral)
{
  int i=0;
  Serial.println("Connecting to sensor node ...");
  if (peripheral.connect()) {
    Serial.println("Connected to sensor node!");
  } else {
    Serial.println("Failed to connect!");
    return;
  }
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }
  //Nhận dữ liệu từ sensor node tới relay node
  BLECharacteristic TemperatureCharacteristic = peripheral.characteristic("2A1C");

  if (!TemperatureCharacteristic) {
    Serial.println("Peripheral does not have Temperature characteristic!");
    peripheral.disconnect();
    return;
  } else if (!TemperatureCharacteristic.canRead()) {
    Serial.println("Peripheral does not have a readable LED characteristic!");
    peripheral.disconnect();
    return;
  }
    current_millis = millis();
    while (peripheral.connected()) 
    {
     TemperatureCharacteristic.readValue(tmp);
     if(i<=packet_per_relay)
     {
     address_sensor_node[i]=tmp;
    Serial.println("Address sensor node gui du lieu: ");
     Serial.println(address_sensor_node[i]);
     }
     else if((i > packet_per_relay) && (i <= 2*packet_per_relay) )
     {
     temp[i-packet_per_relay-1] = tmp;
     Serial.println("nhiet do nhan duoc: ");
     Serial.println(temp[i-packet_per_relay-1]);
     }
     else if(i>2*packet_per_relay)
     {
     power_battery[i-2*packet_per_relay-1] = tmp;
     Serial.println("dung luong pin cua sensor node gui du lieu: ");
     Serial.println(temp[i-packet_per_relay-1]);
     }
     i++;
     if(millis()-current_millis >= 3*1000*packet_per_relay)
     {
      peripheral.disconnect();
      break;
     }
    }
}
