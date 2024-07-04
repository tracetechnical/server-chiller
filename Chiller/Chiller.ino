#include <arduino-timer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#define WLAN_SSID       "GetOrfMyWLAN-Guest"
#define WLAN_PASS       "BobLePonge2020!"


#define AIO_SERVER      "mqtt.io.home"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""



#define ONE_WIRE_BUS D4

#define PUMP_CTRL D0

#define FAN1_TACH D5
#define FAN2_TACH D6
#define FAN3_TACH D7

#define FAN1_PWM D1
#define FAN2_PWM D2
#define FAN3_PWM D3

WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_USERNAME, AIO_KEY);

OneWire oneWire(ONE_WIRE_BUS);
Timer<> timer = timer_create_default();
DallasTemperature sensors(&oneWire);

DeviceAddress tankSensor;
DeviceAddress inflowSensor;
DeviceAddress outflowSensor;
DeviceAddress ambientSensor;
DeviceAddress postcoolSensor;
DeviceAddress postradSensor;

uint8_t pumpStat = 0;

int fanCount1;
int fanCount2;
int fanCount3;
unsigned long nowTime;
unsigned long lastTime;
uint8_t spo = 0;
uint8_t stage = 0;
float tankSensorTemp;
float inflowSensorTemp;
float outflowSensorTemp;
float ambientSensorTemp;
float postcoolSensorTemp;
float postradSensorTemp;
bool primed = 0;
bool displayPacket = 0;
uint8_t displayCount = 0;
char displayBuffer[10];
uint8_t pumpCount = 0;
uint16_t overrideTimer = 0;

Adafruit_MQTT_Subscribe pumpOnOff = Adafruit_MQTT_Subscribe(&mqtt, "servers/cooling/pump/request");
Adafruit_MQTT_Subscribe fanSpeed = Adafruit_MQTT_Subscribe(&mqtt, "servers/cooling/fans/request");

//1K = 0
//2K = 40
//3K = 80
//4K = 134
//5K = 185
//6K = 234

uint8_t getSpeed(uint16_t rpm) {
  Serial.println(rpm);
  float diffFromBase = 0;
  float range = 0;
  uint8_t minR = 0;
  uint8_t maxR = 0;
  uint16_t minSp = 0;
  uint8_t out = 0;
  float diffRatio = 0;
  if (rpm >= 0 && rpm <= 999) {
    return 0;
  }
  if (rpm >= 1000 && rpm <= 1999) {
    minSp = 1000;
    minR = 0;
    maxR = 40;
  }
  if (rpm >= 2000 && rpm <= 2999) {
    minSp = 2000;
    minR = 40;
    maxR = 80;
  }
  if (rpm >= 3000 && rpm <= 3999) {
    minSp = 3000;
    minR = 80;
    maxR = 133;
  }
  if (rpm >= 4000 && rpm <= 4999) {
    minSp = 4000;
    minR = 134;
    maxR = 184;
  }
  if (rpm >= 5000 && rpm <= 5999) {
    minSp = 5000;
    minR = 185;
    maxR = 233;
  }
  if (rpm >= 6000 && rpm <= 6999) {
    minSp = 6000;
    minR = 234;
    maxR = 255;
  }
  diffFromBase = rpm - minSp;
  range = maxR - minR;
  diffRatio = (diffFromBase / 1000) * range;
  out =  minR + diffRatio;
  Serial.println(out);
  return out;
}

ICACHE_RAM_ATTR void pinChanged1() {
    fanCount1++;
}

ICACHE_RAM_ATTR void pinChanged2() {
    fanCount2++;
}

ICACHE_RAM_ATTR void pinChanged3() {
    fanCount3++;
}

char tempString[8];
Adafruit_MQTT_Subscribe *subscription;
  
bool resetCount(void *) {
  int rpm1 = fanCount1 * 30; //Pulse 2x per rev
  int rpm2 = fanCount2 * 30; //Pulse 2x per rev
  int rpm3 = fanCount3 * 30; //Pulse 2x per rev
  float diffEvap = postcoolSensorTemp - ambientSensorTemp;
  float diffRad = outflowSensorTemp - inflowSensorTemp;
  float diffRadA = postradSensorTemp - postcoolSensorTemp;
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command
  
  Serial.println("----------");
  Serial.print("Override:");
  Serial.println(overrideTimer);
  Serial.println("----------");
  Serial.print("Pump Status:");
  Serial.print(pumpCount);
  if (pumpStat) {
    Serial.println(" ON");
  } else { 
    Serial.println(" OFF");
  }
  Serial.println("----------");
  Serial.println(" F1 / F2 / F3 ");
  Serial.println("----------");
  Serial.print(rpm1);
  Serial.print("/");
  Serial.print(rpm2);
  Serial.print("/");
  Serial.println(rpm3);
  Serial.println("----------");
  Serial.println(" TS  / IF  /  OF  / AM  / PC  / PR  ");
  Serial.println("----------");
  Serial.print(tankSensorTemp);
  Serial.print("/");
  Serial.print(inflowSensorTemp);
  Serial.print("/");
  Serial.print(outflowSensorTemp);
  Serial.print("/");
  Serial.print(ambientSensorTemp);
  Serial.print("/");
  Serial.print(postcoolSensorTemp);
  Serial.print("/");
  Serial.println(postradSensorTemp);
  Serial.println("----------");
  Serial.print("TΔ Evap:      ");
  if(diffEvap > 0) {
    Serial.print("+");
  }
  Serial.println(diffEvap);
  Serial.print("TΔ Rad Water: ");
  if(diffRad > 0) {
    Serial.print("+");
  }
  Serial.println(diffRad);
  Serial.print("TΔ Rad Air:   ");
  if(diffRadA > 0) {
    Serial.print("+");
  }
  Serial.println(diffRadA);
  Serial.println("----------");
  fanCount1 = 0;
  fanCount2 = 0;
  fanCount3 = 0;

  mqttConnect();

  mqtt.processPackets(10);

  if(pumpStat){
    mqtt.publish("servers/cooling/pump", "ON");
  } else {
    mqtt.publish("servers/cooling/pump", "OFF");
  }
  mqtt.publish("servers/cooling/fans/1", workAroundStupidAdafruitDevs(rpm1));
  mqtt.publish("servers/cooling/fans/2", workAroundStupidAdafruitDevs(rpm2));
  mqtt.publish("servers/cooling/fans/3", workAroundStupidAdafruitDevs(rpm3));
  mqtt.publish("servers/cooling/temps/tank", workAroundStupidAdafruitDevs(tankSensorTemp));
  mqtt.publish("servers/cooling/temps/water/inflow", workAroundStupidAdafruitDevs(inflowSensorTemp));
  mqtt.publish("servers/cooling/temps/water/outflow", workAroundStupidAdafruitDevs(outflowSensorTemp));
  mqtt.publish("servers/cooling/temps/air/ambient", workAroundStupidAdafruitDevs(ambientSensorTemp));
  mqtt.publish("servers/cooling/temps/air/postcool", workAroundStupidAdafruitDevs(postcoolSensorTemp));
  mqtt.publish("servers/cooling/temps/air/postrad", workAroundStupidAdafruitDevs(postradSensorTemp));
  mqtt.publish("servers/cooling/temps/delta/evap", workAroundStupidAdafruitDevs(diffEvap));
  mqtt.publish("servers/cooling/temps/delta/rad/water", workAroundStupidAdafruitDevs(diffRad));
  mqtt.publish("servers/cooling/temps/delta/rad/air", workAroundStupidAdafruitDevs(diffRadA));
  
  if(!mqtt.ping()) {
    mqtt.disconnect();
  }
  
  return true;
}

char * workAroundStupidAdafruitDevs(float a) {
  dtostrf(a,2,2,tempString);
  return tempString;
}


void printAddress(DeviceAddress addr) {
  char buffer[20];
  for (int j = 0; j < 8; j++) {
    sprintf(buffer, "%02x", addr[j]); 
    Serial.print(buffer);
  }
  Serial.println("");
}

void setSpeed(uint16_t speed) {
  spo = getSpeed(speed);
  analogWrite(FAN1_PWM, spo);
  analogWrite(FAN2_PWM, spo);
  analogWrite(FAN3_PWM, spo);
}

void checkSerialBuffer(void) {
//EG: AHQ45000V - 5000RPM
//EG: AHR0V    - Pump On
//EG: AHS0V    - Pump Off
  if (Serial.available() > 0) {
    char done = 0;
    char byIn = Serial.read();
    if(displayPacket && primed) {
      displayBuffer[displayCount]=byIn;
      displayCount++;
    }
    if(byIn == 'V' && !done) {
      // Done
      char type = displayBuffer[0];
      uint8_t pll = displayBuffer[1]-'0';
      Serial.print("Display packet:[");
      Serial.print("Type:");
      Serial.print(type);
      Serial.print(" PLL:");
      Serial.print(pll);
      if(pll > 0) {
        Serial.print(" PL:");
      }
      for(int i=2; i < (2 + pll); i++) {
        Serial.print(displayBuffer[i]);
      }
      if (type == 'Q') {
        uint8_t T = displayBuffer[2] - '0';
        uint8_t h = displayBuffer[3] - '0';
        uint8_t t = displayBuffer[4] - '0';
        uint8_t u = displayBuffer[5] - '0';
        uint16_t out = (T*1000) + (h * 100) + (t * 10) + u;
        setSpeed(out);
        Serial.print("<Fan:");
        Serial.print(out);
        Serial.println(">");
      }
      if (type == 'R') {
        Serial.print("Pump on");
        pumpStat=1;
        digitalWrite(PUMP_CTRL, 1);
        pumpCount = 0;
      }
      if (type == 'S') {
        Serial.print("Pump off");
        pumpStat=0;
        digitalWrite(PUMP_CTRL, 0);
      }
      Serial.println("]");
      primed = 0;
      displayCount = 0;
      displayPacket = 0;
      overrideTimer=129;
      done = 1;
    }
    if(byIn == 'A' && !done) {
      primed = 1;
      done = 1;
    }
    if(byIn == 'H' && !done) {
      if(primed == 1) {
        displayPacket = 1;
        displayCount = 0;
        done = 1;
      } else {
        primed = 0;
        done = 1;
      }
    }
  }
}

float avgInflow = 0;
float avgInflowCalc = 0;
uint8_t avgInflowCount = 0;

bool mainLoop(void *) {
  static uint8_t mainTick;
  static uint8_t fanPsc;
  mainTick++;
  int startT = millis();
  checkSerialBuffer();
  int tempVar;
  switch(mainTick) {
    case 1:
      sensors.requestTemperaturesByAddress(tankSensor);
      break;
    case 2:
      sensors.requestTemperaturesByAddress(inflowSensor);
      break;
    case 3:
      sensors.requestTemperaturesByAddress(outflowSensor);
      break;
    case 4:
      sensors.requestTemperaturesByAddress(ambientSensor);
      break;
    case 5:
      sensors.requestTemperaturesByAddress(postcoolSensor);
      break;
    case 6:
      sensors.requestTemperaturesByAddress(postradSensor);
      break;
    case 7:
      tankSensorTemp = sensors.getTempC(tankSensor) * 10;
      tempVar = tankSensorTemp;
      tankSensorTemp = (float)tempVar / 10;
      break;
    case 8:
      inflowSensorTemp = sensors.getTempC(inflowSensor) * 10;
      tempVar = inflowSensorTemp;
      inflowSensorTemp = (float)tempVar / 10;
      if(avgInflowCount++ < 20) {
        avgInflowCalc += inflowSensorTemp;
      } else {
        avgInflowCalc = avgInflowCalc / 20;
        avgInflow = avgInflowCalc;
        avgInflowCount = 0;
      }
      break;
    case 9:
      outflowSensorTemp = sensors.getTempC(outflowSensor) * 10;
      tempVar = outflowSensorTemp;
      outflowSensorTemp = (float)tempVar / 10;
      break;
    case 10:
      ambientSensorTemp = sensors.getTempC(ambientSensor) * 10;
      tempVar = ambientSensorTemp;
      ambientSensorTemp = (float)tempVar / 10;
      break;
    case 11:
      postcoolSensorTemp = sensors.getTempC(postcoolSensor) * 10;
      tempVar = postcoolSensorTemp;
      postcoolSensorTemp = (float)tempVar / 10;
      break;
    case 12:
      postradSensorTemp = sensors.getTempC(postradSensor) * 10;
      tempVar = postradSensorTemp;
      postradSensorTemp = (float)tempVar / 10;
      
      if(overrideTimer==1) {
        Serial.println("<< END OVERRIDE >>");
      }
      if(overrideTimer > 0) {
        overrideTimer--;
        mainTick = 0;
      }
      break;
    case 13:
      pumpCount++;
      if(pumpCount <= 15) {
        float diffEvap = postcoolSensorTemp - ambientSensorTemp;
        if(postcoolSensorTemp > outflowSensorTemp || outflowSensorTemp > inflowSensorTemp || (outflowSensorTemp > 22)) {
          pumpStat = 1;
          digitalWrite(PUMP_CTRL, 1);
          setSpeed(500);
        } else {
          pumpStat = 0;
          digitalWrite(PUMP_CTRL, 0);
        }
      } else {
        pumpStat = 0;
        digitalWrite(PUMP_CTRL, 0);
        if(fanPsc++ > 10) {
          fanPsc = 0;
          if(avgInflow < 20) {
            setSpeed(1500);  
          }
          if(avgInflow >= 20 && avgInflow < 22) {
            setSpeed(2000);
          }
          if(avgInflow >= 22 && avgInflow < 24) {
            setSpeed(2500);
          }
          if(avgInflow >= 24 && avgInflow < 26) {
            setSpeed(3500);
          }
          if(avgInflow >= 26) {
            setSpeed(4000);
          }
        }
      }
      if(avgInflow <= 23 && pumpCount>=150){
        pumpCount = 0;
      }
      if(avgInflow > 23 && avgInflow < 25 && pumpCount>=60){
        pumpCount = 0;
      }
      if(avgInflow > 24 && avgInflow < 26 && pumpCount>=55){
        pumpCount = 0;
      }
      if(avgInflow > 25 && avgInflow < 27 && pumpCount>=50){
        pumpCount = 0;
      }
      if(avgInflow > 26 && pumpCount>=45){
        pumpCount = 0;
      }
      break;
    default:
      mainTick = 0;
      break;
  }
  return true;
}

void onoffCallback(char *data, uint16_t len) {
  Serial.print(">>> Pump request:");
  Serial.println(data);
  if(strcmp(data, "ON") == 0){
    Serial.println("Pump On");
    pumpStat=1;
    digitalWrite(PUMP_CTRL, 1);
    pumpCount = 0;
    overrideTimer=129;
  }
  if(strcmp(data, "OFF") == 0){
    Serial.println("Pump Off");
    pumpStat=0;
    digitalWrite(PUMP_CTRL, 0);
    overrideTimer=129;
  }
}

void fanCallback(uint32_t speed) {
  Serial.print(">>> Fan request:");
  Serial.println(speed);
  setSpeed(speed);
  Serial.print("<Fan:");
  Serial.print(speed);
  Serial.println(">");
  overrideTimer=129;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
    // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  Serial.println("WiFi connected");
  Serial.println("IP address: "); 
  Serial.println(WiFi.localIP());
  
  pinMode(FAN1_TACH, INPUT_PULLUP);
  pinMode(FAN2_TACH, INPUT_PULLUP);
  pinMode(FAN3_TACH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FAN1_TACH), pinChanged1, RISING);
  attachInterrupt(digitalPinToInterrupt(FAN2_TACH), pinChanged2, RISING);
  attachInterrupt(digitalPinToInterrupt(FAN3_TACH), pinChanged3, RISING);
  spo = getSpeed(2500);
  pinMode(PUMP_CTRL, OUTPUT);   
  digitalWrite(PUMP_CTRL,1);
  analogWrite(FAN1_PWM, spo);
  analogWrite(FAN2_PWM, spo);
  analogWrite(FAN3_PWM, spo);
  timer.every(1000, resetCount);
  timer.every(100, mainLoop);
  
  sensors.setWaitForConversion(false);
  Serial.println("Scanning for 1Wire Devices");
  // Start up the library
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  uint8_t count=sensors.getDeviceCount();
  
  sensors.setResolution(12);
  
  Serial.print("Found ");
  char xx = '0' + count;
  Serial.print(xx);
  Serial.println(" devices");
  if(count > 4) {
    count == 4;
  }
  for(int i=0; i<count; i++) {
    DeviceAddress addr;
    sensors.getAddress(addr,i);
    printAddress(addr);
    if(addr[7] == 0x86) {
      Serial.println("Found tankSensor");
      sensors.getAddress(tankSensor,i);
    }
    if(addr[7] == 0xC2) {
      Serial.println("Found inflowSensor");
      sensors.getAddress(inflowSensor,i);
    }
    if(addr[7] == 0x3C) {
      Serial.println("Found outflowSensor");
      sensors.getAddress(outflowSensor,i);
    }
    if(addr[7] == 0x0C) {
      Serial.println("Found ambientSensor");
      sensors.getAddress(ambientSensor,i);
    }
    if(addr[7] == 0x3D) {
      Serial.println("Found postcoolSensor");
      sensors.getAddress(postcoolSensor,i);
    }
    if(addr[7] == 0x71) {
      Serial.println("Found postradSensor");
      sensors.getAddress(postradSensor,i);
    }
  }
  pumpOnOff.setCallback(onoffCallback);
  mqtt.subscribe(&pumpOnOff);
  fanSpeed.setCallback(fanCallback);
  mqtt.subscribe(&fanSpeed);
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void mqttConnect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 1 second...");
       mqtt.disconnect();
       delay(1000);
       retries--;
       if (retries == 0) {
        Serial.println("Failed to connect");
        return;
       }
  }
  Serial.println("MQTT Connected!");
}

void loop() {
  timer.tick();
}
