/*
  downstairs sender

  detects mailbox tilt, temperature, air pressure, sends to upstairs
  recieves LED commands

*/

// Board: Heltec WiFi LoRa 32(V2)
// pinout https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series/blob/master/PinoutDiagram/WIFI_LoRa_32_V2.pdf

#include <Wire.h>
#include "heltec.h"


#include "FastLED.h"

#include <MPU6050_tockn.h>
#include "Adafruit_BME680.h"



//timing:
unsigned long lastSendTime = 0;        // last send time
unsigned long lastSecond = 0;        // use for second ticker

unsigned long  interval_ms = 5050;          // interval between sends

unsigned long mail_seconds = 0; // seconds since mail indicator was lit
float mail_timeout_hrs = 10; // time out mail indicator after this many hours lit
int mail_delay_sec = 0;    // Light mail indicator after this many seconds
int mail_open_seconds = 0;  // open mailbox this many seconds before starting mail count.
#define MAIL_OPEN_SECONDS (5)  // open mailbox this many seconds before starting mail count.
float tilt_thresh_deg = 10.0; // tilt more than this to trigger mail sensor


// Pinout:
// FastLED data out
#define NEOPIXEL_PIN 13
//#define NEOPIXEL_PIN 0

// Put all I2C hardware on this bus.
#define PIN_SDA 4
#define PIN_SCL 15
// turn on this pin to enable 3.3 VEXT
#define VEXT_PIN 21
#define DOOR_PIN 23



//fastled
#define NUM_LEDS 24
CRGB leds[NUM_LEDS];


#define SEALEVELPRESSURE_HPA (1013.25)
//   Adafruit_BME680 (TwoWire *theWire=&Wire)
Adafruit_BME680 bme; // I2C

// MPU6050 9-dof sensor to get mailbox tilt
MPU6050 mpu6050(Wire);



// opcodes: temperature, angle x, barometric pressure, LED
// do gas after temp so max time to cool down?
#define TEMP_OP 0
#define BARO_OP 1
#define GASR_OP 2
#define HUMID_OP 3
#define MAIL_OP 4
#define ANGY_OP 5
#define SHAKE_OP 6
#define DOOR_OP 7

// for stater machine, cycle through sending op codes
int state = 0;
#define MAX_STATE 8


// LOra stuff
#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6

#define PACKET_BYTES 32
char lora_str[PACKET_BYTES];

String outgoing;              // outgoing message



byte localAddress = 0xBB;     // address of this device
byte destAddress = 0xFD;      // destination to send to


float max_shake = 0.;  // maximum mag acceleration since we last checked
float angle; // last detected y tilt

// set when garage door is open
int door_flag = 0;

void setup()
{
  char buff[PACKET_BYTES];

  //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);

  // Turn on VEXT
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, LOW);


  // set up door sensor input
  pinMode(DOOR_PIN, INPUT_PULLUP);


  snprintf(buff, PACKET_BYTES, "addr:02x%, dest:%02x", localAddress, destAddress);
  Serial.println("downstairs:" + String(buff));


  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, "downstairs" );
  Heltec.display->drawString(0, 20, buff );
  Heltec.display->display();

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    //while (1);
  }



  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms



  FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds, NUM_LEDS);
  // turn off if on
  set_rgb(0,   0, 0);


}

void set_rgb(byte r, byte g, byte b) {
  int i = 0;
  for (i = 0;  i < NUM_LEDS; i++) {
    /*
        if (i < 4) {
          leds[i].setRGB(r, g, b);
        } else {
          leds[i].setRGB(0, 0, 0);

        }
    */
    leds[i].setRGB(r, g, b);

  }
  FastLED.show();
}

void send_next_msg() {

  switch (state) {

    case TEMP_OP:
      if (! bme.performReading()) {
        Serial.println("Error reading BME");
        return;
      }
      snprintf(lora_str, PACKET_BYTES, "temp: %4.1f", bme.temperature - 3.0 );
      break;
    case ANGY_OP:


      snprintf(lora_str, PACKET_BYTES, "angy: % 5.2f", angle);
      break;

    case MAIL_OP:
      snprintf(lora_str, PACKET_BYTES, "mail: %5d", mail_seconds);
      break;


    case  BARO_OP:
      if (! bme.performReading()) {
        Serial.println("Error reading BME");
        return;
      }
      snprintf(lora_str, PACKET_BYTES, "baro: % 5.2f", bme.pressure / 100.0);
      break;

    case  GASR_OP:
      if (! bme.performReading()) {
        Serial.println("Error reading BME");
        return;
      }
      snprintf(lora_str, PACKET_BYTES, "gasr: % 7.2f", bme.gas_resistance / 1000.0);
      break;

    case  HUMID_OP:
      if (! bme.performReading()) {
        Serial.println("Error reading BME");
        return;
      }
      snprintf(lora_str, PACKET_BYTES, "humid: % 7.2f", bme.humidity );
      break;

    case  SHAKE_OP:
      snprintf(lora_str, PACKET_BYTES, "shake: % 5.3f", max_shake);
      max_shake = 0;
      break;

    case  DOOR_OP:
      snprintf(lora_str, PACKET_BYTES, "door: %1d", door_flag);
      break;

    default:
      break;
  }

  sendMessage(String(lora_str));
  Serial.print("Sending ");
  Serial.println(lora_str);

  ++state;
  if (state >= MAX_STATE) {
    state = 0;
  }

}

void loop()
{

` // fixed 24 feb 2020, forgot unsigned! 
  unsigned long now = (unsigned long) millis();
  
  if (now - lastSendTime > interval_ms) {

    send_next_msg();
    lastSendTime = millis();            // timestamp the message
  }

  if (now - lastSecond > (unsigned long) 1000) {

    mpu6050.update();
    angle = mpu6050.getAccAngleY();

    if (mail_seconds) {
      ++mail_seconds;
    }


    if (abs(angle) > tilt_thresh_deg) {
      ++mail_open_seconds;
      if (mail_open_seconds > MAIL_OPEN_SECONDS){
        // start counting
        ++mail_seconds;
      }
    }
    else {
      mail_open_seconds = 0;
    }

    //Serial.println(mail_open_seconds);

    if (digitalRead(DOOR_PIN) == HIGH) {
      door_flag = 1;

    }
    else {
      door_flag = 0;

    }

    //Serial.print("Door flag:");
    //Serial.println(door_flag);

    update_display();
    lastSecond = now;
  }


  if (mail_seconds > (long int) 3600 * mail_timeout_hrs) {
    mail_seconds = 0;
    mail_open_seconds = 0;
    set_rgb(0, 0, 0);
  }
  if (mail_seconds > mail_delay_sec) {
    set_rgb(0, 0xFF, 0);
  }
  else {
    set_rgb(0, 0, 0);

  }
  delay(1);



  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  get_shake();
  digitalWrite(LED, LOW);   // turn the LED on (HIGH is the voltage level)

}



void get_shake() {
  float acc = 0;
  mpu6050.update();
  float comp = mpu6050.getAccX();
  acc = comp * comp;
  comp = mpu6050.getAccY();
  acc += comp * comp;
  comp = mpu6050.getAccZ();
  acc += comp * comp;

  acc = sqrt(acc);
  if (acc > max_shake) {
    max_shake = acc;
  }
}

void sendMessage(String outgoing)
{
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  LoRa.beginPacket();
  LoRa.setTxPower(20, RF_PACONFIG_PASELECT_PABOOST);
  // start packet
  LoRa.write(destAddress);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(0xAA);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
}

void onReceive(int packetSize)
{
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available())
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length())
  { // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from : 0x" + String(sender, HEX));
  Serial.println("Sent to : 0x" + String(recipient, HEX));
  Serial.println("Message ID : " + String(incomingMsgId));
  Serial.println("Message length : " + String(incomingLength));
  Serial.println("Message : " + incoming);
  parse_lora_cmd(incoming);
  Serial.println("RSSI : " + String(LoRa.packetRssi()));
  Serial.println("Snr : " + String(LoRa.packetSnr()));
  Serial.println();
}


void parse_lora_cmd(String in) {
  int r, g, b;
  int tempi;
  in.toCharArray(lora_str, PACKET_BYTES);

  //Serial.println("parsing: " + in);
  if (sscanf(lora_str, "disp: %i", &tempi)) {
    /*
      Serial.print("got LoRa disp command: ");
      Serial.println(lora_str);
      Serial.println(tempi);
    */
    if (tempi) {
      Heltec.display->displayOn();
    }

    else {
      Heltec.display->displayOff();
    }
  }


  else if (sscanf(lora_str, "rgb: %i %i %i", &r, &g, &b) == 3) {
    /*
      Serial.print("got LoRa rgb command n=");
         Serial.println("r " + String(r));
      Serial.println("g " + String(g));
      Serial.println("b " + String(b));
      Serial.println(lora_str);
    */
    set_rgb(r, g, b);
  }

}

void update_display() {

  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(3, 0, "tilt: " + String(angle));
  //Heltec.display->drawString(3, 0, "shake: " + String(max_shake));
  Heltec.display->drawString(3, 18, "seconds: " + String(mail_seconds));
  Heltec.display->drawString(3, 36, "alarm: ARMED");
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(3, 54, String("RSSI : " + String(LoRa.packetRssi())));
  Heltec.display->drawString(Heltec.display->getWidth() / 2, 54, String("SNR : " + String(LoRa.packetSnr())));
  Heltec.display->display();

}
