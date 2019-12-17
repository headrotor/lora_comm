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



//timing:
unsigned long lastSendTime = 0;        // last send time
unsigned long lastSecond = 0;        // use for second ticker

int interval_ms = 5000;          // interval between sends

int mail_seconds = 0; // seconds since mail indicator was lit
float mail_timeout_hrs = 10; // time out mail indicator after this many hours lit
int mail_delay_sec = 30;    // Light mail indicator after this many seconds


float tilt_thresh_deg = 5.0; // tilt more than this to trigger mail sensor

//fastled
#define NUM_LEDS 24
CRGB leds[NUM_LEDS];
#define NEOPIXEL_PIN 22

#include <MPU6050_tockn.h>
#include "Adafruit_BME680.h"


#define SEALEVELPRESSURE_HPA (1013.25)
//   Adafruit_BME680 (TwoWire *theWire=&Wire)
Adafruit_BME680 bme; // I2C

// MPU6050 9-dof sensor to get mailbox tilt
MPU6050 mpu6050(Wire);

// Put all I2C hardware on this bus.
#define PIN_SDA 4
#define PIN_SCL 15



// opcodes: temperature, angle x, barometric pressure, LED

#define TEMP_OP 0
#define ANGY_OP 1
#define BARO_OP 2
#define MAIL_OP 3
#define GASR_OP 4
#define SHAKE_OP 5

// for stater machine, cycle through sending op codes
int state = 0;
#define MAX_STATE 6


// LOra stuff
#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6

#define PACKET_BYTES 32
char lora_str[PACKET_BYTES];

String outgoing;              // outgoing message



byte localAddress = 0xBB;     // address of this device
byte destAddress = 0xFD;      // destination to send to


float max_shake = 0.;

void setup()
{
  char buff[PACKET_BYTES];

  //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);

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


  FastLED.addLeds<NEOPIXEL, 22>(leds, NUM_LEDS);
  // turn off if on
  set_rgb(0, 0, 0);
  delay(1000);

}

void set_rgb(byte r, byte g, byte b) {
  int i = 0;
  for (i = 0;  i < NUM_LEDS; i++) {
    if (i < 4) {
      leds[i].setRGB(r, g, b);
    } else {
      leds[i].setRGB(0, 0, 0);

    }
  }
  FastLED.show();
}

void send_next_msg() {
  float angle;
  switch (state) {

    case TEMP_OP:
      if (! bme.performReading()) {
        Serial.println("Error reading BME");
        return;
      }
      snprintf(lora_str, PACKET_BYTES, "temp: %4.1f", bme.temperature );
      break;
    case ANGY_OP:

      mpu6050.update();
      angle = mpu6050.getAccAngleY();

      if (abs(angle) > tilt_thresh_deg) {
        mail_seconds = 1;
        Serial.println("Mail detected!");
      }

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

    case  SHAKE_OP:
      snprintf(lora_str, PACKET_BYTES, "shake: % 5.3f", max_shake);
      max_shake = 0;
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

  unsigned long now = millis();

  if (now - lastSendTime > interval_ms) {

    send_next_msg();
    lastSendTime = now;            // timestamp the message
  }

  if (now - lastSecond > 1000) {

    if (mail_seconds) {
      ++mail_seconds;
    }
    update_display();
    lastSecond = now;
  }


  if (mail_seconds > int(3600 * mail_timeout_hrs)) {
    Serial.println("Mail indicator off");
    mail_seconds = 0;
    set_rgb(0, 0, 0);
  }
  if (mail_seconds > mail_delay_sec) {
    Serial.println("Mail indicator on");
    set_rgb(0, 0xFF, 0);
  }



  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  get_shake();
  digitalWrite(LED, LOW);   // turn the LED on (HIGH is the voltage level)

}



void get_shake() {
  float acc = 0;
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

  mpu6050.update();
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(3, 0, "tilt: " + String(mpu6050.getAccAngleY()));
  //Heltec.display->drawString(3, 0, "shake: " + String(max_shake));
  Heltec.display->drawString(3, 18, "seconds: " + String(mail_seconds));
  Heltec.display->drawString(3, 36, "alarm: ARMED");
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(3, 54, String("RSSI : " + String(LoRa.packetRssi())));
  Heltec.display->drawString(Heltec.display->getWidth() / 2, 54, String("SNR : " + String(LoRa.packetSnr())));
  Heltec.display->display();

}
