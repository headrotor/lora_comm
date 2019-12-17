/*

  upstairs reciever
  recieves  mailbox tilt, temperature, air pressure, sends to upstairs
  sends recieves LED commands

*/
#include "heltec.h"

#define BAND    433E6  //you can set band here directly,e.g. 868E6,915E6

// for parsing and displaying input data
#define PACKET_BYTES 32
char lora_str[PACKET_BYTES];
float temp_C = -1.;
float temp_F = -1.;
float baro_mb = -1.;
float angx_deg = -1.;

// for handling serial input
char inbuff[PACKET_BYTES];
int cmd_len = 0;


String outgoing;              // outgoing message

byte localAddress = 0xFD;     // address of this device
byte destAddress = 0xBB;      // destination to send to

byte msgCount = 0;            // count of outgoing messages
unsigned long lastSendTime = 0;    // millis() of last send time
unsigned long lastSec = 0;        // miils() from last second for second timer
int interval_ms = 10000;          // interval between sends
unsigned int watchdog_seconds  = 0;                //watchdog timer: complain when no signal for 60 seconds

unsigned int mail_seconds = 0;

void setup()
{
  char buff[PACKET_BYTES];

  //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.LoRa Enable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);

  snprintf(buff, PACKET_BYTES, "addr:02x%, dest:%02x", localAddress, destAddress);
  Serial.println("upstairs: " + String(buff));

  Heltec.display->init();
  Heltec.display->flipScreenVertically();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(0, 0, "upstairs");
  Heltec.display->drawString(0, 20, buff );
  Heltec.display->display();


}

void loop()
{
  unsigned long now;
  now = millis();
  if (now - lastSendTime > interval_ms)


  {
    String message = "Upstairs says hello";   // send a message
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = now;            // timestamp the message
  }

  // update display every second
  if (now - lastSec > 1000)
  {
    update_display();
    lastSec = now;            // timestamp the message
    ++watchdog_seconds;
  }

  while (Serial.available() > 0) {
    // read the incoming bytes:
    add_command(Serial.read());
  }


  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  delay(20);
  digitalWrite(LED, LOW);   // turn the LED on (HIGH is the voltage level)

}


void add_command(char inbyte) {

  if (inbyte == '\n') {
    inbuff[cmd_len] = '\0';
    Serial.println("got str: " + String(inbuff));
    parse_serial_msg(inbuff);
    cmd_len = 0;
    return;
  }
  if (cmd_len < PACKET_BYTES - 1) {
    inbuff[cmd_len++] = inbyte;
  }

}


void sendMessage(String outgoing)
{
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  LoRa.beginPacket();                   // start packet
  LoRa.setTxPower(20, RF_PACONFIG_PASELECT_PABOOST);
  LoRa.write(destAddress);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}



void parse_lora_cmd(String in) {
  float tempf;
  in.toCharArray(lora_str, PACKET_BYTES);

  //Serial.println("parsing: " + in);
  if (sscanf(lora_str, "temp: %f", &tempf)) {
    temp_C = tempf;
    temp_F = 1.8 * tempf + 32.;
    //Serial.print("got temp:");
    //Serial.println(tempf);
  }

  else if (sscanf(lora_str, "angx: %f", &tempf)) {
    angx_deg = tempf;
  }

  else if (sscanf(lora_str, "baro: %f", &tempf)) {
    baro_mb = tempf;
  }

  else if (sscanf(lora_str, "mail: %5d", &mail_seconds)) {
  }
}

void parse_serial_msg(char *cmd) {
  int tempi;
  int r, g, b;
  Serial.println("parsing serial: " + String(cmd));
  if (sscanf(cmd, "disp: %d", &tempi)) {
    //Serial.print("got display command ");
    //Serial.println(tempi);
    String scmd = String(cmd);
    sendMessage(scmd);
  }

  else if (sscanf(cmd, "rgb: %i %i %i", &r, &g, &b) == 3) {
    //Serial.print("got rgb command");
    //Serial.println(cmd);
    String scmd = String(cmd);
    sendMessage(scmd);
  }

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
    delayMicroseconds(100);
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
  parse_lora_cmd(incoming);
  watchdog_seconds = 0;
  update_display();


  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
}

void update_display() {

  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_16);
  Heltec.display->drawString(3, 0, "temp: " + String(temp_F) + "F");
  Heltec.display->drawString(3, 18, "baro: " + String(baro_mb) + "mb");

  if (watchdog_seconds > 60) {
    Heltec.display->drawString(3, 36, "WATCHDOG " + String(int(floor(watchdog_seconds / 60.))) + " min ago");

  }
  else {
    if (mail_seconds) {
      if (mail_seconds < 3600) {
        Heltec.display->drawString(3, 36, "mail " + String(int(floor(mail_seconds / 60.))) + " min ago");
      } else {
        Heltec.display->drawString(3, 36, "mail " + String(int(floor(mail_seconds / 3600.))) + " hrs ago");
      }
    } else {
      Heltec.display->drawString(3, 36, "no mail yet");
    }
  }
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(3, 54, String("RSSI: " + String(LoRa.packetRssi())));
  Heltec.display->drawString(Heltec.display->getWidth() / 2, 54, String("SNR: " + String(LoRa.packetSnr())));
  Heltec.display->display();

}
