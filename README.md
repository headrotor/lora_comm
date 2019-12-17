# lora_comm
Code for bidirectional LoRa communication using Heltec WiFi LoRa 32(V2)


## downstairs_sender: 
Connected to BMP680 temp/pressure sensor and MPU6050 accelerometer. Sends LoRa messages with temp, barometric pressure, tilt, and time since y tilt has exceeded 5 degrees (attached to mailbox, when delivered will tilt). 
## upstairs_receiever:
Receives and displays LoRa messages from downstairs sender. Can also send LoRa messages to change neopixel LED color and blank display.

## outdoor_server.py:
receives and sends serial commands to upstairs_receiver, can log temp pressure, and time of mail arrival and also turn on/off downstairs LEDs and display
