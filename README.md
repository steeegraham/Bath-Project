# Bath-Project ESP32cam MQTT
Run bath using ESP32cam

Aim - Run bath triggered by various inputs: button, online button, alexa. Stops when reaches water sensor or after 10mins. Trigger can also be used to top the bath up ie. can stop within the 10 min time. Using board ESP32Cam to also stream running tap/bath level.



## Hardware

ESP32-CAM

5V power supply

Push button Switch

Relay 

240v tap solenoid

Water leak sensor




## Flashing

* ESP32-CAM does not have a built-in UART/USB converter, therefore you will need one.

  1. TX goes to RX;
  2. RX goes to TX;
  3. Power the board while keeping `GIPO0` low;
  4. You can now release it and upload your code;

[![ESP32-CAM](https://i2.wp.com/randomnerdtutorials.com/wp-content/uploads/2019/03/ESP32-CAM-wiring-FTDI1.png?w=750&ssl=1)](https://randomnerdtutorials.com/esp32-cam-troubleshooting-guide/)
 * image from [https://randomnerdtutorials.com/esp32-cam-troubleshooting-guide/](https://randomnerdtutorials.com/esp32-cam-troubleshooting-guide/)
