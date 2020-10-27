# synced-to-the-beat
A raspberry pi/ESP32 project involving musical beat detection and stepper motors

## ESP32s
I used the 38-pin ESP32 development boards available from A-Z Delivery <https://az-delivery.com/products/esp32-developmentboard> or Amazon <https://www.amazon.com/AZDelivery-Nodemcu-CP2102-Module-Development/dp/B07H3TTCCW/>

## Silent Step Stick
This is the TMC2130 chip made by Trinamic. Info <here https://www.trinamic.com/support/eval-kits/details/silentstepstick/>. _Note that you need the SPI version of hte development board._

## PCB Parts List
Frizting file is included. Parts list:
- R1: 510 ohm
- R2: 39 ohm  1W
- C5: 1uf 25V
- C6: 10uF 25V
- C7: 10uF 25V
- C8: 100nF
- C9: 1uF
- D2: 1N5817

TMC2130 Silent Stepstick dev board
IRLB8721 N-Channel MOSFET
VS1838B IR Receiver
- 
