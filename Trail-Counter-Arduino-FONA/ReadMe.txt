
  Trail-Counter-Arduino-FONA
  Project
  ----------------------------------
  Developed with embedXcode

  Project Trail-Counter-Arduino-FONA
  Created by Charles McClelland on 5/15/16
  Copyright © 2016 Charles McClelland
  Licence GNU General Public Licence

/* Chip McClelland - Cellular Data Logger
BSD license, Please keep my name and the required Adafruit text in any redistribution

Rewritten to remove delay() and all use of Strings

Requirements:
- Account on Ubidots.  http://www.ubidots.com
- Adafruit FONA GPRS Card - Listed below
- I used a 3.3V 8MHz Arduino Pro Mini from Sparkfun - https://www.sparkfun.com/products/11114
- I also used the Sparkfun MMA8452 Accelerometer - https://www.sparkfun.com/products/12756
- There are a couple switches, pots and LEDs on my Carrier board -  https://oshpark.com/shared_projects/ygCgpmMP
- In V5, I added a DS1339 I2C Real Time Clock to make reporting more regular
- For Solar Power, I added a LiPo Fuel Gauge - LucaDentella's library - https://github.com/lucadentella/ArduinoLib_MAX17043

- Some changes to accomodate the Pro Micro
- Pin 0 - Int 2 - Clock
- Pin 1 - Int 3 - Accelerometer
- Pin 7 - Int 4 - Magnetometer
- Key is moved to pin 20

I made use of the Adafruit Fona library and parts of the example code
/***************************************************
This is an example for our Adafruit FONA Cellular Module

Designed specifically to work with the Adafruit FONA
----> http://www.adafruit.com/products/1946
----> http://www.adafruit.com/products/1963

These displays use TTL Serial to communicate, 2 pins are required to
interface
Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada for Adafruit Industries.
BSD license, all text above must be included in any redistribution
****************************************************/



  References
  ----------------------------------


  embedXcode
  embedXcode+
  ----------------------------------
  Embedded Computing on Xcode
  Copyright © Rei VILO, 2010-2016
  All rights reserved
  http://embedXcode.weebly.com

