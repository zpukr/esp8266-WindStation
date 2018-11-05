# esp8266-WindStation
Simply and very low cost weather station on ESP8266 board for windguru.cz and narodmon.com for Arduino

An example of building a weather station on the ESP8266 Wemos D1 mini board (cost ~$4), Davis Anemometer ( ~$120) and DHT11 ( ~$1):
![alt tag](https://github.com/zpukr/esp8266-WindStation/blob/master/windstation.jpg)

Installation of the humidity/temperature sensor DHT-11 is option. You can set DHT-22 ( ~3$) or DHT-21 ( ~4$) instead it, which has a lot more accuracy and can show negative temperatures. Also, to minimize final costs, instead of the Davis Anemometer, you can use the cheap La Crosse TX23U sensor ( ~$50). Or even build an anemometer yourself from old computer fan, example on russian http://skootsone.yolasite.com/wind-pow-02.php

Flash a program to the ESP8266 with Arduino IDE. After first run ESP start as Access Point mode with SSID "WindStationAP" and default password "87654321", spins up a DNS and WebServer (default ip 192.168.4.1). Using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point. Set some parameters and click "Save":
![alt tag](https://github.com/zpukr/esp8266-WindStation/blob/master/WindStationAP.jpg)

After this ESP will try connect to internet. If successful, settings save to flash memory. If not, all settings are reset and WindStation start as Access Point again

Optionally, the station supports Deep Sleep mode for cases when only power is available from batteries or solar panels. In this mode, the consumption of approximately 6mAh (15 sec active/ 5min sleep) whereas in normal mode 80mAh
