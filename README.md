# esp8266-WindStation
Simply and low cost weather station on ESP8266 for windguru.cz and narodmon.com for Arduino

An example of building a weather station on the ESP8266 Wemos D1 mini board (cost ~$4), Davis Anemometer (cost ~$120) and DHT11 (cost ~$1):
![alt tag](https://github.com/zpukr/esp8266-WindStation/blob/master/windstation.jpg)

Installation of the humidity/temperature sensor DHT-11 is option. Also, to minimize final costs, instead of the Davis Anemometer, you can use the cheap La Crosse TX23U sensor (cost ~$50). Or even build an anemometer yourself, example on russian http://skootsone.yolasite.com/wind-pow-02.php

Writing a program to the ESP8266 with Arduino IDE. After first run ESP start as Access Point mode with SSID "WindStationAP" and default password "87654321", spins up a DNS and WebServer (default ip 192.168.4.1). Using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point. Set some parameters and click "Save":
![alt tag](https://github.com/zpukr/esp8266-WindStation/blob/master/WindStationAP.jpg)

ESP will try connect to internet. If successful, setting save to flash memory. If not, start as Access Point again
