# esp8266_bme680
连接homekit的环境传感器.


1. 传感器 BME680 3.3的集成模块
2. ESP8266-01S
3. 3.7v电池, 使用AMS1117转成3.3
4. AMS1117 3.3V



使用的arduino库:
1. HomeKit-ESP8266
2. BSEC Software Library
3. BME68X Sensor Library
4. Adafruit BME680 Library


BME680<->ESP8266-01S连线:
1. IO2 -> SDA
2. IO0 -> SCL


最终效果:
1. 