# DeskBuddy Device Controller


### Register modified LCD library


https://raw.githubusercontent.com/tonykambo/LiquidCrystal_I2C/master/library.json


### Add dependency directly to library

```
[env:nodemcuv2]
platform = espressif8266
board = nodemcuv2
framework = arduino
lib_deps =
  https://github.com/tonykambo/LiquidCrystal_I2C.git
```
