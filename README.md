# TMP117-STM32-HAL-Driver
STM32 HAL-based library for the TMP117 temperature sensor by Texas Instruments

## Sensor Documentation
The datasheet is available [here](https://www.ti.com/lit/ds/symlink/tmp117.pdf?ts=1650967044922&ref_url=https%253A%252F%252Fwww.google.com%252F).

## Functionality
- [x] Sensor's initialization
- [x] Sensor's configuration: <br>
      - Set averaging <br>
      - Set alert mode <br>
      - Set alert polarity <br>
      - Set conversion mode <br>
      - Set conversion time <br>
      - Set alert mode <br>
      - Software reset <br> 
      - Get configuration register value <br>
      - Get device ID
- [x] Set the low and high temperature limits
- [x] Read temperature: <br>
      - Get the result temperature <br>
      - Get the offset temperature <br>
      - Get the high-limit temperature <br>
      - Get the low-limit temperature <br>
- [x] Calibration
- [x] Read / Write the sensor's EEPROM 
