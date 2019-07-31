# Temperature and Light Intensity Sensor

## Description
This project is about displaying the value of sensor in LCD screen. 

## Tools
- STM32F103 Blue Pill Microcontroller, [Link](https://jeelabs.org/img/2016/DSC_5474.jpg)
- STM32F103 Smart V2 Microcontroller, [Link](https://stm32-base.org/assets/img/boards/STM32F103C8T6_STM32_Smart_V2.0-1.jpg)
- 16 x 2 LCD Screen,  [Link](https://5.imimg.com/data5/MU/MN/MY-25117786/16x2-lcd-display-green-500x500.jpg)
- Light Intensity Sensor (OPT 101), [Link](https://www.aam.com.pk/wp-content/uploads/2018/03/opt101.jpg)
- Temperature Sensor (NTC), [Link](https://ae01.alicdn.com/kf/HTB13TzPSFXXXXaWXFXXq6xXFXXXq/100K-Ohm-NTC-3950-Thermistors-with-Cable-for-3D-Printer-Reprap-Mend.jpg_640x640.jpg)

## Connections
Blue Pill MCU | Smart V2 MCU
--------------|-------------
PB0           | R (NRST)
PB12, PB14    | PA13 (SWDIO)
PB13, PA5     | PA14 (SWCLK)

- PB0 of Blue Pill MCU should connected to a resistor of 110Ω before connect R of Smart V2 MCU.
- PB14 of Blue Pill MCU also need to connected to a resistor of 110Ω before connect PA13 of Smart V2 MCU.
- PB13 and PA5 of Blue Pill MCU connected to a same resistor of 110Ω before connect PA14 of Smart V2 MCU.

