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
|Blue Pill MCU | Smart V2 MCU|
| :----------: |:-----------:|
|PB0           | R (NRST)    |
|PB12, PB14    | PA13 (SWDIO)|
|PB13, PA5     | PA14 (SWCLK)|

Connection of target(Smart V2 MCU) and program(Blue Pill MCU) 
- PB0 of Blue Pill MCU should connected to a resistor of 110Ω before connect R of Smart V2 MCU.
- PB14 of Blue Pill MCU also need to connected to a resistor of 110Ω before connect PA13 of Smart V2 MCU.
- PB13 and PA5 of Blue Pill MCU connected to a same resistor of 110Ω before connect PA14 of Smart V2 MCU.

Connection of Temperature sensor and microcontroller(Smart V2 MCU)
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/schematic_temp.png)

Connection of Light Intensity sensor and microcontroller(Smart V2 MCU)
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/light%20schematic.JPG)

Connection of LCD and microcontroller(Smart V2 MCU)
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/LCD_Diagram.PNG)

|No. Pin | Function | Name |
|:----------: |:-----------:|:-----------:|
| 1 | Ground (0V) | Ground |
| 2 | Supply voltage; 5V (4.7V – 5.3V) | Vcc |
| 3 | Contrast adjustment; the best way is to use a variable resistor such as a potentiometer. The output of the potentiometer is connected to this pin. Rotate the potentiometer knob forward and backwards to adjust the LCD contrast. | Vo/VEE |
| 4 | Selects command register when low, and data register when high | RS (Register Select) |
| 5 | Low to write to the register; High to read from the register | Read/Write |
| 6 | Sends data to data pins when a high to low pulse is given; Extra voltage push is required to execute the instruction and EN(enable) signal is used for this purpose. Usually, we make it en=0 and when we want to execute the instruction we make it high en=1 for some milliseconds. After this we again make it ground that is, en=0. | ENABLE |
| 7 | 8-bit data pins | D0 |
| 8 | 8-bit data pins | D1 |
| 9 | 8-bit data pins | D2 |
| 10 | 8-bit data pins | D3 |
| 11 | 8-bit data pins | D4 |
| 12 | 8-bit data pins | D5 |
| 13 | 8-bit data pins | D6 |
| 14 | 8-bit data pins | D7 |
| 15 | Backlight VCC (5V) | LED+ |
| 16 | Backlight Ground (0V) | LED- |
- The data pin was connected to the microcontroller
- Data can transfer in 4-bit mode or 8-bit mode, if 4-bit mode was choosen D0 - D3 was not in used, while in 8-bit mode all the pin D0 - D7 are used.

## Setup in CubeMX
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/cubemx_config.JPG)
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/GPIO%20pin.JPG)
- Configure 2 analog pin(1 for temperature sensor, 1 for light intensity sensor)(can choose any pin that have ADC configuration)
- Configure 8 output pin for data 
- Configure 3 output pin for register select, read/write and enable
- Configure the pin of data to OUTPUT OPEN DRAIN mode, others as OUTPUT PUSH PULL mode

## Setup in LCD
- Initalise the LCD before sending anything to LCD. Refer to --> [Link](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Src/LCD.c), function lcdInit().
- To display on LCD, can send LCD command to let the LCD know what to do, LCD command can refer to [Link](https://electronicsforu.com/resources/learn-electronics/16x2-lcd-pinout-diagram)
- For displaying string or number in float, can configure in the eclipse with following step :

--> Project >> Properties >> C/C++ Builder >> Setting >> MCU GCC Linker >> Miscellaneous >> Linker Flags >> add -specs=nosys.specs -specs=nano.specs -u _printf_float

--> also can refer to [Link](https://github.com/chaosAD/Semihosting), for guiding to linker to change the linker flags.

- -u _printf_float was used for printing the float number.
- Modify the __io_putchar function by calling the lcdWriteMsg function to display on LCD screen, refer to my code, [Link](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Src/LCD.c)
- Modify also the initialise_monitor_handles function by adding the following code in the syscalls.c, refer to [Link](http://www.openstm32.org/forumthread1055)

## Measure Temperature
- Get the ADC value using the HAL library

Calculate temperature from adc value
1/T = 1/T0 + 1/B * ln(R/Rntc)

The variable T is the ambient temperature in Kelvin, T0 is the room temperature, in Kelvin (25°C = 298.15K), B is the beta constant, R is the thermistor resistance at the ambient temperature, and Rntc is the thermistor resistance at temperature T0. 

If the ADC reference voltage (Vref) and voltage divider source voltage (Vs) are the same then the following is true: 

adcMax / adcVal = Vref / Vs

R = Rntc * ( ( adcMax / adcVal ) - 1 )

then:

1/T = 1/T0 + 1/B * ln( Rntc * ( ( adcMax / adcVal ) - 1 ) / Rntc )

R0 cancels out, which leaves:

1/T = 1/T0 + 1/B * ln( ( adcMax / adcVal ) – 1 )

where:-
- adcMax is the adc resolution
- B is the constant depends of what type of temperature sensor used
- adcVal is the digital value after conversion from analog value(using HAL to convert)

After calculated the T in Kelvin, temperature in Celcius can be found out by:

C = T - 273.15

## Measure Light Intensity
- Get the ADC value using the HAL library
- Photodiode area 5.22mm² 
- irradiance : 1u/5.22mm² = 0.19W/m² (constant)
- Because the microcontroller cannot receive more than 3.3V, so the voltage must be step down from any voltage above 3.3V.
- The volatge can be adjusted by using a potentiometer.
- The voltage supply to light ntensity sensor can be any volatge between than 36V - 2.5V, bacause the light intensity can work between 2.5V - 36V.
- To find the intensity, get the actual volatge from the formula given in below and times with the irradiance constant.

voltage = ((float )ADC_Light / 4096) * stepDownVolatge

actual_volatge = (float)voltage * (Voltage/stepDownVolatge)

intensity = (float)actual_volatge * IRRADIANCE_CONST

where:-

- stepDownVoltage is the voltage connected to microncontroller(the analog pin) after step down from a higher voltage
- Voltage is the volatge connected to the light intensity sensor(2.5V - 36V)
- IRRADIANCE_CONST is 0.19 as calculated as above

## References
1. http://www.ti.com/lit/ds/symlink/opt101.pdf
2. https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
3. https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
4. https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf/jcr:content/translations/en.CD00171190.pdf
5. https://www.makeralot.com/download/Reprap-Hotend-Thermistor-NTC-3950-100K.pdf
6. https://electronicsforu.com/resources/learn-electronics/16x2-lcd-pinout-diagram
