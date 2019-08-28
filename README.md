# Temperature and Light Intensity Sensor

## Description
This project is about displaying the value of sensor in LCD screen. The value of sensor was get from the GPIO ADC pin of the microcontroller, through ADC(Analog to Digital) the value from sensor in analog form was change to digital form using the microcontroller. In this project, 2 sensor was used which is temperature sensor and light intensity sensor. The tempearature sensor can sense the current temperature while the light intensity sensor will sense the current light intensity value and display on the LCD screen. On the LCD screen, some custom symbol aslo designed to display on LCD screen.

## Tools
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/tool.png)
- STM32F103 Blue Pill Microcontroller, this microcontroller was used as ST link programmer to program the target microcontroller. [Link](https://jeelabs.org/img/2016/DSC_5474.jpg)
- STM32F103 Smart V2 Microcontroller, this microcontroller was used as target that connect every connection with the sensor and LCD. [Link](https://stm32-base.org/assets/img/boards/STM32F103C8T6_STM32_Smart_V2.0-1.jpg)
- 16 x 2 LCD Screen, used to display value of sensor or anything that can display on LCD screen. [Link](https://5.imimg.com/data5/MU/MN/MY-25117786/16x2-lcd-display-green-500x500.jpg)
- Light Intensity Sensor (OPT 101), this sensor was used to read the light intensity value. [Link](https://www.aam.com.pk/wp-content/uploads/2018/03/opt101.jpg)
- Temperature Sensor (NTC), this sensor was used to read the temperature value. [Link](https://ae01.alicdn.com/kf/HTB13TzPSFXXXXaWXFXXq6xXFXXXq/100K-Ohm-NTC-3950-Thermistors-with-Cable-for-3D-Printer-Reprap-Mend.jpg_640x640.jpg)
- Level Shifter, this level shifter was used to shift the voltage level betwee the microcontroller and the LCD screen. [Link](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/level%20shifter.jpg)

**Software used :**
- STM32 CubeMX
- STM32 System Workbench

## Connections
Connection of target(Smart V2 MCU) and program(Blue Pill MCU) 
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/schem.JPG)

Connection of Temperature sensor and microcontroller(Smart V2 MCU)
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/temp%20schem.png)

Connection of Light Intensity sensor and microcontroller(Smart V2 MCU)
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/light%20intensity%20schem.png)

Connection of LCD and microcontroller(Smart V2 MCU)
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/LCD_Diagram.PNG)

|No. Pin | Function | Name |
|:----------: |:-----------:|:-----------:|
| 1 | Ground (0V) | Ground |
| 2 | Supply voltage; 5V (4.7V – 5.3V) | Vcc |
| 3 | Contrast adjustment | Vo/VEE |
| 4 | Selects command register when low, and data register when high | RS (Register Select) |
| 5 | Low to write to the register; High to read from the register | Read/Write |
| 6 | Sends data to data pins when a high to low pulse is given | ENABLE |
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
- The data pin was connected to the microcontroller to send data from microcontroller to LCD. 
- Data can transfer in 4-bit mode or 8-bit mode, if 4-bit mode was choosen D0 - D3 was not in used, while in 8-bit mode all the pin D0 - D7 are used.
- In pin 3, to adjust the contrast of LCD, a potentiameter was used. The output of the poteantiameter was connected t this pin. To make the LCD display more clear, can tune the potentiameter to more negative side(Ground).

## Setup in CubeMX
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/cubemx_config.JPG)
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/GPIO%20pin.JPG)
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/analog%20pin.png)
- To configure an Analog pin, can choose any pin that have ADC configuration that in the microcontroller, 2 analog pin was configured in this project (1 for temperature sensor, 1 for light intensity sensor).
- Configure 8 output pin for data, this is to output the data from microcontroller to LCD. The output pin can choose any pin that in the microcontroller. (pin that are in same GPIO type are suggested to choose --> this is because during the the configuration of setting the output of the pin is more easier).  e.g. if GPIOA was used, then all pin also use the same GPIO type which is GPIOA.
- Configure 3 output pin for register select, read/write and enable, this three pin was use by the LCD. 
- Configure the pin of data to OUTPUT OPEN DRAIN mode, others as OUTPUT PUSH PULL mode

## Setup in LCD
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/level%20shifter%20connection.png)
- Before connect all the pin to LCD from microcontroller, a level shifter needed to connected before connect to LCD from microcontroller.
- The lower level side of the level shifter connected to the microcontroller and 3.3V, the higher level side of the level shifter connected to the LCD and 5V. Both side also need to be connect to ground.
- After finish construct all the pin, LCD needed to initalise before sending anything to LCD. Refer to --> [Link](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Src/LCD.c), function lcdInit().
- To display on LCD, can send LCD command to let the LCD know what to do, LCD command can refer to [Link](https://electronicsforu.com/resources/learn-electronics/16x2-lcd-pinout-diagram)
- For displaying string or number in float, can configure in the eclipse with following step :
![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/linker%20guide.png)
--> Project >> Properties >> C/C++ Builder >> Setting >> MCU GCC Linker >> Miscellaneous >> Linker Flags >> add **-specs=nosys.specs -specs=nano.specs -u _printf_float**

- **-u _printf_float** was used for printing the float number.
- Modify the __io_putchar function by calling the lcdWriteMsg function to display on LCD screen, refer to the code. [Link](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Src/LCD.c)
- Modify also the initialise_monitor_handles function by adding the following code in the syscalls.c, refer to [Link](http://www.openstm32.org/forumthread1055)

## Measure Temperature
- Get the ADC value using the HAL library

Calculate temperature from adc value
1/T = 1/T0 + 1/B * ln(R/Rntc)

The variable T is the ambient temperature in Kelvin, T0 is the room temperature, in Kelvin (25°C = 298.15K), B is the beta constant, R is the thermistor resistance at the ambient temperature, and Rntc is the thermistor resistance at temperature T0. 

If the ADC reference voltage (Vref) and voltage divider source voltage (Vs) are the same then the following is true: 

![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/temp%20formula.png)

where:-
- adcMax is the adc resolution
- B is the constant depends of what type of temperature sensor used
- adcVal is the digital value after conversion from analog value(using HAL to convert)

After calculated the T in Kelvin, temperature in Celcius can be found out by:

C = T - 273.15

In this project, the B constant does not follow the B constant given by the NTC(temperature sensor) follow by the manual which is 3950. [Link](https://www.makeralot.com/download/Reprap-Hotend-Thermistor-NTC-3950-100K.pdf) 

The B constant value was taken experimentally.
In the experiment, all the resistance was recorded according with the temperature from 30°C to 110°C.
The experiment was carried out by using a temperature oven to record the temperature and resistance.
The value of Ravr(theory) was taken from the given table with B constant value of 3950. The value of R(prac) was measured practical with 100kohm of resistor.
The resistance value was transform to linear format before finding a suitable B constant value. To transform from non-linear to linear, the y-axis element was plot as **ln y** which is the resistance value. 
Below shows the formula how to transform from non-linear graph to linear graph. The linear graph was plot using this formula.

![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/graph%20formula.png)

Below show the graph before and after linearize, and the new B constant was found also which is 3631. Excel file, [Link](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/r%20vs%20t.xlsx)

![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/linearize%20graph.png)

## Measure Light Intensity
- Get the ADC value using the HAL library
- Photodiode area 5.22mm² 
- irradiance :

![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/constant%20value.png)

- Because the microcontroller cannot receive more than 3.3V, so the voltage must be step down from any voltage above 3.3V.
- The volatge can be adjusted by using a potentiometer.
- The voltage supply to light ntensity sensor can be any volatge between than 36V - 2.5V, bacause the light intensity can work between 2.5V - 36V.
- To find the intensity, get the actual volatge from the formula given in below and times with the irradiance constant.

![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/light%20formula.png)

where:-

- stepDownVoltage is the voltage connected to microncontroller(the analog pin) after step down from a higher voltage
- Voltage is the volatge connected to the light intensity sensor(2.5V - 36V)
- IRRADIANCE_CONST is 0.19 as calculated as above

##Custom Symbol
The LCD only have 8 locations 0-7 for custom chars in the CGRAM.
User can custom any symbol for 5x8 dots or 5x10 dots font size.
For 5x8 dots font size, user can fully use the 2 line in LCD to put the created custom symbol.
For 5x10 dots font size, user can only use 1 line to display the created custom symbol.


![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/custom%20symbol.png)

## Result
From the code, this is the result will show.

![alt text](https://github.com/ZHISHANN/TemperatureAndLightIntensitySensor/blob/master/Image/result.PNG)

## References
1. OPT101 Monolithic Photodiode and Single-Supply Transimpedance Amplifier manual, [Link](http://www.ti.com/lit/ds/symlink/opt101.pdf)
2. HD44780U (LCD-II) manual, [Link](https://www.sparkfun.com/datasheets/LCD/HD44780.pdf)
3. RM0008 Reference manual, [Link](https://www.st.com/content/ccc/resource/technical/document/reference_manual/59/b9/ba/7f/11/af/43/d5/CD00171190.pdf/files/CD00171190.pdf)
4. Reprap Hotend Thermistor NTC 3950 100K with 1M Cable manual, [Link](https://www.makeralot.com/download/Reprap-Hotend-Thermistor-NTC-3950-100K.pdf)
5. How 16×2 LCDs work, [Link](https://electronicsforu.com/resources/learn-electronics/16x2-lcd-pinout-diagram)
6. Linear regression graph, [Link](https://www.ablebits.com/office-addins-blog/2018/08/01/linear-regression-analysis-excel/)
