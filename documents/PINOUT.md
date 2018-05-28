# Supported Devices
[Back to index](README.md).

## Naze32 Rev5
The Naze32 Rev5 is the recommended device in terms of support and stability.

#### Compiling
Use the flag `NAZE32_REV5` compile flag while compiling and flashing.

#### Pinout
1. PWM motor outputs (motor numbers on back)
2. Auxilary outputs
   - GND (unlabeled)
   - 5V (Dot)
   1. PPM
   2. N/C
   3. UART2 Tx (N/C)
   4. UART2 Rx (N/C)
   5. Safety Button
   6. N/C
   7. N/C
   8. N/C
3. Battery Monitor (N/C)
4. Buzzer
5. Secondary Tx(?) output (N/C)
6. 3.3V output (100mA max)
7. Booloader Pins (short to enter bootloader during power on)
8. UART1 Tx/Rx

![Naze32 Rev5 Pinout](naz32_rev5_pinout.png)

## Naze32 Rev6
The Naze32 Rev6 features a nicer pinout profile, and may be easier to find in stock.

#### Compiling
Use the flag `NAZE32_REV6` compile flag while compiling and flashing.

#### Pinout
1. PWM motor outputs (motor numbers on back)
2. Auxilary outputs (labelled on back)
   - GND
   - 5V
   1. PPM
   2. N/C
   3. UART2 Tx (N/C)
   4. UART2 Rx (N/C)
   5. Safety Button
   6. N/C
   7. N/C
   8. N/C
3. Battery Monitor (N/C)
4. Buzzer
5. Secondary Tx(?) output (N/C)
6. 3.3V output (100mA max)
7. Booloader Pins (short to enter bootloader during power on)
8. UART1 Tx/Rx

![Naze32 Rev6 Pinout](naz32_rev6_pinout.png)

## Naze32 Mini Rev3
The Naze32 Mini Rev3 is a miniturized design of the Naze32 Rev5.

#### Compiling
Use the flag `NAZE32_REV5` compile flag while compiling and flashing.

#### Pinout
1. PWM motor outputs (motor numbers on back)
2. Auxilary outputs (split accross multiple headers)
   - GND
   - 5V
   - SIG -> PPM
   - N/C
   - UART2 Tx (N/C)
   - UART2 Rx (N/C)
   - CH5 -> Safety Button
   - CH6 -> N/C
   - CH7 -> N/C
   - CH8 -> N/C
3. Battery Monitor (N/C)
4. Buzzer
5. N/C
6. 3.3V output (100mA max)
7. Booloader Pins (short to enter bootloader during power on)
8. UART1 Tx/Rx

![Naze32 Mini Rev3 Pinout](naz32_mini_rev3_pinout.png)
