# AVR MFSK (Multiple Frequency Shift Keying) modem transceiver

Original project based on ATmega88/168

Goertzel FFT based tone decoding for noisy audio links

Made specifically for radio use

## Features

 - Serial configurable: base frequency, frequency spacing and rate (~14 bits per second)

 - Repeated transmission of last data sent for performance testing

 - Audio input clipping indication

 - Automatic PTT control output

 - Automatic valid data checking

 - 8-bit sinewave output for regular audio channels

 - Suitable for keyboard-to-keyboard terminal operation

## Operation

There are two modem modes: normal operation / configuration. They are toggled by pressing the button

Configuring the parameters and enabling repeated transmission is done by going into configuration mode and sending a number corresponding to a byte that each of the it's double bits is a parameter value.

| param name | bit position | 0   | 1   | 2    | 3    |
|------------|--------------|-----|-----|------|------|
| rtrans     | 0            | off | on  | -    | -    |
| base freq  | 2            | 375 | 750 | 1125 | 1500 |
| freq space | 4            | 50  | 100 | 150  | 200  |
| rate*      | 6            | 15  | 25  | 35   | 45   |

*the smaller the value the higher the bit rate

Examples:
```
88 > (01 01 10 00) - rtrans off, bf 2, fs 1, r 1
89 > (01 01 10 01) - rtrans on,  bf 2, fs 1, r 1

152 > (10 01 10 00) - rtrans off, bf 2, fs 1, r 2
```

While in configuration mode the LED shows audio input clipping. To set proper input level, a tone from a modem should be played into the input and the level adjusted to where the LED stops flickering. A solid light means too much signal is being fed.

Transmitting data/message is done by sending it by serial in normal operation mode.
Receiving data/message is only done in normal operation mode and retrieved from serial.

LED stays constantly on while: catching a preamble, receiving data/message and transmitting data/message.

<img src="images/spectrum.png" width="800">

_spectral view of the transmission_

## Hardware

The PCB design is made for a 28-pin ATmega (QFN/MLF), whether that's 88, 168, 328, is a personal choice. Single-sided with some jumper wires required, vias are only for ground connection to the bottom layer.

Using an ATmega328 is encouraged, since originally an ATmega88 is used, flash usage is at ~99%, that limits adding more features.

There are pads left on the PCB for ISP programming and future use.

This project uses [MiniCore](https://github.com/MCUdude/MiniCore). Also a 16Mhz crystal is used.

Files for a 3D printed case are included. Case is held together by running a soldering iron along the seam of the top panel and main body.

<img src="images/board.png" width="800">

<img src="images/iso numbers blue.png" width="800">

<img src="images/side numbers blue.png" width="800">

<img src="images/modem in action lr.jpg" width="800">

_modem powered on with a battery and connected with a kenwood style cable_

<img src="images/modem inside lr.jpg" width="800">

_bare modem_

<img src="images/bt modem inside lr.jpg" width="800">

_modem with a bluetooth module added_

<img src="images/pcbs lr.jpg" width="800">

_toner method etched pcbs_

<img src="images/modems lr.jpg" width="800">

## Programming

 - Download and install [MiniCore](https://github.com/MCUdude/MiniCore)
 
 - Open code in IDE of choice
 
 - Configure board settings
 
   - Select your MCU and external 16Mhz clock
   
   - Leave the rest of the board settings by default
   
   - If want, you can enable bootloader and leave out a reset pin on the serial connector for future configuring of the key macros.
   
 - Flash via ISP programmer and it's done.

## Things to improve

 - Sometimes data doesn't sync
 - Sample tones more than one time and average them for better results in noisy audio
 - Make a better pwm filtering circuit
