# newt_lcd
Convert Newton MP2x00 LCD data stream into TFT output

### [YouTube Video Explaining this Setup](https://youtu.be/Q1Ef7FQeF78?si=kHJoH5G_v21F1Ucw&t=712)

![experimental setup](assets/newt_lcd_setup.jpg)

The Newton MessagePad MP2x00 is one of the first PDAs ever, produced by Apple in 1994, and infamous for its bad handwriting recognition. The project was abandoned by Apple, but a bunch of these machine and a core fan base remain.

One of the big issue with original MessagPad is the LCD. It never had great contrast to begin with, but over the years, either the displays degraded even more, or the the eyes of the aging users degraded. It's probably both. 

I have been researching up and down the internet for replcaement displays, but the original ones are not available new, and very few other displays have the same form factor. And even if they do, they are usualy color displays and have a completel different connector and protocol. Luckily, we live in 2024, and CPUs have become so fast, that they can easily scan the MP's output to the display, rearrange bytes, and generate output in a different format to go to a replacement display. 

The RP2040 for example can manage video in and out almost completely using its DMA and programmable PIO, and it still has two full blown ARM CPUs to manage the image data. It can extract grayscale information, upscale pixels and adapt to color displays, all while keeping the original hardware going. It can even convert touch information back into an analog signal for the ADC converter in the Newton.

So here is a description of my setup above that has proven to work.

We have tow RP2040 Pico W, the second board is the Debug Probe for the first board. This needs 6 jumpers from RP1 to RP2 (debugger):

| Pin | Signal | Signal | Pin |
| --- | ------ | ------ | --- |
| 39 | VSYS | VSYS | 39 |
| 40 | GND | GND | 40 |
| X1 | SWCLK | GP2 | 4 |
| X3 | SWDIO | GP3 | 5 |
| 2 | RX0 | TX1 | 7 |
| 1 | TX0 | RX1 | 6 |

The second bunch of connectors go to the new TFT display. This part is quite flexible. We could connect an e-Papre display here. Waveshare has a nice 5.65" module that fits almost perfectly into the Newton case. On the left is the RP2040 Pico W, on the right are the pins of the Waveshare 4.0" 480x320 TFT:

| Pin | Signal | Signal | Pin |
| --- | ------ | ------ | --- |
| 18 | +3V | VCC | 1 |
| 36 | GND | GND | 2 |
| 22 | GP17 | CS | 3 |
| 19 | GP14 | RESET | 4 |
| 20 | GP15 | DC/RS | 5 |
| 25 | GP19 | SDI | 6 |
| 24 | GP18 | SCK | 7 |
| 18 | +3V | LED | 8 |
| 21 | GP16 | SDO | 9 |
|  | n.c. | TOUCH... | 10..14 |



