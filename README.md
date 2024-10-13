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

- 39 - VSYS - 39
- 40 - GND - 40
- SWCLK - GP2 - 4
- SWDIO - GP3 - 5
- 2 - RX0 - TX1 - 7
- 1 - TX0 - RX1 - 6

