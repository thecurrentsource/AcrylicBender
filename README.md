# AcrylicBender
Code used for the filament timer on video #31 Acrylic Bending
Can be found here: https://youtu.be/VjEyUUQP1s8

I didn't create a schematic for this project as it is simply an HD44780 style LCD attached to port B & D of a PIC18F4550 microcontroller. The pinout can be read at the top of main.c. There's a 10k pot between Vcc & GND and the wiper goes to AN0 - this selects how long the timer will run, and it's not even close to accurate. I flubbed these numbers as the interrupt service routine is so busy making jumps between subroutines, it wastes a crapload of time itself. In the end I found that it was just easier to control the nichrome heater with the pot in the zero position (infinitely on-off) and using the switch to control it - which is attached to RA1.
Anyway, you are a savvy person and can figure out the rest yourself. Before you go batshit on the quality of the code, the hardware was done in a day, and the fimrware done in about 4 hours, then video released. Hands-off after that... so if you want to improve on it, go to town. I'll buy you a beer.

Cheers.
Derek
