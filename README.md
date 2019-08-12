![Retroleds](doc/16x2display.jpg?raw=true "Picture of HDSP-21xx 5x7 LED display showing ascii, extended ascii, Japanese Katakana and Cyrillic characters.")

This is a 16x2 serial character display using the following HP/Avago/Broadcom LEDs:

HDSP-2107 HDSP-2112 HDSP-2110 HDSP-2111 HDSP-2113
          HDSP-2122           HDSP-2121 HDSP-2123 

Inform me if other LED displays work too.

The display consists of a pcb board and an arduino sketch.

Any character received on the serial port is shown on the LED display.
The display provides a very complete ASCII character set, including accented characters, Japanese Katakana and Cyrillic. Other 5x7 bitmap fonts can be added if needed.

Optionally, a PS/2 keyboard can be connected. Any key typed on the keyboard is sent to the serial port. Together, display and PS/2 keyboard form a rudimentary serial terminal.

To use the display, a USB to serial adapter and a 5V power supply are needed. Plug a USB to 5V TTL serial cable into the serial port header. The serial port runs at 38400 8N1, no hardware flow control. Plug in a 5V DC 2A power supply, center pin positive, into the barrel jack connector.  Carefully check connector polarity before plugging in.

Escape codes provide simple display control.

Escape Code|Action
-----------|------
Esc[2J|		     Clear screen
Esc[5m|              Turn blinking mode on
Esc[0m|              Turn blinking mode off
Esc[30m ... Esc[37m| Set display intensity 0 ... 7. 0 = blanked display, 7 = maximum brightness. Default: 3.
Esc[1G ... Esc[33G|  Set cursor position. 1 = top left. 33 = bottom right
Esc[10m|             Select ascii font (default)
Esc[11m|             Select Katakana font
Esc[12m|             Select Cyrillic font
Esc[2;|		     Display test

A blinking question mark '?' indicates an error condition, usually an unrecognized escape code.

