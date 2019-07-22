
/*

   #           #                           #####       #                          
   #           #                          #     #     ##                          
   ####    #####   ####   #####                 #      #  #    #  #    #          
   #   #  #    #  #        #   #   #####    ####       #   #  #    #  #           
   #   #  #    #   ####    #   #           #           #    ##      ##            
   #   #  #    #       #   ####           #            #   #  #    #  #           
  ##   #   #### # #####    #              #######      #  #    #  #    #          
                          ##                                                      

  16x2 character display

*/


/*
  Copyright (c) 2019 Koen De Vleeschauwer.  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include <Arduino.h>
#include <TimerOne.h>
#include <Wire.h>
#include <Adafruit_MCP23017.h>
#include <PS2Keyboard.h>

#include "glcdfont.h"

/* pins on atmega328p */

#define LED_CE0  16
#define LED_CE1  11
#define LED_CE2  17
#define LED_CE3  12
#define LED_CLK  9
#define LED_FL   14
#define LED_RD   13
#define LED_RST  15
#define LED_WR   10
#define PS2_CLK  3
#define PS2_DAT  2

Adafruit_MCP23017 mcp;
PS2Keyboard keyboard;

#define DISP_SIZE 32
#define MAX_DISPLAYS 4
#define ALL_DISPLAYS 255

#define MAX_FONTS 2
#define DEFAULT_INTENSITY 4
#define BAUD_RATE 19200

// hdsp-21xx chip enable pins, active low
static uint8_t hdsp_chip_enable[4] = {LED_CE0, LED_CE1, LED_CE2, LED_CE3};

// Display buffer
struct display_char
{
  uint8_t ch;
  uint8_t font_id;
  bool blinking;
};

struct display_char hdsp_display[DISP_SIZE];

// 16-bit port extender
void mcp_init() {
  mcp.begin();
  for (uint8_t i; i < 16; i++) mcp.pinMode(i, OUTPUT);
  mcp.writeGPIOAB(0);
}

// current terminal mode
static bool term_blinking = false;
static uint8_t term_font_id = 0;
static uint8_t term_cursor = 0;

// terminal state machine for parsing escape codes
typedef enum {STATE_START, STATE_ESC, STATE_BRACKET, STATE_DIGIT0, STATE_DIGIT1} term_state_type;
term_state_type term_state = STATE_START;
uint8_t esc_digit0 = 0, esc_digit1 = 0;

// low-level routines

/* set hdsp-21xx address and data bus values on bus extender */
inline void hdsp_set_addr_dta(uint8_t addr, uint8_t dta)
{
  uint16_t ba;
  ba = addr;
  ba = ba << 8;
  ba = ba | dta;
  mcp.writeGPIOAB(ba);
  return;
}

// configure hdsp clock. all displays receive the same clock, so characters blink in sync
void hdsp_init_clk()
{
  // configure TIMER 1 for frequency of 57348 Hz.
  Timer1.initialize(17);  // set PWM period to 17 us = 58.8 kHz

  // set LED_CLK output pin to 50% duty cycle
  Timer1.pwm(LED_CLK, 511);
  
  return;
}

// display hard reset
void hdsp_reset() {
    /* Pull HDSP-21xx reset pin low */
  digitalWrite(LED_RST, LOW);
  delay(1); /* Needs three clock cycles for hardware reset */
  digitalWrite(LED_RST, HIGH);
  delay(1);
}

// write one byte on hdsp bus
// disp: display number, 0..3
// addr: address
// dta: data
// flash: if low, access "flash" memory to store blinking attribute.
void hdsp_write_cycle(uint8_t disp, uint8_t addr, uint8_t dta, uint8_t flash)
{

#ifdef DEBUG
  Serial.print("hdsp_write_cycle disp ");
  Serial.print(disp);
  Serial.print(" addr ");
  Serial.print(addr, BIN);
  Serial.print(" dta ");
  Serial.println(dta, BIN);
  Serial.print(" flash ");
  Serial.print(flash);
#endif

  digitalWrite(LED_FL, flash);
  hdsp_set_addr_dta(addr, dta);
  delayMicroseconds(1);
  if (disp != ALL_DISPLAYS)
    digitalWrite(hdsp_chip_enable[disp], LOW);
  else {
    /* pull all chip enables low for parallel write */
    digitalWrite(LED_CE0, LOW);
    digitalWrite(LED_CE1, LOW);
    digitalWrite(LED_CE2, LOW);
    digitalWrite(LED_CE3, LOW);    
  }
  delayMicroseconds(1);
  digitalWrite(LED_WR, LOW);
  delayMicroseconds(1);
  digitalWrite(LED_WR, HIGH);
  delayMicroseconds(1);
  if (disp != ALL_DISPLAYS)
    digitalWrite(hdsp_chip_enable[disp], HIGH);
  else {
    /* restore all chip enables */
    digitalWrite(LED_CE0, HIGH);
    digitalWrite(LED_CE1, HIGH);
    digitalWrite(LED_CE2, HIGH);
    digitalWrite(LED_CE3, HIGH);    
  }
  digitalWrite(LED_FL, HIGH);
  delayMicroseconds(1);
}

/* blink character at position pos */
void hdsp_blink_char(uint8_t pos, bool blink) {
  uint8_t disp = pos >> 3;
  uint8_t col = pos & 0x7;

  uint8_t addr = pos;
  uint8_t dta =  blink ? 0x1 : 0x0;

  hdsp_write_cycle(disp, addr, dta, LOW); // This is the only hdsp_write_cycle where flash is low.

  return;
}

// write a character of the built-in character set on postion pos, pos = 0..31.
// the character is ascii or katakana, depending on the built-in character set of the hdsp-21xx.

void hdsp_write_builtin_char(uint8_t pos, uint8_t ch, bool blinking = false) {
  
#ifdef DEBUG
  Serial.print("builtin char pos ");
  Serial.print(pos);
  Serial.print(" char '");
  Serial.print(ch);
  Serial.println("'");
#endif
  
  uint8_t disp = pos >> 3;
  uint8_t column = pos & 0x7;
  
  uint8_t addr = pos | 0x18;
  uint8_t dta = ch & 0x7f;

  // set blinking ("flash") attribute
  hdsp_write_cycle(disp, addr, dta, HIGH);
  hdsp_blink_char(pos, blinking);

}

  /* 
   * Write using user-defined characters (udc) on position pos, pos = 0...31. font_id is font number, font_id = 0..2
   * Leftmost character on the hdsp-21xx is set to udc 0, rightmost character is set to udc 7.
   * When a character has to be written to the display, look up the character bitmap in the font table, and update the bitmap of the udc.
   * This provides a very complete ascii character set, including accented characters. 
   */

void hdsp_write_user_defined_char(uint8_t pos, uint8_t ch, bool blinking = false, uint8_t font_id = 0) {

#ifdef DEBUG
  Serial.print("udc char pos ");
  Serial.print(pos);
  Serial.print(" char '");
  Serial.print(ch);
  Serial.println("'");
#endif
  
  uint8_t disp = pos >> 3;
  uint8_t col = pos & 0x7;

  // check inputs
  if (ch > 255) ch = '?';
  if (font_id > MAX_FONTS) font_id = 0;
 
   // find offset in font table for character ch. 
  uint16_t char_idx = 0;
  if (ch < 128)        /* the first 128 characters are the same in all code pages */
    char_idx = ch * 5; /* glcdfont.h font arranged in 5 rows of 7 pixels per character */
  else 
    char_idx = font_id * 128 * 5 + ch * 5;

  // set position 'pos' of display to user-defined character 'pos'
  hdsp_write_cycle(disp, 0x18 | col, 0x80 | col, HIGH); /* Figure 2 in datasheet */  
  
  // write bitmap of character ch to udc ram. first set udc address register of character
  hdsp_write_cycle(disp, col, col, HIGH); /* Figure 3 in datasheet */
  
  // then lookup each row of pixels of the character
    
  for (uint8_t row = 0; row <= 6; row++)
  {
    uint8_t pixels = 0;
    for (uint8_t col = 0; col <= 4; col++)
    {
      uint16_t line_idx = font + char_idx + col;
      //uint8_t line = font[char_idx + col]; // if font table in ram
      uint8_t line = pgm_read_byte(line_idx); // if font table in progmem
      uint8_t pixel = (line >> row) & 0x1; 
      pixels |= pixel << (4 - col);
    }
    // write row of pixels to udc ram
    hdsp_write_cycle(disp, 0x08 | row, pixels, HIGH);
  }  

  // set blinking ("flash") attribute
  hdsp_blink_char(pos, blinking);
   
  return;
}

// set display intensity 0..7. Also enables character blinking.
void hdsp_intensity(uint8_t i)
{
  hdsp_write_cycle(ALL_DISPLAYS, 0x10, i & 0x7 | 0x8, HIGH); /* Figure 6 in datasheet */
}

// write internal character buffer to display
void hdsp_update() {
  for (uint8_t i = 0; i < DISP_SIZE; i++)
    hdsp_write_user_defined_char(i, hdsp_display[i].ch, hdsp_display[i].blinking, hdsp_display[i].font_id);
}

// erase internal character buffer and clear screen
void hdsp_clear_screen() {
  for (uint8_t i = 0; i < DISP_SIZE; i++) {
    hdsp_display[i].ch = ' ';
    hdsp_display[i].font_id = 0;
    hdsp_display[i].blinking = false;
  }
  hdsp_update();
  return;
}

// move characters up one line
void hdsp_scroll() {
  for (uint8_t i = 0; i < 16; i++) {
    hdsp_display[i] = hdsp_display[i+16];
    hdsp_display[i+16].ch = ' ';
    hdsp_display[i+16].font_id = 0;
    hdsp_display[i+16].blinking = false;
  }
  hdsp_update();
}

// display 'ready' 
void display_ready() {
  const uint8_t prompt_len = 5;
  char prompt[prompt_len] = "ready";

  for (uint8_t i=0; i<prompt_len; i++) hdsp_write_user_defined_char(i, prompt[i], false, 0);

  term_cursor = 31; // force scroll
  
  return;
}

void display_test() {
  for (uint8_t i=0; i<32; i++) hdsp_write_builtin_char(i, 'A'+i, false);  // depending upon display model, this shows ascii or katakana
  delay(2000);
  for (uint8_t i=0; i<32; i++) hdsp_write_builtin_char(i, 'a'+i, false);  // depending upon display model, this shows lower case ascii or upper case ascii
  delay(2000);
  for (uint8_t i=0; i<32; i++) hdsp_write_user_defined_char(i, 'a'+i, false); // lower case ascii
  delay(2000);
  for (uint8_t i=0; i<8; i++) hdsp_write_user_defined_char(i, 'a'+i, false, 0); // ascii
  for (uint8_t i=0; i<8; i++) hdsp_write_user_defined_char(i + 8, 0xc0+i, false, 0); // accented characters
  for (uint8_t i=0; i<8; i++) hdsp_write_user_defined_char(i + 16,0xc0+i, false, 1); // katakana
  for (uint8_t i=0; i<8; i++) hdsp_write_user_defined_char(i + 24,0xc0+i, false, 2); // cyrillic
  return; 
}

// run once at boot
void setup() {

  /* pins on atmega */ 
  pinMode(LED_CE0, OUTPUT);
  pinMode(LED_CE1, OUTPUT);
  pinMode(LED_CE2, OUTPUT);
  pinMode(LED_CE3, OUTPUT);
  pinMode(LED_CLK, OUTPUT);
  pinMode(LED_FL, OUTPUT);
  pinMode(LED_RD, OUTPUT);
  pinMode(LED_RST, OUTPUT);
  pinMode(LED_WR, OUTPUT);
  digitalWrite(LED_CE0, HIGH);
  digitalWrite(LED_CE1, HIGH);
  digitalWrite(LED_CE2, HIGH);
  digitalWrite(LED_CE3, HIGH);
  digitalWrite(LED_FL,  HIGH);
  digitalWrite(LED_RD,  HIGH);
  digitalWrite(LED_RST, HIGH);
  digitalWrite(LED_WR,  HIGH);
  
  // Serial port
  Serial.begin(BAUD_RATE);

  // The following line is magic needed for pgm_read_byte() to function.
  if (millis() > 100000L) Serial.println((uint16_t) font, HEX);

  // i2c port extender
  mcp_init();

  // display clock
  hdsp_init_clk();

  // clear display
  hdsp_reset();
  hdsp_clear_screen();
  hdsp_intensity(DEFAULT_INTENSITY);  // Set default intensity and allow character blinking

  // initialize ps/2 keyboard
  keyboard.begin(PS2_DAT, PS2_CLK);

  // ready!
  display_ready();

  //display_test();  // uncomment for display test at power-on

  return;
}


// read one character from the keyboard and send it to the serial port

void keyboard_loop()
{
  char ch = 0;

  if (keyboard.available() /* && Serial.availableForWrite() // not needed on atmega328 - Serial.Write never blocks */ )
  {
    ch = keyboard.read();
    if (ch > 0) Serial.write(ch & 0xff);
  }
  return;
}

/* read one character from the serial port and send it to the display */
void display_loop()
{
  int ch = 0;
  
  if (Serial.available() <= 0) return;

  ch = Serial.read();
  
  /* Escape codes */
  
  /*      
   Esc[2J              Clear screen
   Esc[5m              Turn blinking mode on
   Esc[0m              Turn blinking mode off
   Esc[30m ... Esc[37m Set display intensity 0 ... 7. 0 = maximum brightness, 7 = blanked display
   Esc[1G ... Esc[33G  Set cursor position. 1 = top left. 33 = bottom right
   Esc[10m             Select default ascii font
   Esc[11m             Select Katakana font
   Esc[12m             Select Cyrillic font
   */
  
  switch (term_state) {
    case STATE_START:
      if (ch == '\e') 
      { 
        term_state = STATE_ESC; 
        return;
      }
      break;

    case STATE_ESC:
      if (ch == '[') 
      { 
        term_state = STATE_BRACKET; 
        return; 
      }
      term_state = STATE_START;
      break;

    case STATE_BRACKET:
      if ((ch >= '0') && (ch  <= '9')) 
      { 
        esc_digit0 = ch - '0'; 
        term_state = STATE_DIGIT0; 
        return; 
      }
      term_state = STATE_START;
      break;

    case STATE_DIGIT0:
      if ((ch == 'J') && (esc_digit0 == 2)) 
      {
        /* Esc[2J - clear screen */
        hdsp_clear_screen(); 
        term_state = STATE_START; 
        return;
      } 
      if ((ch == 'm') && (esc_digit0 == 5)) 
      {
        /* Esc[5m - blinking on */
        term_blinking = true; 
        term_state = STATE_START; 
        return;
      } 
      if ((ch == 'm') && (esc_digit0 == 0)) 
      {
        /* Esc[0m - blinking off */
        term_blinking = false; 
        term_state = STATE_START; 
        return;
      } 
      if ((ch == 'G') && (esc_digit0 > 0)) 
      {
        /* Esc[1G .. Esc[9G - set cursor position, one digit. */
        term_cursor = esc_digit0 - 1;
        term_state = STATE_START;
        return;
      } 
      if ((ch >= '0') && (ch  <= '9')) 
      { 
        esc_digit1 = ch - '0'; 
        term_state = STATE_DIGIT1; 
        return; 
      }
      term_state = STATE_START;
      break;

    case STATE_DIGIT1:
      if ((ch == 'm') && (esc_digit0 == 3)) 
      { 
        /* Esc[30m .. Esc[37m - set display intensity */
        if (esc_digit1 <= 7) hdsp_intensity(esc_digit1); 
        term_state = STATE_START; 
        return;
      } 
      if ((ch == 'm') && (esc_digit0 == 1)) 
      { 
        /* 
         * Esc[10m Select default ascii font
         * Esc[11m Select Katakana font
         * Esc[12m Select Cyrillic font
         */
        if (esc_digit1 <= MAX_FONTS) term_font_id = esc_digit1;   
        term_state = STATE_START; 
        return;
      } 
      if (ch == 'G') 
      { 
        /* Esc[10G .. Esc[33G - set cursor position, two digits. */ 
        int8_t new_cursor = esc_digit0 * 10 + esc_digit1 - 1; 
        if ((new_cursor >= 0) && (new_cursor <= 32)) term_cursor = new_cursor;
        term_state = STATE_START; 
        return;
      } 
      term_state = STATE_START;
      break;

    default:
      term_state = STATE_START;
      break;
  }
  
  /* carriage return new line */
  if (ch == '\r')
  {
    term_cursor &= 0xf8;
    return;
  }
  
  /* new line */
  if (ch == '\n')
  {
    if (term_cursor >= 16) hdsp_scroll();
    term_cursor = 16;
    return;
  }

  if (ch == '\b') {
    if (term_cursor == 0) return;
    --term_cursor;
    hdsp_write_user_defined_char(term_cursor, ' ', term_blinking, term_font_id);
    return;
  }
  
  /* scrolling */
  if (term_cursor >= DISP_SIZE) {
      hdsp_scroll(); /* scroll display one line */
      term_cursor = 16;
  }

  /* write incoming character at current cursor position */
  hdsp_write_user_defined_char(term_cursor, ch, term_blinking, term_font_id);

  // save in buffer
  hdsp_display[term_cursor].ch = ch;
  hdsp_display[term_cursor].blinking = term_blinking;
  hdsp_display[term_cursor].font_id = term_font_id;
  
  term_cursor++;

  return;
}
  
void loop() {
  keyboard_loop();
  display_loop();
  return;
}

//not truncated
