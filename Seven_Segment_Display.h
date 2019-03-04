#pragma once
enum {
	display_REG_DECODE = 0x09,
	display_REG_INTENSITY = 0x0A,
	display_REG_SCANLIMIT = 0x0B,
	display_REG_SHUTDOWN = 0x0C,
	display_REG_DISPTEST = 0x0F,
};
enum { OFF = 0, ON = 1 };
bool display_flag = false;
const byte DP = 0b10000000;
const byte t = 0b00001111;
const byte h = 0b00010111;
const byte P = 0b01100111;
const byte dash = 0b00000001;
const byte E = 0b01001111;
const byte R = 0b01110111;
const byte S = 0b01011011;
const byte O = 0b01111110;
const byte space = 0b00000000;
void set_register(byte reg, byte value) {   // ... write a value into a max729 register See MAX729 Datasheet, Table , page 6
	digitalWrite(display_CS, LOW);
	shiftOut(display_DIN, display_CLK, MSBFIRST, reg);
	shiftOut(display_DIN, display_CLK, MSBFIRST, value);
	digitalWrite(display_CS, HIGH);
}
void resetDisplay() {
	set_register(display_REG_SHUTDOWN, OFF);   // turn off display
	set_register(display_REG_DISPTEST, OFF);   // turn off test mode
	set_register(display_REG_INTENSITY, 0x0D); // display intensity
}
void HUB_display(String thisString) { // ... display on the 7-segment display
	set_register(display_REG_SHUTDOWN, OFF);  // turn off display
	set_register(display_REG_SCANLIMIT, 7);   // scan limit 8 digits
	set_register(display_REG_DECODE, 0b11011011);		// 
	if ((thisString.charAt(0) < 0x30) || (thisString.charAt(0) > 0x39)) thisString.setCharAt(0, 0x30);
	if ((thisString.charAt(1) < 0x30) || (thisString.charAt(1) > 0x39)) thisString.setCharAt(1, 0x30);
	//	if ((thisString.charAt(2) < 0x30) || (thisString.charAt(2) > 0x39)) thisString.setCharAt(2, 0x30);
	if ((thisString.charAt(3) < 0x30) || (thisString.charAt(3) > 0x39)) thisString.setCharAt(3, 0x30);
	if ((thisString.charAt(4) < 0x30) || (thisString.charAt(4) > 0x39)) thisString.setCharAt(4, 0x30);
	//	if ((thisString.charAt(5) < 0x30) || (thisString.charAt(5) > 0x39)) thisString.setCharAt(5, 0x30);
	if ((thisString.charAt(6) < 0x30) || (thisString.charAt(6) > 0x39)) thisString.setCharAt(6, 0x30);
	if ((thisString.charAt(7) < 0x30) || (thisString.charAt(7) > 0x39)) thisString.setCharAt(7, 0x30);
	set_register(1, thisString.charAt(7));
	set_register(2, thisString.charAt(6));
	set_register(3, dash);	// 6
	set_register(4, thisString.charAt(4));	// 5
	set_register(5, thisString.charAt(3));  // 4
	set_register(6, dash);	// 3
	set_register(7, thisString.charAt(1));  // 2
	set_register(8, thisString.charAt(0));  // 1
	set_register(display_REG_SHUTDOWN, ON);   // Turn on display
}
void HUB_error_display(String thisString) { // ... display on the 7-segment display, thisString is the error number
	set_register(display_REG_SHUTDOWN, OFF);  // turn off display
	set_register(display_REG_SCANLIMIT, 7);   // scan limit 8 digits
	set_register(display_REG_DECODE, 0b000000);		// 
	if ((thisString.charAt(0) < 0x30) || (thisString.charAt(0) > 0x39)) thisString.setCharAt(0, 0x30);
	if ((thisString.charAt(1) < 0x30) || (thisString.charAt(1) > 0x39)) thisString.setCharAt(1, 0x30);
	set_register(1, thisString.charAt(1));
	set_register(2, thisString.charAt(0));
	set_register(3, dash);
	set_register(4, R);		// R  
	set_register(5, O);     // O
	set_register(6, R);     // R
	set_register(7, R);		// R
	set_register(8, E);     // E
	set_register(display_REG_SHUTDOWN, ON);   // Turn on display
}
