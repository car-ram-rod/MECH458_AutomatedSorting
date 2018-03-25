#include "SSD1306.h"
#include "I2C.h"


void SSD1306Init(void)    /* Function definition */
{
	
 // Turn display off
    sendCommand(SSD1306_DISPLAYOFF);

    sendCommand(SSD1306_SETDISPLAYCLOCKDIV);
    sendCommand(0x80);

    sendCommand(SSD1306_SETMULTIPLEX);
    sendCommand(0x3F);
    
    sendCommand(SSD1306_SETDISPLAYOFFSET);
    sendCommand(0x00);
    
    sendCommand(SSD1306_SETSTARTLINE | 0x00);
    
    // We use internal charge pump
    sendCommand(SSD1306_CHARGEPUMP);
    sendCommand(0x14);
    
    // Horizontal memory mode
    sendCommand(SSD1306_MEMORYMODE);
    sendCommand(0x00);
    
    sendCommand(SSD1306_SEGREMAP | 0x1);

    sendCommand(SSD1306_COMSCANDEC);

    sendCommand(SSD1306_SETCOMPINS);
    sendCommand(0x12);

    // Max contrast
    sendCommand(SSD1306_SETCONTRAST);
    sendCommand(0xCF);

    sendCommand(SSD1306_SETPRECHARGE);
    sendCommand(0xF1);

    sendCommand(SSD1306_SETVCOMDETECT);
    sendCommand(0x40);

    sendCommand(SSD1306_DISPLAYALLON_RESUME);

    // Non-inverted display
    sendCommand(SSD1306_NORMALDISPLAY);

    // Turn display back on
    sendCommand(SSD1306_DISPLAYON);
}

void sendCommand(uint8_t command)
{
	i2cStart(SSD1306_DEFAULT_ADDRESS);
    i2cWrite(0x00);
    i2cWrite(command);
    i2cStop();
}

void invert(uint8_t inverted)
{
	    if (inverted) {
        sendCommand(SSD1306_INVERTDISPLAY);
    } else {
        sendCommand(SSD1306_NORMALDISPLAY);
    }
}


void sendFramebuffer(uint8_t *buffer) 
{
    sendCommand(SSD1306_COLUMNADDR);
    sendCommand(0x00);
    sendCommand(0x7F);

    sendCommand(SSD1306_PAGEADDR);
    sendCommand(0x00);
    sendCommand(0x07);

    // We have to send the buffer as 16 bytes packets
    // Our buffer is 1024 bytes long, 1024/16 = 64
    // We have to send 64 packets
    for (uint8_t packet = 0; packet < 64; packet++) { //*** change to 32? was 64
        i2cStart(SSD1306_DEFAULT_ADDRESS);
        i2cWrite(0x40);
        for (uint8_t packet_byte = 0; packet_byte < 16; ++packet_byte) {
            i2cWrite(buffer[packet*16+packet_byte]);
        }
        i2cStop();
    }
}
