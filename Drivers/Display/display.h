#ifndef DISPLAYH
#define DISPLAYH
#include <stdint.h>
/*
The 4-line/8-bit serial bus interface of ILI9341 can be used by setting external pin(R35 - R38) as IM [3:0] to “0110” for serial
interface I or IM [3:0] to “1110” for serial interface II.

Current interface I is “0110” 4-line/8-bit serial bus interface of ILI9341

SCL  line --> PF7  (clock SCK SPI5)

D/CX line --> PD13 (comannd or data select)  D/CX bit is “low”, the
transmission byte is interpreted as a command byte. If the D/CX bit is “high”, the transmission byte is stored as
the display data RAM (Memory write command), or command register as parameter

CSX  line --> PC2  (chip select)   The serial interface is
initialized when CSX is high status. In this state, SCL clock pulse and SDA data are no effect. A falling edge on
CSX enables the serial interface and indicates the start of data transmission.

SDA  line --> PF9  (bidirectional in/out data MOSI SPI5)
*/

uint32_t ReadDisplayStatus(void);
uint32_t ReadDisplayId(void);
uint8_t ReadDisplayPowerMode(void);
uint16_t ReadDisplayPixelFormat(void);
void ResetDisplay(void);
void DisplayInversionON(void);
void DisplayON(void);
#endif // !DISPLAYH