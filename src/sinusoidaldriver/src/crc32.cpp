#include <sinusoidaldriver/crc32.h>

uint32_t GenerateCrc(uint8_t *bb, uint32_t size)
{
    uint32_t Crc = 0;
    uint32_t Hold = 0;
    uint32_t index = 0;
    uint32_t i = 0;
    uint32_t len = size;

    Crc = (int)0xFFFFFFFF; // Initial state

    for (i = 0; i < len-3; i+=4)
    {
        Hold = bb[i+0] + (bb[i+1] << 8) + (bb[i+2] << 16) + (bb[i+3] << 24);
        Crc = Crc32Fast(Crc, Hold); // 4-bytes at a time
    }
    return Crc;

}

uint32_t Crc32(uint32_t Crc, uint32_t Data)
{
    int i;
    Crc = Crc ^ Data;
    for(i=0; i<32; i++)
        if (Crc & 0x80000000)
            Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
        else
            Crc = (Crc << 1);
    return(Crc);
 }
 
 uint32_t Crc32Fast(uint32_t Crc, uint32_t Data)
 {
    static const uint32_t CrcTable[16] = { // Nibble lookup table for 0x04C11DB7 polynomial
    0x00000000,0x04C11DB7,0x09823B6E,0x0D4326D9,0x130476DC,0x17C56B6B,0x1A864DB2,0x1E475005,
    0x2608EDB8,0x22C9F00F,0x2F8AD6D6,0x2B4BCB61,0x350C9B64,0x31CD86D3,0x3C8EA00A,0x384FBDBD };
 
    Crc = Crc ^ Data; // Apply all 32-bits
 
    // Process 32-bits, 4 at a time, or 8 rounds
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28]; // Assumes 32-bit reg, masking index to 4-bits
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28]; // 0x04C11DB7 Polynomial used in STM32
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    Crc = (Crc << 4) ^ CrcTable[Crc >> 28];
    return(Crc);
 }