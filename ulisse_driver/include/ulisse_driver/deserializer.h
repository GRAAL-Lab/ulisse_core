#ifndef DESERIALIZER_H
#define DESERIALIZER_H

#include <stdint.h>
#include <memory>
#include <vector>

typedef float float32_t;
typedef double float64_t;

class Deserializer
{
public:
    Deserializer(std::vector<uint8_t> buffer);

    void MoveOffset(uint8_t newOffset);

    uint16_t PacketExtract_char(char* value);

    uint16_t PacketExtract_int8(int8_t* value);


    uint16_t PacketExtract_int16(int16_t* value);


    uint16_t PacketExtract_int32(int32_t* value);


    uint16_t PacketExtract_int64(int64_t* value);

    uint16_t PacketExtract_uint8(uint8_t* value);


    uint16_t PacketExtract_uint16(uint16_t* value);


    uint16_t PacketExtract_uint32(uint32_t* value);


    uint16_t PacketExtract_uint64(uint64_t* value);

    uint16_t PacketExtract_float32(float32_t* value);
    uint16_t PacketExtract_float64(float64_t* value);

    uint16_t PacketExtract_charArray(char* value, uint16_t size);


    uint16_t PacketExtract_uint16Array(uint16_t* value, uint16_t size);
    uint8_t offset_;
private:
    std::vector<uint8_t> buffer_;

};

#endif // DESERIALIZER_H
