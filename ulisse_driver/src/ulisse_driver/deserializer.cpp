#include "ulisse_driver/deserializer.h"
#include <string.h>
#include <iostream>

Deserializer::Deserializer(std::vector<uint8_t> buffer)
{
    buffer_ = buffer;
    offset_ = 0;
}

void Deserializer::MoveOffset(uint8_t newOffset)
{
    offset_ = newOffset;
}

uint16_t Deserializer::PacketExtract_char(char* value)
{
    memcpy(value, &buffer_[offset_], sizeof(char));
    offset_ += sizeof(char);
    return offset_;
}

uint16_t Deserializer::PacketExtract_int8(int8_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(int8_t));
    offset_ += sizeof(int8_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_int16(int16_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(int16_t));
    offset_ += sizeof(int16_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_int32(int32_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(int32_t));
    offset_ += sizeof(int32_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_int64(int64_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(int64_t));
    offset_ += sizeof(int64_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_uint8(uint8_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(uint8_t));
    offset_ += sizeof(uint8_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_uint16(uint16_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(uint16_t));
    offset_ += sizeof(uint16_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_uint32(uint32_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(uint32_t));
    offset_ += sizeof(uint32_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_uint64(uint64_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(uint64_t));
    offset_ += sizeof(uint64_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_float32(float32_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(float32_t));
    offset_ += sizeof(float32_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_float64(float64_t* value)
{
    memcpy(value, &buffer_[offset_], sizeof(float64_t));
    offset_ += sizeof(float64_t);
    return offset_;
}

uint16_t Deserializer::PacketExtract_charArray(char* value, uint16_t size)
{
    memcpy(value, &buffer_[offset_], size);
    offset_ += size;
    return offset_;
}

uint16_t Deserializer::PacketExtract_uint16Array(uint16_t* value, uint16_t size)
{
    memcpy(value, &buffer_[offset_], size * sizeof(uint16_t));
    offset_ += size * sizeof(uint16_t);
    return offset_;
}
