#include "ulisse_driver/serializer.h"
#include <string.h>

Serializer::Serializer(std::shared_ptr<uint8_t*> buffer)
{
    buffer_ = buffer;
    offset_ = 0;
}

uint16_t Serializer::PacketAdd_char(char value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(char));
    offset_ += sizeof(char);
    return offset_;
}

uint16_t Serializer::PacketAdd_int8(int8_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(int8_t));
    offset_ += sizeof(int8_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_int16(int16_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(int16_t));
    offset_ += sizeof(int16_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_int32(int32_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(int32_t));
    offset_ += sizeof(int32_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_int64(int64_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(int64_t));
    offset_ += sizeof(int64_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_uint8(uint8_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(uint8_t));
    offset_ += sizeof(uint8_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_uint16(uint16_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(uint16_t));
    offset_ += sizeof(uint16_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_uint32(uint32_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(uint32_t));
    offset_ += sizeof(uint32_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_uint64(uint64_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(uint64_t));
    offset_ += sizeof(uint64_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_float32(float32_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(float32_t));
    offset_ += sizeof(float32_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_float64(float64_t value)
{
    memcpy(buffer_.get() + offset_, &value, sizeof(float64_t));
    offset_ += sizeof(float64_t);
    return offset_;
}

uint16_t Serializer::PacketAdd_charArray(char* value, uint16_t size)
{
    memcpy(buffer_.get() + offset_, value, size);
    offset_ += size;
    return offset_;
}

uint16_t Serializer::PacketAdd_uint16Array(uint16_t* value, uint16_t size)
{
    memcpy(buffer_.get() + offset_, value, size * sizeof(uint16_t));
    offset_ += size * sizeof(uint16_t);
    return offset_;
}
