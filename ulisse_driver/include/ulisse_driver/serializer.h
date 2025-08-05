#ifndef SERIALIZER_H
#define SERIALIZER_H

#include <stdint.h>
#include <memory>
#include <vector>

typedef float float32_t;
typedef double float64_t;

class Serializer
{
public:
    Serializer();

    uint16_t PacketAdd_char(char value);

    uint16_t PacketAdd_int8(int8_t value);

    uint16_t PacketAdd_int16(int16_t value);

    uint16_t PacketAdd_int32(int32_t value);

    uint16_t PacketAdd_int64(int64_t value);

    uint16_t PacketAdd_uint8(uint8_t value);

    uint16_t PacketAdd_uint16(uint16_t value);

    uint16_t PacketAdd_uint32(uint32_t value);

    uint16_t PacketAdd_uint64(uint64_t value);

    uint16_t PacketAdd_float32(float32_t value);

    uint16_t PacketAdd_float64(float64_t value);

    uint16_t PacketAdd_charArray(char* value, uint16_t size);

    uint16_t PacketAdd_uint16Array(uint16_t* value, uint16_t size);

    auto Buffer() const -> const std::vector<uint8_t>& { return buffer_; }

    void Reset() { offset_ = 0; }

private:
    std::vector<uint8_t> buffer_;
    uint8_t offset_;
};

#endif // SERIALIZER_H
