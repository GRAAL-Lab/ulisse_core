#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>
#include <vector>

namespace chksm {
uint16_t ComputeByteSum(const std::vector<uint8_t>& bytes, uint16_t offset)
{
    uint16_t size = bytes.size();
    int32_t remainder;
    uint16_t word;
    char* msb;
    char* lsb;
    const uint8_t* ptr = bytes.data() + offset;
    uint16_t sum = 0;

    remainder = size - offset;
    lsb = (char*)(&word);
    msb = lsb + 1;

    while (remainder > 0) {
        if (remainder == 1) {
            *msb = 0x00;
            *lsb = *ptr;
            remainder -= 1;
            ptr = ptr + 1;
        } else {
            *msb = *(ptr + 1);
            *lsb = *(ptr);
            remainder -= 2;
            ptr = ptr + 2;
        }

        sum += word;
    }

    return sum;
}

uint16_t CalculateChecksum(const std::vector<uint8_t>& bytes, uint16_t offset)
{
    uint16_t checksum = ComputeByteSum(bytes, offset);

    checksum = ~(checksum) + 1;

    return checksum;
}

uint16_t CheckChecksum(const std::vector<uint8_t>& bytes, uint16_t offset, uint16_t checksum)
{
    uint16_t sum = ComputeByteSum(bytes, offset);

    sum += checksum;

    if (sum == 0) {
        return 1;
    } else {
        return 0;
    }
}
}

#endif