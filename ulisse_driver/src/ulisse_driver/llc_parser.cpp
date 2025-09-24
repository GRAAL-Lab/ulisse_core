#include "ulisse_driver/llc_parser.h"
#include <iostream>
#include <string.h>

LLCParser::LLCParser()
{
    maxPayloadSize_ = 1000;
    incomingPacketBuffer_.resize(maxPayloadSize_);
}

int LLCParser::ParseByte(uint8_t byte)
{
    switch (state_) {
    case ParseState::header: {
        // 0x0A 0x0B SIZE_LSB SIZE_MSB TYPE_LSB TYPE_MSB **PAYLOAD** CHECKSUM_LSB CHECKSUM_MSB
        // checksum computed from size field to end of payload, i.e. excluding the preamble 0A0B
        if (headerCount_ == 0) {
            if (byte == 0x0A) {
                headerCount_++;
            } else {
                headerCount_ = 0;
            }
        } else if (headerCount_ == 1) {
            if (byte == 0x0B) {
                headerCount_++;
            } else
                headerCount_ = 0;
        } else if (headerCount_ == 2) // size lsb
        {
            payloadSize_ = byte;
            headerCount_++;

            // first byte is the size
            incomingPacketBuffer_[0] = byte;
            dataCount_ = 1;
        } else if (headerCount_ == 3) // size msb
        {
            payloadSize_ += (byte << 8); // shifting the byte since this is MSB
            headerCount_++;

            if (payloadSize_ > maxPayloadSize_) {
                state_ = ParseState::header;
                headerCount_ = 0;
                errorCount_++;
                return -1;
            } else {
                incomingPacketBuffer_[dataCount_] = byte;
                dataCount_++;
            }
        } else if (headerCount_ == 4) // type lsb
        {
            type_ = byte;
            headerCount_++;
            incomingPacketBuffer_[dataCount_] = byte;
            dataCount_++;
        } else if (headerCount_ == 5) // type msb
        {
            type_ += (byte << 8); // shifting the byte since this is MSB
            headerCount_++;

            state_ = ParseState::payload;

            // incrementing to take into account_ also the size field
            // since the checksum needs to encompass also the size
            payloadSize_ += 2;
            incomingPacketBuffer_[dataCount_] = byte;
            dataCount_++;
        }
        break;
    }
    case ParseState::payload: {
        // check if we already have read enough bytes
        if (dataCount_ == payloadSize_) {
            // set the state_ as checksum and fallover
            state_ = ParseState::checksum;
            size_ = payloadSize_;
            // no break
        } else {
            // byte is payload, set the buffer accordingly
            incomingPacketBuffer_[dataCount_] = byte;
            dataCount_++;
            break;
        }
    } // no break is ok
        [[fallthrough]];
    case ParseState::checksum: {
        // parse the two bytes checksum
        if (checksumCount_ == 0) {
            uint8_t* ptr = (uint8_t*)(&recvChecksum_);
            *ptr = byte;
            checksumCount_ += 1;
        } else {
            uint8_t* ptr = (uint8_t*)(&recvChecksum_);
            *(ptr + 1) = byte;
            checksumCount_ = 0;

            if (CheckChecksum(&incomingPacketBuffer_[0], size_, recvChecksum_) == 1) {
                state_ = ParseState::header;
                headerCount_ = 0;

                return 1;
            } else {
                state_ = ParseState::header;
                headerCount_ = 0;
                errorCount_++;
                return -2;
            }
        }

        break;
    }
    default: {
        headerCount_ = 0;
        state_ = ParseState::header;
        break;
    }
    }

    return 0;
}

uint16_t LLCParser::ComputeByteSum(const uint8_t* bytes, uint16_t size)
{
    int32_t remainder;
    uint16_t word;
    char* msb;
    char* lsb;
    const uint8_t* ptr = bytes;
    uint16_t sum = 0;

    remainder = size;
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

uint16_t LLCParser::CalculateChecksum(const uint8_t* bytes, uint16_t size)
{
    uint16_t checksum = ComputeByteSum(bytes, size);

    checksum = ~(checksum) + 1;

    return checksum;
}

uint16_t LLCParser::CheckChecksum(const uint8_t* bytes, uint16_t size, uint16_t checksum)
{
    uint16_t sum = ComputeByteSum(bytes, size);

    sum += checksum;

    if (sum == 0) {
        return 1;
    } else {
        return 0;
    }
}
