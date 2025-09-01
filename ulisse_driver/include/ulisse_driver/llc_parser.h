#ifndef LLC_PARSER_H
#define LLC_PARSER_H

#include <stdint.h>
#include <memory>
#include <vector>

typedef float float32_t;
typedef double float64_t;

enum class ParseState {
    header,
    data,
    checksum,
    payload
};

class LLCParser {
public:
    LLCParser();

    int ParseByte(uint8_t byte);

    uint16_t PacketAdd_char(uint8_t* buffer, uint16_t offset, char value);
    uint16_t PacketAdd_int8(uint8_t* buffer, uint16_t offset, int8_t value);
    uint16_t PacketAdd_int16(uint8_t* buffer, uint16_t offset, int16_t value);
    uint16_t PacketAdd_int32(uint8_t* buffer, uint16_t offset, int32_t value);
    uint16_t PacketAdd_int64(uint8_t* buffer, uint16_t offset, int64_t value);
    uint16_t PacketAdd_uint8(uint8_t* buffer, uint16_t offset, uint8_t value);
    uint16_t PacketAdd_uint16(uint8_t* buffer, uint16_t offset, uint16_t value);
    uint16_t PacketAdd_uint32(uint8_t* buffer, uint16_t offset, uint32_t value);
    uint16_t PacketAdd_uint64(uint8_t* buffer, uint16_t offset, uint64_t value);
    uint16_t PacketAdd_float32(uint8_t* buffer, uint16_t offset, float32_t value);
    uint16_t PacketAdd_float64(uint8_t* buffer, uint16_t offset, float64_t value);
    uint16_t PacketAdd_charArray(uint8_t* buffer, uint16_t offset, char* value, uint16_t size);
    uint16_t PacketAdd_uint16Array(uint8_t* buffer, uint16_t offset, uint16_t* value, uint16_t size);

    uint16_t PacketExtract_char(uint8_t* buffer, uint16_t offset, char* value);
    uint16_t PacketExtract_int8(uint8_t* buffer, uint16_t offset, int8_t* value);
    uint16_t PacketExtract_int16(uint8_t* buffer, uint16_t offset, int16_t* value);
    uint16_t PacketExtract_int32(uint8_t* buffer, uint16_t offset, int32_t* value);
    uint16_t PacketExtract_int64(uint8_t* buffer, uint16_t offset, int64_t* value);
    uint16_t PacketExtract_uint8(uint8_t* buffer, uint16_t offset, uint8_t* value);
    uint16_t PacketExtract_uint16(uint8_t* buffer, uint16_t offset, uint16_t* value);
    uint16_t PacketExtract_uint32(uint8_t* buffer, uint16_t offset, uint32_t* value);
    uint16_t PacketExtract_uint64(uint8_t* buffer, uint16_t offset, uint64_t* value);
    uint16_t PacketExtract_float32(uint8_t* buffer, uint16_t offset, float32_t* value);
    uint16_t PacketExtract_float64(uint8_t* buffer, uint16_t offset, float64_t* value);
    uint16_t PacketExtract_charArray(uint8_t* buffer, uint16_t offset, char* value, uint16_t size);
    uint16_t PacketExtract_uint16Array(uint8_t* buffer, uint16_t offset, uint16_t* value, uint16_t size);

    void FinalizePacket(uint8_t* packetPointer, uint16_t offset, uint16_t& size);

    int GetLastMessageType() {
        return type_;
    }

    uint16_t GetSize() {
        return size_;
    }

    std::vector<uint8_t> GetIncomingBuffer() {
        return incomingPacketBuffer_;
    }

private:
    uint16_t ComputeByteSum(const uint8_t* bytes, uint16_t size);
    uint16_t CalculateChecksum(const uint8_t* bytes, uint16_t size);
    uint16_t CheckChecksum(const uint8_t* bytes, uint16_t size, uint16_t checksum);


    ParseState state_;
    uint16_t headerCount_;
    uint16_t dataCount_;
    uint16_t checksumCount_;
    uint16_t errorCount_;
    uint16_t payloadSize_;
    int type_;
    uint16_t size_;

    uint16_t recvChecksum_;
    uint16_t maxPayloadSize_;

    bool debugBytes_;
    bool debugIncomingValidMessageType_;
    bool debugFailedCrc_;

    std::vector<uint8_t> incomingPacketBuffer_;
};

#endif // LLC_PARSER_H
