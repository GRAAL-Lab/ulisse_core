/*
 * EESHelper.h
 *
 *  Created on: Jun 16, 2016
 *      Author: wonder
 */

#ifndef SRC_COMM_EESHELPER_H_
#define SRC_COMM_EESHELPER_H_

#include <cstdio>

#include "ulisse_driver/CSerialHelper.h"
#include "ulisse_driver/EESHelperDataStructs.h"
//#include "Functions.h"

namespace ulisse {

namespace ees {

    class EESHelper {
    public:
        EESHelper();
        ~EESHelper();
        ReturnValue SetSerial(std::string serialDevice, int baudRate);
        ReturnValue CollectValidMessage(EESData& data);
        ReturnValue SendMessage(EESData& data);

        void DebugBytes(bool enable);
        void DebugIncomingValidMessageType(bool enable);
        void DebugFailedCrc(bool enable);

    private:
        void Init();
        ReturnValue ParseByte(uint8_t byte);
        MessageType GetLastMessage();
        ReturnValue CreateEESMessage(uint8_t*, uint16_t& size, EESData& data);

        void CreateReferences(uint8_t* packetPointer, uint16_t& size, referencesData& references);
        void CreateSetConfig(uint8_t* packetPointer, uint16_t& size, LowLevelConfiguration& config);
        void CreateBeep(uint8_t* packetPointer, uint16_t& size, beepData& beep);
        void CreateGetVersion(uint8_t* packetPointer, uint16_t& size);
        void CreateEnableReferences(uint8_t* packetPointer, uint16_t& size, enableRefData& enable);
        void CreateStartCompassCalibration(uint8_t* packetPointer, uint16_t& size);
        void CreateStopCompassCalibration(uint8_t* packetPointer, uint16_t& size);
        void CreateReset(uint8_t* packetPointer, uint16_t& size);
        void CreateGetConfig(uint8_t* packetPointer, uint16_t& size);
        void CreatePumps(uint8_t* packetPointer, uint16_t& size, pumpsData& pumps);
        void CreatePwrButtons(uint8_t* packetPointer, uint16_t& size, pwrButtonsData& pwrButtons);

        void GetAck(ackData& ack);
        void GetStatus(statusData& status);
        void GetSensors(sensorData& sensors);
        void ConvertSensors(sensorData& sensors);
        void GetVersion(versionData& version);
        void GetConfig(LowLevelConfiguration& config);
        void GetMotors(motorsData& motors);
        void GetBattery(batteryData& battery);
        void GetSw485Status(sw485StatusData& sw485Status);

        uint16_t ComputeByteSum(const uint8_t* bytes, uint16_t size);
        uint16_t CalculateChecksum(const uint8_t* bytes, uint16_t size);
        uint16_t CheckChecksum(const uint8_t* bytes, uint16_t size, uint16_t checksum);

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

        CSerialHelper* serial_;
        uint8_t* outgoingPacketBuffer_;
        uint8_t* incomingPacketBuffer_;

        ParseState state_;
        uint16_t headerCount_;
        uint16_t dataCount_;
        uint16_t checksumCount_;
        uint16_t errorCount_;
        uint16_t payloadSize_;
        int type_;
        uint16_t size_;
        uint8_t* ptr_;
        uint16_t recvChecksum_;
        MessageType lastReceived_;
        uint16_t maxPayloadSize_;

        bool debugBytes_;
        bool debugIncomingValidMessageType_;
        bool debugFailedCrc_;
    };

} //namespace ees

} //namespace ulisse

#endif /* SRC_COMM_EESHELPER_H_ */
