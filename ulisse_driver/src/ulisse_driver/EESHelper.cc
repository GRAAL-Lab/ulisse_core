/*
 * EESHelper.cc
 *
 *  Created on: Jun 16, 2016
 *      Author: wonder
 */

#include "ulisse_driver/EESHelper.h"

namespace ulisse {

namespace ees {

    EESHelper::EESHelper()
    {
        serial_ = NULL;

        EESData tmp;

        maxPayloadSize_ = tmp.GetMaxSize() + 8; // max data + 0a0b + size + type + crc

        outgoingPacketBuffer_ = new uint8_t[maxPayloadSize_];
        incomingPacketBuffer_ = new uint8_t[maxPayloadSize_];

        debugBytes_ = false;
        debugIncomingValidMessageType_ = false;
        debugFailedCrc_ = false;

        Init();
    }

    EESHelper::~EESHelper()
    {
        delete[] outgoingPacketBuffer_;
        delete[] incomingPacketBuffer_;
    }

    ReturnValue EESHelper::SetSerial(std::string serialDevice, int baudRate)
    {
        serial_ = CSerialHelper::getInstance(serialDevice.c_str(), baudRate);

        if (serial_->IsOpen()) {
            Init();
            return ReturnValue::ok;
        } else {
            return ReturnValue::fail;
        }
    }

    ReturnValue EESHelper::CollectValidMessage(EESData& data)
    {
        //TODO portare fuori i vari GetAck e soci per evitare il passaggio inutile dalla union
        if (serial_->IsOpen()) {
            MessageType messageId;
            char byte;
            while (1) {
                int ret = serial_->ReadBlocking(&byte, 1);

                if (ret == 1) {
                    ReturnValue retval = ParseByte(byte);

                    if (retval == ReturnValue::complete) {
                        messageId = GetLastMessage();

                        if (debugIncomingValidMessageType_) {
                            fprintf(stderr, "Received messageId %u\n", (uint16_t)messageId);
                        }

                        switch (messageId) {

                        case MessageType::ack: {
                            ackData tmp;
                            GetAck(tmp);
                            data.messageType = messageId;
                            data.ack = tmp;
                            return ReturnValue::ok;
                        } break;
                        case MessageType::status: {
                            statusData tmp;
                            GetStatus(tmp);
                            data.messageType = messageId;
                            data.status = tmp;
                            return ReturnValue::ok;
                        } break;
                        case MessageType::sensor: {
                            sensorData tmp;
                            GetSensors(tmp);
                            ConvertSensors(tmp);
                            data.messageType = messageId;
                            data.sensors = tmp;
                            return ReturnValue::ok;
                        } break;
                        case MessageType::version: {
                            versionData tmp;
                            GetVersion(tmp);
                            data.messageType = messageId;
                            data.version = tmp;
                            return ReturnValue::ok;
                        } break;
                        case MessageType::set_config: {
                            configData tmp;
                            GetConfig(tmp);
                            data.messageType = messageId;
                            data.config = tmp;
                            return ReturnValue::ok;
                        } break;
                        case MessageType::motors: {
                            motorsData tmp;
                            GetMotors(tmp);
                            data.messageType = messageId;
                            data.motors = tmp;
                            return ReturnValue::ok;
                        } break;
                        case MessageType::battery: {
                            batteryData tmp;
                            GetBattery(tmp);
                            data.messageType = messageId;
                            data.battery = tmp;
                            return ReturnValue::ok;
                        } break;
                        case MessageType::sw485Status: {
                            sw485StatusData tmp;
                            GetSw485Status(tmp);
                            data.messageType = messageId;
                            data.sw485Status = tmp;
                            return ReturnValue::ok;
                        } break;
                        default:
                            //ortos::DebugConsole::Write(ortos::LogLevel::warning, "EESHelper::CollectValidMessage", "unsupported message id %d", messageId);
                            break;
                        }
                    }
                } else {
                    //ortos::DebugConsole::Write(ortos::LogLevel::warning, "EESHelper::CollectValidMessage", "serial->ReadBlocking returned %d", ret);
                }
            }
        } else {
            //ortos::DebugConsole::Write(ortos::LogLevel::warning, "EESHelper::CollectValidMessage", "Invalid or closed channel");
            return ReturnValue::fail;
        }
    }

    ReturnValue EESHelper::SendMessage(EESData& data)
    {
        if (serial_->IsOpen()) {
            uint16_t size = 0;
            ReturnValue ret = CreateEESMessage(outgoingPacketBuffer_, size, data);

            if (ret == ReturnValue::ok) {

                if (debugBytes_) {
                    fprintf(stderr, "\n------type %d-------\n", (int)data.messageType);
                    for (unsigned int i = 0; i < size; ++i)
                        fprintf(stderr, "%02x ", outgoingPacketBuffer_[i]);
                    fprintf(stderr, "\n--------------------\n");
                }
                serial_->Write((char*)outgoingPacketBuffer_, size);
            } else {
                return ReturnValue::fail;
            }

        } else {
            //ortos::DebugConsole::Write(ortos::LogLevel::warning, "EESHelper::SendMessage", "Invalid or closed channel");
            return ReturnValue::fail;
        }
        return ReturnValue::ok;
    }

    void EESHelper::DebugBytes(bool enable)
    {
        debugBytes_ = enable;

        if (enable)
            fprintf(stderr, "Debug Bytes Activated\n");
    }

    void EESHelper::DebugIncomingValidMessageType(bool enable)
    {
        debugIncomingValidMessageType_ = enable;

        if (enable)
            fprintf(stderr, "Debug Incoming Valid Message Type Activated\n");
    }

    void EESHelper::DebugFailedCrc(bool enable)
    {
        debugFailedCrc_ = enable;

        if (enable)
            fprintf(stderr, "Debug Failed CRC Activated\n");
    }

    void EESHelper::Init()
    {
        state_ = ParseState::header;
        headerCount_ = 0;
        dataCount_ = 0;
        checksumCount_ = 0;
        errorCount_ = 0;
    }

    ReturnValue EESHelper::ParseByte(uint8_t byte)
    {
        uint8_t* payload_ptr;

        if (debugBytes_)
            fprintf(stderr, "%02x ", byte);

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
            } else if (headerCount_ == 2) //size lsb
            {
                payloadSize_ = byte;
                headerCount_++;

                // first byte is the size
                payload_ptr = (uint8_t*)incomingPacketBuffer_;
                *(payload_ptr) = byte;
                dataCount_ = 1;
            } else if (headerCount_ == 3) //size msb
            {
                payloadSize_ += (byte << 8); // shifting the byte since this is MSB
                headerCount_++;

                if (payloadSize_ > maxPayloadSize_) {
                    //ortos::DebugConsole::Write(ortos::LogLevel::warning, "EESHelper", "Payload size too big (%u > %u)", payloadSize_, maxPayloadSize_);

                    state_ = ParseState::header;
                    headerCount_ = 0;
                    errorCount_++;
                    return ReturnValue::fail;
                } else {
                    payload_ptr = (uint8_t*)incomingPacketBuffer_;
                    *(payload_ptr + dataCount_) = byte;
                    dataCount_++;
                }
            } else if (headerCount_ == 4) //type lsb
            {
                type_ = byte;
                headerCount_++;

                payload_ptr = (uint8_t*)incomingPacketBuffer_;
                *(payload_ptr + dataCount_) = byte;
                dataCount_++;
            } else if (headerCount_ == 5) //type msb
            {
                type_ += (byte << 8); // shifting the byte since this is MSB
                headerCount_++;

                state_ = ParseState::payload;

                // incrementing to take into account_ also the size field
                // since the checksum needs to encompass also the size
                payloadSize_ += 2;

                payload_ptr = (uint8_t*)incomingPacketBuffer_;
                *(payload_ptr + dataCount_) = byte;
                dataCount_++;
            }
            break;
        }
        case ParseState::payload: {
            // check if we already have read enough bytes
            if (dataCount_ == payloadSize_) {
                // set the state_ as checksum and fallover
                state_ = ParseState::checksum;
                //size = payloadSize;
                size_ = payloadSize_;
                ptr_ = (uint8_t*)incomingPacketBuffer_;
                // no break
            } else {
                // byte is payload, set the buffer accordingly
                payload_ptr = (uint8_t*)incomingPacketBuffer_;
                *(payload_ptr + dataCount_) = byte;
                dataCount_++;
                break;
            }
        } //no break is ok
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

                if (CheckChecksum(ptr_, size_, recvChecksum_) == 1) {
                    state_ = ParseState::header;
                    headerCount_ = 0;
                    lastReceived_ = (MessageType)type_;

                    if (debugBytes_) {
                        fprintf(stderr, "\n CRC ok \n");
                    }

                    return ReturnValue::complete;
                } else {
                   // ortos::DebugConsole::Write(ortos::LogLevel::warning, "EESHelper", "ERROR: message %u size %u CHECKSUM FAIL, received %u computed %u sum %u", type_, size_, recvChecksum_,
                   //     ComputeByteSum(ptr, size_), ComputeByteSum(ptr, size_) + recvChecksum_);

                    if (debugFailedCrc_) {
                        fprintf(stderr, "\n--rcv-type %d-------\n", type_);
                        fprintf(stderr, "0a 0b ");

                        for (int i = 0; i < dataCount_; ++i) {
                            fprintf(stderr, "%02x ", incomingPacketBuffer_[i]);
                        }
                        fprintf(stderr, "%02x %02x", ((uint8_t*)(&recvChecksum_))[0], ((uint8_t*)(&recvChecksum_))[1]);
                        fprintf(stderr, "\n--------------------\n");
                    }

                    state_ = ParseState::header;
                    headerCount_ = 0;
                    errorCount_++;
                    return ReturnValue::fail;
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

        return ReturnValue::nodata;
    }

    MessageType EESHelper::GetLastMessage()
    {
        return lastReceived_;
    }

    ReturnValue EESHelper::CreateEESMessage(uint8_t* packetPointer, uint16_t& size, EESData& data)
    {
        switch (data.messageType) {
        case MessageType::reference:
            CreateReferences(outgoingPacketBuffer_, size, data.references);
            break;
        case MessageType::beep:
            CreateBeep(outgoingPacketBuffer_, size, data.beep);
            break;
        case MessageType::enable_ref:
            CreateEnableReferences(outgoingPacketBuffer_, size, data.enableRef);
            break;
        case MessageType::set_config:
            CreateSetConfig(outgoingPacketBuffer_, size, data.config);
            break;
        case MessageType::get_config:
            CreateGetConfig(outgoingPacketBuffer_, size);
            break;
        case MessageType::get_version:
            CreateGetVersion(outgoingPacketBuffer_, size);
            break;
        case MessageType::start_compass_cal:
            CreateStartCompassCalibration(outgoingPacketBuffer_, size);
            break;
        case MessageType::stop_compass_cal:
            CreateStopCompassCalibration(outgoingPacketBuffer_, size);
            break;
        case MessageType::reset:
            CreateReset(outgoingPacketBuffer_, size);
            break;
        case MessageType::pumps:
            CreatePumps(outgoingPacketBuffer_, size, data.pumps);
            break;
        case MessageType::pwrbuttons:
            CreatePwrButtons(outgoingPacketBuffer_, size, data.pwrButtons);
            break;
        default:
            //ortos::DebugConsole::Write(ortos::LogLevel::info, "EESHelper::CreateEESMessage", "type unknown (%d)", data.messageType);
            return ReturnValue::fail;
            break;
        }
        return ReturnValue::ok;
    }

    void EESHelper::CreateReferences(uint8_t* packetPointer, uint16_t& size, referencesData& references)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, references.GetSize() + 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::reference);
        offset = PacketAdd_int16(packetPointer, offset, references.leftThruster);
        offset = PacketAdd_int16(packetPointer, offset, references.rightThruster);
        offset = PacketAdd_int16(packetPointer, offset, references.speed);
        offset = PacketAdd_int16(packetPointer, offset, references.jog);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreateSetConfig(uint8_t* packetPointer, uint16_t& size, configData& config)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, config.GetSize() + 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::set_config);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbCompass0);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbCompassMax);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbMagnetometer0);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbMagnetometerMax);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbPacketSensors0);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbPacketSensorsMax);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbPacketStatus0);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbPacketStatusMax);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbPacketMotors0);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbPacketMotorsMax);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbPacketBattery0);
        offset = PacketAdd_uint16(packetPointer, offset, config.hbPacketBatteryMax);
        offset = PacketAdd_float32(packetPointer, offset, config.timeoutAccelerometer);
        offset = PacketAdd_float32(packetPointer, offset, config.timeoutCompass);
        offset = PacketAdd_float32(packetPointer, offset, config.timeoutMagnetometer);
        offset = PacketAdd_float32(packetPointer, offset, config.pwmUpMin);
        offset = PacketAdd_float32(packetPointer, offset, config.pwmUpMax);
        offset = PacketAdd_float32(packetPointer, offset, config.pwmPeriodMin);
        offset = PacketAdd_float32(packetPointer, offset, config.pwmPeriodMax);
        offset = PacketAdd_float32(packetPointer, offset, config.pwmTimeThreshold);
        offset = PacketAdd_float32(packetPointer, offset, config.pwmZeroThreshold);
        offset = PacketAdd_float32(packetPointer, offset, config.deadzoneTime);
        offset = PacketAdd_uint16(packetPointer, offset, config.thrusterSaturation);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreateBeep(uint8_t* packetPointer, uint16_t& size, beepData& beep)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, beep.GetSize() + 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::beep);
        offset = PacketAdd_uint8(packetPointer, offset, beep.numberOfBeeps);
        offset = PacketAdd_uint8(packetPointer, offset, beep.loop);
        offset = PacketAdd_float32(packetPointer, offset, beep.delay);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreateGetVersion(uint8_t* packetPointer, uint16_t& size)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::get_version);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreateEnableReferences(uint8_t* packetPointer, uint16_t& size, enableRefData& enable)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, enable.GetSize() + 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::enable_ref);
        offset = PacketAdd_uint8(packetPointer, offset, enable.enable);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreateStartCompassCalibration(uint8_t* packetPointer, uint16_t& size)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::start_compass_cal);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreateStopCompassCalibration(uint8_t* packetPointer, uint16_t& size)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::stop_compass_cal);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreateReset(uint8_t* packetPointer, uint16_t& size)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::reset);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreateGetConfig(uint8_t* packetPointer, uint16_t& size)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::get_config);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreatePumps(uint8_t* packetPointer, uint16_t& size, pumpsData& pumps)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, pumps.GetSize() + 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::pumps);
        offset = PacketAdd_uint8(packetPointer, offset, pumps.pumpsFlag[EMB_PUMPS_LEFT_IDX]);
        offset = PacketAdd_uint8(packetPointer, offset, pumps.pumpsFlag[EMB_PUMPS_RIGHT_IDX]);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::CreatePwrButtons(uint8_t* packetPointer, uint16_t& size, pwrButtonsData& pwrButtons)
    {
        uint16_t offset = 0;

        offset = PacketAdd_char(packetPointer, offset, 0x0A);
        offset = PacketAdd_char(packetPointer, offset, 0x0B);
        offset = PacketAdd_uint16(packetPointer, offset, pwrButtons.GetSize() + 2);
        offset = PacketAdd_uint16(packetPointer, offset, (uint16_t)MessageType::pwrbuttons);
        offset = PacketAdd_uint8(packetPointer, offset, pwrButtons.pwrButtonsFlag);

        return FinalizePacket(packetPointer, offset, size);
    }

    void EESHelper::GetAck(ackData& ack)
    {
        uint8_t* packetPointer = &incomingPacketBuffer_[0];
        uint16_t offset = 4;

        offset = PacketExtract_uint16(packetPointer, offset, &ack.messagetype);
        offset = PacketExtract_uint8(packetPointer, offset, &ack.ack);
    }

    void EESHelper::GetStatus(statusData& status)
    {
        uint8_t* packetPointer = &incomingPacketBuffer_[0];
        uint16_t offset = 4;

        offset = PacketExtract_uint32(packetPointer, offset, &status.timestamp);
        offset = PacketExtract_uint16(packetPointer, offset, &status.status);
        offset = PacketExtract_uint16(packetPointer, offset, &status.commDataErrorCount);
        offset = PacketExtract_int16(packetPointer, offset, &status.i2cDataState);
        offset = PacketExtract_uint16(packetPointer, offset, &status.missedDeadlines);
        offset = PacketExtract_uint16(packetPointer, offset, &status.accelerometerTimeouts);
        offset = PacketExtract_uint16(packetPointer, offset, &status.compassTimeouts);
        offset = PacketExtract_uint16(packetPointer, offset, &status.magnetometerTimeouts);
        offset = PacketExtract_uint16(packetPointer, offset, &status.i2cbusBusy);
        offset = PacketExtract_uint64(packetPointer, offset, &status.messageSent485);
        offset = PacketExtract_uint64(packetPointer, offset, &status.messageReceived485);
        offset = PacketExtract_uint16(packetPointer, offset, &status.errorCount);
        offset = PacketExtract_uint16(packetPointer, offset, &status.overflowCount232);
        offset = PacketExtract_uint16(packetPointer, offset, &status.overflowCount485);
    }

    void EESHelper::GetSensors(sensorData& sensors)
    {
        uint8_t* packetPointer = &incomingPacketBuffer_[0];
        uint16_t offset = 4;

        int i;

        offset = PacketExtract_uint32(packetPointer, offset, &sensors.timestamp);
        for (i = 0; i < 3; i++)
            offset = PacketExtract_float32(packetPointer, offset, &sensors.accelerometer[i]);
        offset = PacketExtract_float32(packetPointer, offset, &sensors.compassRoll);
        offset = PacketExtract_float32(packetPointer, offset, &sensors.compassPitch);
        offset = PacketExtract_float32(packetPointer, offset, &sensors.compassHeading);
        for (i = 0; i < 3; i++)
            offset = PacketExtract_float32(packetPointer, offset, &sensors.magnetometer[i]);
        for (i = 0; i < 3; i++)
            offset = PacketExtract_float32(packetPointer, offset, &sensors.gyro[i]);
        for (i = 0; i < 2; i++)
            offset = PacketExtract_float32(packetPointer, offset, &sensors.gyro4x[i]);
        offset = PacketExtract_uint32(packetPointer, offset, &sensors.stepsSincePPS);
        offset = PacketExtract_float32(packetPointer, offset, &sensors.temperatureCtrlBox);
        offset = PacketExtract_float32(packetPointer, offset, &sensors.humidityCtrlBox);
        offset = PacketExtract_uint8(packetPointer, offset, &sensors.sensorStatus);
        offset = PacketExtract_int16(packetPointer, offset, &sensors.leftReference);
        offset = PacketExtract_int16(packetPointer, offset, &sensors.rightReference);
    }

    /**               |
* Vehicle frame   | EES Sensor frame
*                 |
*        ^ x      |  y ^
*        |        |    |
*        |        |    |
* y <----o z      |  z o----> x
*                 |
*/

    void EESHelper::ConvertSensors(sensorData& sensors)
    {
        float32_t tmp;

        tmp = sensors.accelerometer[1];
        sensors.accelerometer[1] = -sensors.accelerometer[0];
        sensors.accelerometer[0] = tmp;

        tmp = sensors.gyro[1];
        sensors.gyro[1] = -sensors.gyro[0];
        sensors.gyro[0] = tmp;

        tmp = sensors.gyro4x[1];
        sensors.gyro4x[1] = -sensors.gyro4x[0];
        sensors.gyro4x[0] = tmp;

        tmp = sensors.magnetometer[1];
        sensors.magnetometer[1] = -sensors.magnetometer[0];
        sensors.magnetometer[0] = tmp;

        tmp = sensors.compassPitch;
        sensors.compassPitch = -sensors.compassRoll;
        sensors.compassRoll = tmp;
        sensors.compassHeading -= M_PI / 2.0;

        if (sensors.compassHeading < 0)
            sensors.compassHeading += 2 * M_PI;
        else if (sensors.compassHeading > 2 * M_PI)
            sensors.compassHeading -= 2 * M_PI;
    }

    void EESHelper::GetVersion(versionData& version)
    {
        uint8_t* packetPointer = &incomingPacketBuffer_[0];
        uint16_t offset = 4;

        offset = PacketExtract_uint16(packetPointer, offset, &version.mdVersion);
        offset = PacketExtract_uint16(packetPointer, offset, &version.swVersion);
        offset = PacketExtract_uint16(packetPointer, offset, &version.lsatVersion);
        offset = PacketExtract_uint16(packetPointer, offset, &version.rsatVersion);
    }

    void EESHelper::GetConfig(configData& config)
    {
        uint8_t* packetPointer = &incomingPacketBuffer_[0];
        uint16_t offset = 4;

        offset = PacketExtract_uint16(packetPointer, offset, &config.hbCompass0);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbCompassMax);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbMagnetometer0);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbMagnetometerMax);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbPacketSensors0);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbPacketSensorsMax);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbPacketStatus0);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbPacketStatusMax);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbPacketMotors0);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbPacketMotorsMax);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbPacketBattery0);
        offset = PacketExtract_uint16(packetPointer, offset, &config.hbPacketBatteryMax);
        offset = PacketExtract_float32(packetPointer, offset, &config.timeoutAccelerometer);
        offset = PacketExtract_float32(packetPointer, offset, &config.timeoutCompass);
        offset = PacketExtract_float32(packetPointer, offset, &config.timeoutMagnetometer);
        offset = PacketExtract_float32(packetPointer, offset, &config.pwmUpMin);
        offset = PacketExtract_float32(packetPointer, offset, &config.pwmUpMax);
        offset = PacketExtract_float32(packetPointer, offset, &config.pwmPeriodMin);
        offset = PacketExtract_float32(packetPointer, offset, &config.pwmPeriodMax);
        offset = PacketExtract_float32(packetPointer, offset, &config.pwmTimeThreshold);
        offset = PacketExtract_float32(packetPointer, offset, &config.pwmZeroThreshold);
        offset = PacketExtract_float32(packetPointer, offset, &config.deadzoneTime);
        offset = PacketExtract_uint16(packetPointer, offset, &config.thrusterSaturation);
    }

    void EESHelper::GetMotors(motorsData& motors)
    {
        uint8_t* packetPointer = &incomingPacketBuffer_[0];
        uint16_t offset = 4;

        offset = PacketExtract_uint32(packetPointer, offset, &motors.timestamp);

        offset = PacketExtract_uint32(packetPointer, offset, &motors.left.timestamp);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.left.flags0);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.left.flags1);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.left.master_state);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.left.master_error_code);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.left.motor_voltage);
        offset = PacketExtract_int16(packetPointer, offset, &motors.left.motor_current);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.left.motor_power);
        offset = PacketExtract_int16(packetPointer, offset, &motors.left.motor_speed);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.left.motor_pcb_temp);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.left.motor_stator_temp);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.left.battery_charge);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.left.battery_voltage);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.left.battery_current);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.left.gps_speed);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.left.range_miles);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.left.range_minutes);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.left.temperature_sw);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.left.temperature_rp);

        offset = PacketExtract_uint32(packetPointer, offset, &motors.right.timestamp);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.right.flags0);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.right.flags1);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.right.master_state);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.right.master_error_code);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.right.motor_voltage);
        offset = PacketExtract_int16(packetPointer, offset, &motors.right.motor_current);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.right.motor_power);
        offset = PacketExtract_int16(packetPointer, offset, &motors.right.motor_speed);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.right.motor_pcb_temp);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.right.motor_stator_temp);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.right.battery_charge);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.right.battery_voltage);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.right.battery_current);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.right.gps_speed);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.right.range_miles);
        offset = PacketExtract_uint16(packetPointer, offset, &motors.right.range_minutes);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.right.temperature_sw);
        offset = PacketExtract_uint8(packetPointer, offset, &motors.right.temperature_rp);
    }

    void EESHelper::GetBattery(batteryData& battery)
    {
        uint8_t* packetPointer = &incomingPacketBuffer_[0];
        uint16_t offset = 4;

        offset = PacketExtract_uint32(packetPointer, offset, &battery.timestamp);
        offset = PacketExtract_uint8(packetPointer, offset, &battery.id);
        offset = PacketExtract_uint64(packetPointer, offset, &battery.timestampSW485);
        offset = PacketExtract_uint64(packetPointer, offset, &battery.timestampSatellite);
        offset = PacketExtract_uint16(packetPointer, offset, &battery.voltage);
        offset = PacketExtract_int16(packetPointer, offset, &battery.current);
        offset = PacketExtract_uint16(packetPointer, offset, &battery.chargePercent);
        offset = PacketExtract_uint16(packetPointer, offset, &battery.temperature);
        offset = PacketExtract_uint16(packetPointer, offset, &battery.equalisationCells);
        offset = PacketExtract_uint16(packetPointer, offset, &battery.commandState);
        offset = PacketExtract_uint16(packetPointer, offset, &battery.alarmState);

        for (int i = 0; i < 14; i++) {
            offset = PacketExtract_float32(packetPointer, offset, &battery.cells[i]);
        }
    }

    void EESHelper::GetSw485Status(sw485StatusData& sw485Status)
    {
        uint8_t* packetPointer = &incomingPacketBuffer_[0];
        uint16_t offset = 4;

        offset = PacketExtract_uint32(packetPointer, offset, &sw485Status.timestamp);
        offset = PacketExtract_uint64(packetPointer, offset, &sw485Status.timestampSW485);
        offset = PacketExtract_uint16(packetPointer, offset, &sw485Status.missedDeadlines);
        offset = PacketExtract_uint64(packetPointer, offset, &sw485Status.leftMotor.received);
        offset = PacketExtract_uint64(packetPointer, offset, &sw485Status.leftMotor.sent);
        offset = PacketExtract_uint64(packetPointer, offset, &sw485Status.rightMotor.received);
        offset = PacketExtract_uint64(packetPointer, offset, &sw485Status.rightMotor.sent);
        offset = PacketExtract_uint64(packetPointer, offset, &sw485Status.leftSatellite.received);
        offset = PacketExtract_uint64(packetPointer, offset, &sw485Status.leftSatellite.sent);
        offset = PacketExtract_uint64(packetPointer, offset, &sw485Status.rightSatellite.received);
        offset = PacketExtract_uint64(packetPointer, offset, &sw485Status.rightSatellite.sent);
    }

    uint16_t EESHelper::ComputeByteSum(const uint8_t* bytes, uint16_t size)
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

    uint16_t EESHelper::CalculateChecksum(const uint8_t* bytes, uint16_t size)
    {
        uint16_t checksum = ComputeByteSum(bytes, size);

        checksum = ~(checksum) + 1;

        return checksum;
    }

    uint16_t EESHelper::CheckChecksum(const uint8_t* bytes, uint16_t size, uint16_t checksum)
    {
        uint16_t sum = ComputeByteSum(bytes, size);

        sum += checksum;

        if (sum == 0) {
            return 1;
        } else {
            return 0;
        }
    }

    uint16_t EESHelper::PacketAdd_char(uint8_t* buffer, uint16_t offset, char value)
    {
        memcpy(buffer + offset, &value, sizeof(char));
        offset += sizeof(char);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_int8(uint8_t* buffer, uint16_t offset, int8_t value)
    {
        memcpy(buffer + offset, &value, sizeof(int8_t));
        offset += sizeof(int8_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_int16(uint8_t* buffer, uint16_t offset, int16_t value)
    {
        memcpy(buffer + offset, &value, sizeof(int16_t));
        offset += sizeof(int16_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_int32(uint8_t* buffer, uint16_t offset, int32_t value)
    {
        memcpy(buffer + offset, &value, sizeof(int32_t));
        offset += sizeof(int32_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_int64(uint8_t* buffer, uint16_t offset, int64_t value)
    {
        memcpy(buffer + offset, &value, sizeof(int64_t));
        offset += sizeof(int64_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_uint8(uint8_t* buffer, uint16_t offset, uint8_t value)
    {
        memcpy(buffer + offset, &value, sizeof(uint8_t));
        offset += sizeof(uint8_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_uint16(uint8_t* buffer, uint16_t offset, uint16_t value)
    {
        memcpy(buffer + offset, &value, sizeof(uint16_t));
        offset += sizeof(uint16_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_uint32(uint8_t* buffer, uint16_t offset, uint32_t value)
    {
        memcpy(buffer + offset, &value, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_uint64(uint8_t* buffer, uint16_t offset, uint64_t value)
    {
        memcpy(buffer + offset, &value, sizeof(uint64_t));
        offset += sizeof(uint64_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_float32(uint8_t* buffer, uint16_t offset, float32_t value)
    {
        memcpy(buffer + offset, &value, sizeof(float32_t));
        offset += sizeof(float32_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_float64(uint8_t* buffer, uint16_t offset, float64_t value)
    {
        memcpy(buffer + offset, &value, sizeof(float64_t));
        offset += sizeof(float64_t);
        return offset;
    }

    uint16_t EESHelper::PacketAdd_charArray(uint8_t* buffer, uint16_t offset, char* value, uint16_t size)
    {
        memcpy(buffer + offset, value, size);
        offset += size;
        return offset;
    }

    uint16_t EESHelper::PacketAdd_uint16Array(uint8_t* buffer, uint16_t offset, uint16_t* value, uint16_t size)
    {
        memcpy(buffer + offset, value, size * sizeof(uint16_t));
        offset += size * sizeof(uint16_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_char(uint8_t* buffer, uint16_t offset, char* value)
    {
        memcpy(value, buffer + offset, sizeof(char));
        offset += sizeof(char);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_int8(uint8_t* buffer, uint16_t offset, int8_t* value)
    {
        memcpy(value, buffer + offset, sizeof(int8_t));
        offset += sizeof(int8_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_int16(uint8_t* buffer, uint16_t offset, int16_t* value)
    {
        memcpy(value, buffer + offset, sizeof(int16_t));
        offset += sizeof(int16_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_int32(uint8_t* buffer, uint16_t offset, int32_t* value)
    {
        memcpy(value, buffer + offset, sizeof(int32_t));
        offset += sizeof(int32_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_int64(uint8_t* buffer, uint16_t offset, int64_t* value)
    {
        memcpy(value, buffer + offset, sizeof(int64_t));
        offset += sizeof(int64_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_uint8(uint8_t* buffer, uint16_t offset, uint8_t* value)
    {
        memcpy(value, buffer + offset, sizeof(uint8_t));
        offset += sizeof(uint8_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_uint16(uint8_t* buffer, uint16_t offset, uint16_t* value)
    {
        memcpy(value, buffer + offset, sizeof(uint16_t));
        offset += sizeof(uint16_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_uint32(uint8_t* buffer, uint16_t offset, uint32_t* value)
    {
        memcpy(value, buffer + offset, sizeof(uint32_t));
        offset += sizeof(uint32_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_uint64(uint8_t* buffer, uint16_t offset, uint64_t* value)
    {
        memcpy(value, buffer + offset, sizeof(uint64_t));
        offset += sizeof(uint64_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_float32(uint8_t* buffer, uint16_t offset, float32_t* value)
    {
        memcpy(value, buffer + offset, sizeof(float32_t));
        offset += sizeof(float32_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_float64(uint8_t* buffer, uint16_t offset, float64_t* value)
    {
        memcpy(value, buffer + offset, sizeof(float64_t));
        offset += sizeof(float64_t);
        return offset;
    }

    uint16_t EESHelper::PacketExtract_charArray(uint8_t* buffer, uint16_t offset, char* value, uint16_t size)
    {
        memcpy(value, buffer + offset, size);
        offset += size;
        return offset;
    }

    uint16_t EESHelper::PacketExtract_uint16Array(uint8_t* buffer, uint16_t offset, uint16_t* value, uint16_t size)
    {
        memcpy(value, buffer + offset, size * sizeof(uint16_t));
        offset += size * sizeof(uint16_t);
        return offset;
    }

    void EESHelper::FinalizePacket(uint8_t* packetPointer, uint16_t offset, uint16_t& size)
    {
        uint16_t checksum;

        // checksum calculation does not involve the first two bytes
        // offset is the current size of the message
        checksum = CalculateChecksum(packetPointer + 2, offset - 2);

        // add checksum and the packet is complete
        offset = PacketAdd_uint16(packetPointer, offset, checksum);

        size = offset;
    }

} //namespace ees

} //namespace ulisse
