/*
 * EESHelperDataStructs.cc
 *
 *  Created on: Jun 22, 2016
 *      Author: wonder
 */

#include "ulisse_driver/EESHelperDataStructs.h"

namespace ulisse {

namespace ees {
/*
ReferencesContainer::ReferencesContainer() {
	timestamp = 0;
	Describe();
}

ReferencesContainer::ReferencesContainer(const ReferencesContainer& rhs)
{
	timestamp = rhs.timestamp;
	d = rhs.d;
	Describe();
}

void ReferencesContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int64, 1, &timestamp);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.leftThruster);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.rightThruster);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.speed);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.jog);
	Finalize();
}

SensorsContainer::SensorsContainer() {
	timestamp = 0;
	Describe();
}

SensorsContainer::SensorsContainer(const SensorsContainer& rhs)
{
	timestamp = rhs.timestamp;
	d = rhs.d;
	Describe();
}

void SensorsContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int64, 1, &timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint32, 1, &d.timestamp);
	AddElement(ortos::xcom::DataType::ortos_float, 3, &d.accelerometer);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.compassRoll);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.compassPitch);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.compassHeading);
	AddElement(ortos::xcom::DataType::ortos_float, 3, &d.magnetometer);
	AddElement(ortos::xcom::DataType::ortos_float, 3, &d.gyro);
	AddElement(ortos::xcom::DataType::ortos_float, 2, &d.gyro4x);
	AddElement(ortos::xcom::DataType::ortos_uint32, 1, &d.stepsSincePPS);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.temperatureCtrlBox);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.humidityCtrlBox);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.sensorStatus);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.leftReference);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.rightReference);
	Finalize();
}

StatusContainer::StatusContainer() {
	timestamp = 0;
	Describe();
}

StatusContainer::StatusContainer(const StatusContainer& rhs)
{
	timestamp = rhs.timestamp;
	d = rhs.d;
	Describe();
}

void StatusContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int64, 1, &timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint32, 1, &d.timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.status);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.commDataErrorCount);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.i2cDataState);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.missedDeadlines);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.accelerometerTimeouts);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.compassTimeouts);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.magnetometerTimeouts);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.i2cbusBusy);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.messageSent485);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.messageReceived485);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.errorCount);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.overflowCount232);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.overflowCount485);
	Finalize();
}

ConfigContainer::ConfigContainer() {
	timestamp = 0;
	Describe();
}

ConfigContainer::ConfigContainer(const ConfigContainer& rhs)
{
	timestamp = rhs.timestamp;
	d = rhs.d;
	Describe();
}

void ConfigContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int64, 1, &timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbCompass0);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbCompassMax);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbMagnetometer0);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbMagnetometerMax);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbPacketSensors0);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbPacketSensorsMax);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbPacketStatus0);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbPacketStatusMax);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbPacketMotors0);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbPacketMotorsMax);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbPacketBattery0);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.hbPacketBatteryMax);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.timeoutAccelerometer);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.timeoutCompass);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.timeoutMagnetometer);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.pwmUpMin);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.pwmUpMax);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.pwmPeriodMin);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.pwmPeriodMax);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.pwmTimeThreshold);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.pwmZeroThreshold);
	AddElement(ortos::xcom::DataType::ortos_float, 1, &d.deadzoneTime);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.thrusterSaturation);
	Finalize();
}

AckContainer::AckContainer() {
	timestamp = 0;
	Describe();
}

AckContainer::AckContainer(const AckContainer& rhs)
{
	timestamp = rhs.timestamp;
	d = rhs.d;
	Describe();
}

void AckContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int64, 1, &timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.messagetype);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.ack);
	Finalize();
}

VersionContainer::VersionContainer() {
	timestamp = 0;
	Describe();
}

VersionContainer::VersionContainer(const VersionContainer& rhs)
{
	timestamp = rhs.timestamp;
	d = rhs.d;
	Describe();
}

void VersionContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int64, 1, &timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.mdVersion);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.swVersion);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.lsatVersion);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.rsatVersion);
	Finalize();
}

MotorsContainer::MotorsContainer() {
	timestamp = 0;
	Describe();
}

MotorsContainer::MotorsContainer(const MotorsContainer& rhs)
{
	timestamp = rhs.timestamp;
	d = rhs.d;
	Describe();
}

void MotorsContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int64, 1, &timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint32, 1, &d.timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint32, 1, &d.left.timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.flags0);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.flags1);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.master_state);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.master_error_code);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.motor_voltage);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.left.motor_current);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.motor_power);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.left.motor_speed);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.motor_pcb_temp);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.motor_stator_temp);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.battery_charge);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.battery_voltage);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.battery_current);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.gps_speed);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.range_miles);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.range_minutes);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.temperature_sw);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.temperature_rp);

	AddElement(ortos::xcom::DataType::ortos_uint32, 1, &d.right.timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.flags0);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.flags1);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.master_state);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.master_error_code);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.motor_voltage);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.right.motor_current);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.motor_power);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.right.motor_speed);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.motor_pcb_temp);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.motor_stator_temp);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.battery_charge);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.battery_voltage);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.battery_current);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.gps_speed);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.range_miles);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.range_minutes);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.temperature_sw);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.temperature_rp);
	Finalize();
}

BatteryContainer::BatteryContainer() {
	timestamp = 0;
	Describe();
}

BatteryContainer::BatteryContainer(const BatteryContainer& rhs)
{
	timestamp = rhs.timestamp;
	d = rhs.d;
	Describe();
}

void BatteryContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int64, 1, &timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint32, 1, &d.left.timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.left.id);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.left.timestampSW485);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.left.timestampSatellite);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.voltage);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.left.current);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.chargePercent);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.temperature);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.equalisationCells);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.commandState);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.left.alarmState);
	AddElement(ortos::xcom::DataType::ortos_float, 14, d.left.cells);
	AddElement(ortos::xcom::DataType::ortos_uint32, 1, &d.right.timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint8, 1, &d.right.id);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.right.timestampSW485);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.right.timestampSatellite);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.voltage);
	AddElement(ortos::xcom::DataType::ortos_int16, 1, &d.right.current);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.chargePercent);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.temperature);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.equalisationCells);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.commandState);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.right.alarmState);
	AddElement(ortos::xcom::DataType::ortos_float, 14, d.right.cells);
	Finalize();
}

Sw485StatusContainer::Sw485StatusContainer() {
	timestamp = 0;
	Describe();
}

Sw485StatusContainer::Sw485StatusContainer(const Sw485StatusContainer& rhs)
{
	timestamp = rhs.timestamp;
	d = rhs.d;
	Describe();
}

void Sw485StatusContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int64, 1, &timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint32, 1, &d.timestamp);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.timestampSW485);
	AddElement(ortos::xcom::DataType::ortos_uint16, 1, &d.missedDeadlines);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.leftMotor.sent);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.leftMotor.received);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.rightMotor.sent);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.rightMotor.received);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.leftSatellite.sent);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.leftSatellite.received);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.rightSatellite.sent);
	AddElement(ortos::xcom::DataType::ortos_uint64, 1, &d.rightSatellite.received);
	Finalize();
}
*/
std::string CommandTypeToString(CommandType type) {
	std:: string name;
     uint16_t type_uint = (uint16_t)type;

    switch(type_uint) {
	case (uint16_t)CommandType::undefined:
		name = "undefined";
		break;
	case (uint16_t)CommandType::beep:
		name = "beep";
		break;
	case (uint16_t)CommandType::enableref:
		name = "enableref";
		break;
	case (uint16_t)CommandType::setconfig:
		name = "setconfig";
		break;
	case (uint16_t)CommandType::setpumps:
		name = "setpumps";
		break;
	case (uint16_t)CommandType::setpowerbuttons:
		name = "setpowerbuttons";
		break;
	case (uint16_t)CommandType::getconfig:
		name = "getconfig";
		break;
	case (uint16_t)CommandType::getversion:
		name = "getversion";
		break;
	case (uint16_t)CommandType::startcompasscal:
		name = "startcompasscal";
		break;
	case (uint16_t)CommandType::stopcompasscal:
		name = "stopcompasscal";
		break;
	case (uint16_t)CommandType::reset:
		name = "reset";
		break;
	case (uint16_t)CommandType::reloadconfig:
		name = "reloadconfig";
		break;
	default:
        name = "Unhandled...please update om2ctrl::ees::CommandTypeToString method adding command type " + std::to_string(type_uint);
		break;
	}

	return name;
}

std::string CommandAnswerToString(CommandAnswer answer) {
	std:: string name;
    int16_t ans_int = (int16_t)answer;

    switch(ans_int) {
    case (int16_t)CommandAnswer::fail:
		name = "fail";
		break;
    case (int16_t)CommandAnswer::undefined:
		name = "undefined";
		break;
    case (int16_t)CommandAnswer::ok:
		name = "ok";
		break;
	default:
        name = "Unhandled...please update om2ctrl::ees::CommandAnswerToString() method adding answer type " + std::to_string(ans_int);
		break;
	}

	return name;
}
/*
CommandContainer::CommandContainer() {
	timestamp = 0;
	commandType = (uint16_t)CommandType::undefined;

	SetProducerTaskName("Undefined");

	DescribeAndFinalize();
}

CommandContainer::CommandContainer(const CommandContainer& rhs)
{
	timestamp = rhs.timestamp;
	commandType = rhs.commandType;

	SetProducerTaskName(rhs.producerTaskName);

	DescribeAndFinalize();
}

void CommandContainer::Describe() {
	AddElement(ortos::xcom::DataType::ortos_int8, sizeof(d), &d);
}

void CommandContainer::DescribeAndFinalize() {
	BaseCommandContainer::Describe();
	CommandContainer::Describe();
	Finalize();
}

void CommandContainer::Serialize(ortos::xcom::BaseMarshaller& marshaller) const {
	BaseCommandContainer::Serialize(marshaller);

	switch (commandType) {
		case (uint16_t)CommandType::beep:
			marshaller.MarshalUInt8(reinterpret_cast<const uint8_t*>(&d.beep.numberOfBeeps), 1);
			marshaller.MarshalUInt8(reinterpret_cast<const uint8_t*>(&d.beep.loop), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.beep.delay), 1);
			break;
		case (uint16_t)CommandType::enableref:
			marshaller.MarshalUInt8(reinterpret_cast<const uint8_t*>(&d.enableRef.enable), 1);
			break;
		case (uint16_t)CommandType::setconfig:
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbCompass0), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbCompassMax), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbMagnetometer0), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbMagnetometerMax), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbPacketSensors0), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbPacketSensorsMax), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbPacketStatus0), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbPacketStatusMax), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbPacketMotors0), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbPacketMotorsMax), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbPacketBattery0), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.hbPacketBatteryMax), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.setConfig.timeoutAccelerometer), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.setConfig.timeoutCompass), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.setConfig.timeoutMagnetometer), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.setConfig.pwmUpMin), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.setConfig.pwmUpMax), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.setConfig.pwmPeriodMin), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.setConfig.pwmPeriodMax), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.setConfig.pwmTimeThreshold), 1);
			marshaller.MarshalFloat(reinterpret_cast<const float32_t*>(&d.setConfig.deadzoneTime), 1);
			marshaller.MarshalUInt16(reinterpret_cast<const uint16_t*>(&d.setConfig.thrusterSaturation), 1);
			break;
		case (uint16_t)CommandType::setpumps:
			marshaller.MarshalUInt8(reinterpret_cast<const uint8_t*>(d.pumps.pumpsFlag), 2);
			break;
		case (uint16_t)CommandType::setpowerbuttons:
			marshaller.MarshalUInt8(reinterpret_cast<const uint8_t*>(&d.powerButtons.pwrButtonsFlag), 1);
			break;
		case (uint16_t)CommandType::getconfig:
			break;
		case (uint16_t)CommandType::getversion:
			break;
		case (uint16_t)CommandType::startcompasscal:
			break;
		case (uint16_t)CommandType::stopcompasscal:
			break;
		case (uint16_t)CommandType::reset:
			break;
		case (uint16_t)CommandType::reloadconfig:
			break;
		default:
			ortos::DebugConsole::Write(ortos::LogLevel::warning, "CommandContainer::Serialize", "Unhandled command %s", CommandTypeToString(commandType).c_str());
			break;
	}
}

void CommandContainer::Deserialize(ortos::xcom::BaseMarshaller& marshaller) {
	BaseCommandContainer::Deserialize(marshaller);

	switch (commandType) {
	case (uint16_t)CommandType::beep:
		marshaller.UnmarshalUInt8(&d.beep.numberOfBeeps, 1);
		marshaller.UnmarshalUInt8(&d.beep.loop, 1);
		marshaller.UnmarshalFloat(&d.beep.delay, 1);
		break;
	case (uint16_t)CommandType::enableref:
		marshaller.UnmarshalUInt8(&d.enableRef.enable, 1);
		break;
	case (uint16_t)CommandType::setconfig:
		marshaller.UnmarshalUInt16(&d.setConfig.hbCompass0, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbCompassMax, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbMagnetometer0, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbMagnetometerMax, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbPacketSensors0, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbPacketSensorsMax, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbPacketStatus0, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbPacketStatusMax, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbPacketMotors0, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbPacketMotorsMax, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbPacketBattery0, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.hbPacketBatteryMax, 1);
		marshaller.UnmarshalFloat(&d.setConfig.timeoutAccelerometer, 1);
		marshaller.UnmarshalFloat(&d.setConfig.timeoutCompass, 1);
		marshaller.UnmarshalFloat(&d.setConfig.timeoutMagnetometer, 1);
		marshaller.UnmarshalFloat(&d.setConfig.pwmUpMin, 1);
		marshaller.UnmarshalFloat(&d.setConfig.pwmUpMax, 1);
		marshaller.UnmarshalFloat(&d.setConfig.pwmPeriodMin, 1);
		marshaller.UnmarshalFloat(&d.setConfig.pwmPeriodMax, 1);
		marshaller.UnmarshalFloat(&d.setConfig.pwmTimeThreshold, 1);
		marshaller.UnmarshalFloat(&d.setConfig.deadzoneTime, 1);
		marshaller.UnmarshalUInt16(&d.setConfig.thrusterSaturation, 1);
		break;
	case (uint16_t)CommandType::setpumps:
		marshaller.UnmarshalUInt8(d.pumps.pumpsFlag, 2);
		break;
	case (uint16_t)CommandType::setpowerbuttons:
		marshaller.UnmarshalUInt8(&d.powerButtons.pwrButtonsFlag, 1);
		break;
	case (uint16_t)CommandType::getconfig:
		break;
	case (uint16_t)CommandType::getversion:
		break;
	case (uint16_t)CommandType::startcompasscal:
		break;
	case (uint16_t)CommandType::stopcompasscal:
		break;
	case (uint16_t)CommandType::reset:
		break;
	case (uint16_t)CommandType::reloadconfig:
		break;
	default:
		ortos::DebugConsole::Write(ortos::LogLevel::warning, "CommandContainer::Deserialize", "Unhandled command %s", CommandTypeToString(commandType).c_str());
		break;
	}
}

CommandAnswerContainer::CommandAnswerContainer() {
	timestamp = 0;
	answer = (uint16_t)CommandAnswer::undefined;
	commandType_ = (uint16_t)CommandType::undefined;
	commandTimestamp_ = 0;

	SetProducerTaskName("Undefined");

	DescribeAndFinalize();
}

CommandAnswerContainer::CommandAnswerContainer(const CommandAnswerContainer& rhs)
{
	timestamp = rhs.timestamp;
	answer = rhs.answer;

	commandType_ = rhs.commandType_;
	commandTimestamp_ = rhs.commandTimestamp_;

	SetProducerTaskName(rhs.producerTaskName_);

	DescribeAndFinalize();
}

void CommandAnswerContainer::DescribeAndFinalize() {
	BaseCommandAnswerContainer::Describe();
	Finalize();
}
*/
} //namespace ees

} //namespace ulisse
