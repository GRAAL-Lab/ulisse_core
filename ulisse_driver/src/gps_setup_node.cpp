/*
 * taskGPSSetup.cc
 *
 *  Created on: Feb 27, 2017
 *      Author: wonder
 */

#include <rclcpp/rclcpp.hpp>

#include "ulisse_driver/CSerialHelper.h"

int CheckCommandConfirmation(const char* serialDevice, const char* command, const char* expectedAnswer,
		bool& matchingAnswer) {
    ulisse::CSerialHelper* serial = ulisse::CSerialHelper::getInstance(serialDevice, 115200);

	char buffer;
	struct timeval tmp;
	tmp.tv_sec = 1;
	tmp.tv_usec = 0;
	bool exit = false;
	int ret = ORTOS_RV_OK;
	char answerStartCharacter = '<';
	bool answereStartDetected = false;
	int answerPosition = 0;
	int expectedAnswerLength = strlen(expectedAnswer);
	std::string answerText;
	matchingAnswer = true;

	while (exit != true) {
		ret = serial->ReadNonblocking(&buffer, 1, tmp);
		switch (ret) {
		case SERIAL_ERROR:
			ortos::DebugConsole::Write(ortos::LogLevel::error, "CheckCommandConfirmation", "ReadNonblocking error");
			ret = ORTOS_RV_FAIL;
			exit = true;
			break;
		case SERIAL_TIMEOUT:
			ortos::DebugConsole::Write(ortos::LogLevel::error, "CheckCommandConfirmation", "ReadNonblocking timeout");
			ret = ORTOS_RV_FAIL;
			exit = true;
			break;
		default: {
			ret = ORTOS_RV_OK;
			if (!answereStartDetected) {
				if (buffer == answerStartCharacter) {
					answereStartDetected = true;
				}
			} else {
				answerText.push_back(buffer);

				if (buffer == '\r') {
					exit = true;
				}

				if (buffer != expectedAnswer[answerPosition]) {
					if (matchingAnswer == true) {
						matchingAnswer = false;
						ortos::DebugConsole::Write(ortos::LogLevel::warning, "CheckCommandConfirmation",
								"Answer doesn't match");
					}
				} else {
					++answerPosition;
					if (answerPosition == expectedAnswerLength) {
						exit = true;
						matchingAnswer = true;
					}
				}
			}
		}
			break;
		}
	}

	if (matchingAnswer == false) {
		ortos::DebugConsole::Write(ortos::LogLevel::warning, "CheckCommandConfirmation",
				"Sent command:%s Answer text:%s", command, answerText.c_str());
	}

	return ret;
}

int main(int argc, char *argv[]) {

	std::string serialDevice = "/dev/ttyS1";

	for (int i = 1; i < argc; i++) {

		if ((strcmp(argv[i], "--serial") == 0) && (i + 1 < argc)) {
			serialDevice.assign(argv[i + 1]);
		}

		if ((strcmp(argv[i], "--help") == 0) || (strcmp(argv[i], "-h") == 0)) {
			ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Usage: %s [options]", argv[0]);
			ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Options");
			ortos::DebugConsole::Write(ortos::LogLevel::info, "main",
					"--serial device\t: 'device' is the serial port (default %s)", serialDevice.c_str());
			return 0;
		}
	}

	ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Stopping gpsd server");
	int status = system("service gpsd stop");
	ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "service gpsd stop returned %d", status);

    ulisse::CSerialHelper* serial = ulisse::CSerialHelper::getInstance(serialDevice.c_str(), 9600);

	std::string stringToSend;
	int ret;
	bool matchingAnswer;

	if (serial->IsOpen()) {
		stringToSend.assign("\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());

		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Setting Baudrate to 115200");

		stringToSend.assign("COM COM1 115200 N 8 1 N OFF\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());

		sleep(1);

		serial->ChangeBaudRate(115200);

		sleep(1);

		stringToSend.assign("\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());

		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Clear all logs");

		stringToSend.assign("UNLOGALL COM1 TRUE\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Clearing logs failed! (%s)",
					stringToSend.c_str());
			return ORTOS_RV_FAIL;
		}

		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Asking for GPRMC message every 0.1s");

		stringToSend.assign("LOG COM1 GPRMC ONTIME 0.1\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Asking for GPRMC packet failed! (%s)",
					stringToSend.c_str());

			return ORTOS_RV_FAIL;
		}

		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Asking for GPGSA message every 0.1s");

		stringToSend.assign("LOG COM1 GPGSA ONTIME 0.1\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Asking for GPGSA packet failed! (%s)",
					stringToSend.c_str());
		}

		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Asking for GPGSV message every 0.1s");

		stringToSend.assign("LOG COM1 GPGSV ONTIME 0.1\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Asking for GPGSV packet failed! (%s)",
					stringToSend.c_str());
		}

		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Asking for SATVISA message every 60s");

		stringToSend.assign("LOG COM1 SATVISA ONTIME 60\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Asking for SATVISA packet failed! (%s)",
					stringToSend.c_str());
		}

		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Asking for SATVIS2A message on data change");

		stringToSend.assign("LOG COM1 SATVIS2A ONCHANGED\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Asking for SATVIS2A packet failed! (%s)",
					stringToSend.c_str());
		}

		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Asking for PSRDOPA message on data change");

		stringToSend.assign("LOG COM1 PSRDOPA ONCHANGED\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Asking for PSRDOPA packet failed! (%s)",
					stringToSend.c_str());
		}

		// enabling satellite based corrections
		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "ENABLING SBAS");

		stringToSend.assign("SBASCONTROL ENABLE\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Enabling SBAS failed! (%s)",
					stringToSend.c_str());
			return ORTOS_RV_FAIL;
		}

		// pdpfilter setting to dynamic, hopefully to prevent pinning a position when the system moves slowly
		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Setting PDPMODE to DYNAMIC");

		stringToSend.assign("PDPMODE NORMAL DYNAMIC\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Setting PDPMODE to DYNAMIC failed! (%s)",
					stringToSend.c_str());
			return ORTOS_RV_FAIL;
		}

		/*
		ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Disabling PDPFILTER");

		stringToSend.assign("PDPFILTER DISABLE\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

		if (!((ret == ORTOS_RV_OK) && (matchingAnswer))) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Disabling PDPFILTER failed! (%s)",
					stringToSend.c_str());
			return ORTOS_RV_FAIL;
		}
		*/
	}

	ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Starting gpsd server");
	status = system("service gpsd start");
	ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "service gpsd start returned %d", status);
}

