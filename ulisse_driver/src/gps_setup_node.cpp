/*
 * taskGPSSetup.cc
 *
 *  Created on: Feb 27, 2017
 *      Author: wonder
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ulisse_driver/CSerialHelper.h"

using namespace std::chrono_literals;

static rclcpp::Node::SharedPtr node;

bool CheckCommandConfirmation(const char* serialDevice, const char* command, const char* expectedAnswer,
    bool& matchingAnswer);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("gps_setup_node");

    auto par_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node);

    while (!par_client_->wait_for_service(1ms)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(0);
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }

    std::string serialDevice = "";
    uint32_t baudRate = 0;
    node->get_parameter_or("SerialDevice", serialDevice, std::string());
    node->get_parameter_or("BaudRate", baudRate, 0u);

    RCLCPP_INFO(node->get_logger(), "GPS SETUP: %s, %u", serialDevice.c_str(), baudRate);

    //    //std::string serialDevice = "/dev/ttyS1";

    //    for (int i = 1; i < argc; i++) {

    //        if ((strcmp(argv[i], "--serial") == 0) && (i + 1 < argc)) {
    //            serialDevice.assign(argv[i + 1]);
    //        }

    //        if ((strcmp(argv[i], "--help") == 0) || (strcmp(argv[i], "-h") == 0)) {
    //            RCLCPP_INFO(node->get_logger(), "Usage: %s [options]", argv[0]);
    //            RCLCPP_INFO(node->get_logger(), "Options");
    //            RCLCPP_INFO(node->get_logger(),
    //                "--serial device\t: 'device' is the serial port (default %s)", serialDevice.c_str());
    //            return 0;
    //        }
    //    }

    RCLCPP_INFO(node->get_logger(), "Stopping gpsd server");
    int status = system("sudo service gpsd stop");
    RCLCPP_INFO(node->get_logger(), "service gpsd stop returned %d", status);

    ulisse::CSerialHelper* serial = ulisse::CSerialHelper::getInstance(serialDevice.c_str(), 9600);

    std::string stringToSend;
    int ret;
    bool matchingAnswer;

    if (serial->IsOpen()) {
        stringToSend.assign("\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());

        RCLCPP_INFO(node->get_logger(), "Setting Baudrate to %u", baudRate);

        stringToSend.assign("COM COM1 115200 N 8 1 N OFF\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());

        sleep(1);

        serial->ChangeBaudRate(baudRate);

        sleep(1);

        stringToSend.assign("\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());

        RCLCPP_INFO(node->get_logger(), "Clear all logs");

        stringToSend.assign("UNLOGALL COM1 TRUE\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());
        ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Clearing logs failed! (%s)",
                stringToSend.c_str());
            return EXIT_FAILURE;
        }

        RCLCPP_INFO(node->get_logger(), "Asking for GPRMC message every 0.1s");

        stringToSend.assign("LOG COM1 GPRMC ONTIME 0.1\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());
        ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Asking for GPRMC packet failed! (%s)",
                stringToSend.c_str());

            return EXIT_FAILURE;
        }

        RCLCPP_INFO(node->get_logger(), "Asking for GPGSA message every 0.1s");

        stringToSend.assign("LOG COM1 GPGSA ONTIME 0.1\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());
        ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Asking for GPGSA packet failed! (%s)",
                stringToSend.c_str());
        }

        RCLCPP_INFO(node->get_logger(), "Asking for GPGSV message every 0.1s");

        stringToSend.assign("LOG COM1 GPGSV ONTIME 0.1\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());
        ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Asking for GPGSV packet failed! (%s)",
                stringToSend.c_str());
        }

        RCLCPP_INFO(node->get_logger(), "Asking for SATVISA message every 60s");

        stringToSend.assign("LOG COM1 SATVISA ONTIME 60\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());
        ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Asking for SATVISA packet failed! (%s)",
                stringToSend.c_str());
        }

        RCLCPP_INFO(node->get_logger(), "Asking for SATVIS2A message on data change");

        stringToSend.assign("LOG COM1 SATVIS2A ONCHANGED\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());
        ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Asking for SATVIS2A packet failed! (%s)",
                stringToSend.c_str());
        }

        RCLCPP_INFO(node->get_logger(), "Asking for PSRDOPA message on data change");

        stringToSend.assign("LOG COM1 PSRDOPA ONCHANGED\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());
        ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Asking for PSRDOPA packet failed! (%s)",
                stringToSend.c_str());
        }

        // enabling satellite based corrections
        RCLCPP_INFO(node->get_logger(), "ENABLING SBAS");

        stringToSend.assign("SBASCONTROL ENABLE\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());
        ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Enabling SBAS failed! (%s)",
                stringToSend.c_str());
            return EXIT_FAILURE;
        }

        // pdpfilter setting to dynamic, hopefully to prevent pinning a position when the system moves slowly
        RCLCPP_INFO(node->get_logger(), "Setting PDPMODE to DYNAMIC");

        stringToSend.assign("PDPMODE NORMAL DYNAMIC\n\r");
        serial->Write(stringToSend.c_str(), stringToSend.size());
        ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Setting PDPMODE to DYNAMIC failed! (%s)",
                stringToSend.c_str());
            return EXIT_FAILURE;
        }

        /*
        RCLCPP_INFO(node->get_logger(), "Disabling PDPFILTER");

		stringToSend.assign("PDPFILTER DISABLE\n\r");
		serial->Write(stringToSend.c_str(), stringToSend.size());
		ret = CheckCommandConfirmation(serialDevice.c_str(), stringToSend.c_str(), "OK", matchingAnswer);

        if (!(ret && (matchingAnswer))) {
            RCLCPP_ERROR(node->get_logger(), "Disabling PDPFILTER failed! (%s)",
					stringToSend.c_str());
            return EXIT_FAILURE;
		}
		*/
    }

    RCLCPP_INFO(node->get_logger(), "Starting gpsd server");
    status = system("sudo service gpsd start");
    RCLCPP_INFO(node->get_logger(), "sudo service gpsd start returned %d", status);

    rclcpp::shutdown();
}

bool CheckCommandConfirmation(const char* serialDevice, const char* command, const char* expectedAnswer,
    bool& matchingAnswer)
{
    ulisse::CSerialHelper* serial = ulisse::CSerialHelper::getInstance(serialDevice, 115200);

    char buffer;
    struct timeval tmp;
    tmp.tv_sec = 1;
    tmp.tv_usec = 0;
    bool exit = false;
    int serialRet;
    bool ret = false;
    char answerStartCharacter = '<';
    bool answereStartDetected = false;
    int answerPosition = 0;
    int expectedAnswerLength = strlen(expectedAnswer);
    std::string answerText;
    matchingAnswer = true;

    while (exit != true) {
        serialRet = serial->ReadNonblocking(&buffer, 1, tmp);
        switch (serialRet) {
        case SERIAL_ERROR:
            RCLCPP_ERROR(node->get_logger(), "ReadNonblocking error");
            ret = false;
            exit = true;
            break;
        case SERIAL_TIMEOUT:
            RCLCPP_ERROR(node->get_logger(), "ReadNonblocking timeout");
            ret = false;
            exit = true;
            break;
        default: {
            ret = true;
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
                        RCLCPP_WARN(node->get_logger(), "Answer doesn't match");
                    }
                } else {
                    ++answerPosition;
                    if (answerPosition == expectedAnswerLength) {
                        exit = true;
                        matchingAnswer = true;
                    }
                }
            }
        } break;
        }
    }

    if (matchingAnswer == false) {
        RCLCPP_WARN(node->get_logger(),
            "Sent command:%s Answer text:%s", command, answerText.c_str());
    }

    return ret;
}
