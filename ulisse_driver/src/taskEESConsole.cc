/*
 * taskEESConsole.cc
 *
 *  Created on: Jul 6, 2016
 *      Author: wonder
 */

#include <stdio.h>
#include <iostream>
#include <ortos/ortos.h>

#include "EESHelper.h"
#include "CommandHelper.h"

using namespace om2ctrl::ees;

int main(int argc, char *argv[]) {
	ortos::Task* task = ortos::Task::GetInstance();
	task->SetType(ortos::TaskType::user);
	int ret = task->CreateAsync(om2ctrl::tasknames::EESConsoleTaskName);
	ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Task created");
	if (ret != ORTOS_RV_OK) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main",
				"Error creating the task");

		task->Exit();
	}

	ortos::xcom::XCOMInterface* xcom = ortos::xcom::XCOMInterface::GetInstance();

	ReferencesContainer references;

	om2ctrl::utils::CommandHelper commandHelper;
	CommandContainer command;
	CommandAnswerContainer commandAnswer;

	ret = commandHelper.RegisterAsCommandProducer(topicnames::eescommands, command, commandAnswer, om2ctrl::tasknames::EESConsoleTaskName);
	if (ret != ORTOS_RV_OK)
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "RegisterAsCommandProducer returned %d", ret);

	int choice;
	bool send, repeat;
	int repetitions, delayMs;

	while (task->Continue()) {

		std::cout << "1 ref send" << std::endl;
		std::cout << "2 beep" << std::endl;
		std::cout << "3 enable ref" << std::endl;
		std::cout << "4 reload config" << std::endl;
		std::cout << "5 get version" << std::endl;
		std::cout << "6 start compass calibration" << std::endl;
		std::cout << "7 stop compass calibration" << std::endl;
		std::cout << "8 reset" << std::endl;
		std::cout << "9 get config" << std::endl;

		std::cout << "10 set pumps" << std::endl;
		std::cout << "11 set pwr buttons" << std::endl;

		std::cin >> choice;

		if (std::cin.fail()) {
			std::cout << "Flushing bad input!" << std::endl;
			std::cin.clear(); // unset failbit
			std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			continue;
		}

		send = true;
		repeat = false;
		repetitions = 0;

		switch (choice) {
		case 1: {
			std::cout << "left thruster (%): ";
			std::cin >> references.d.leftThruster;
			references.d.leftThruster *= 10;
			std::cout << "right thruster (%): ";
			std::cin >> references.d.rightThruster;
			references.d.rightThruster *= 10;
			std::cout << "speed: ";
			std::cin >> references.d.speed;
			std::cout << "jog: ";
			std::cin >> references.d.jog;

			std::cout << "repeat? (1 yes, 0 no): ";
			std::cin >> repeat;
			if (repeat) {
				std::cout << "repetitions: ";
				std::cin >> repetitions;
				std::cout << "delay (ms): ";
				std::cin >> delayMs;
			}

			do {
			references.timestamp = ::om2ctrl::utils::GetTime();
			xcom->Synchronize();
			ret = xcom->WriteIf(topicnames::references, references);
			if (ret != ORTOS_RV_OK) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "main",
						"Error writing topic %s (%d)", topicnames::references, ret);
			} else {
				references.d.DebugPrint("Sent");
			}
				if (--repetitions > 0) {
					usleep(delayMs*1000);
				} else
					repeat = false;

			} while (repeat);
		}
			break;
		case 2: {
			uint16_t numberOfBeeps;
			uint16_t loop;
			std::cout << "number of beeps: ";
			std::cin >> numberOfBeeps;
			command.d.beep.numberOfBeeps = numberOfBeeps;
			std::cout << "loop: ";
			std::cin >> loop;
			command.d.beep.loop = loop;
			std::cout << "delay: ";
			std::cin >> command.d.beep.delay;

			command.commandType = (uint16_t)CommandType::beep;
		}
			break;
		case 3: {
			uint16_t enable;
			std::cout << "enable: ";
			std::cin >> enable;
			command.d.enableRef.enable = enable;

			command.commandType = (uint16_t)CommandType::enableref;
		}
			break;
		case 4: {
			command.commandType = (uint16_t)CommandType::reloadconfig;
		}
			break;
		case 5: {
			command.commandType = (uint16_t)CommandType::getversion;
		}
			break;
		case 6: {
			command.commandType = (uint16_t)CommandType::startcompasscal;
		}
			break;
		case 7: {
			command.commandType = (uint16_t)CommandType::stopcompasscal;
		}
			break;
		case 8: {
			command.commandType = (uint16_t)CommandType::reset;
		}
			break;
		case 9: {
			command.commandType = (uint16_t)CommandType::getconfig;
		}
			break;
		case 10: {
			std::cout << "0: stop all\n1: left\n2: right\n3: left+right\n";
			int posizione, azione, flagaction;
			std::cin >> posizione;

			if ((posizione > 0) && (posizione <=3)) {
				std::cout << "0: stop\n1: load bow\n2: load stern\n3: unload bow\n4: unload stern\n";
				std::cin >> azione;

				switch (azione) {
				case 0:
					flagaction = 0;
					break;
				case 1:
					flagaction = EMB_PUMPS_FLAG_BOWLOADWATER;
					break;
				case 2:
					flagaction = EMB_PUMPS_FLAG_STERNLOADWATER;
					break;
				case 3:
					flagaction = EMB_PUMPS_FLAG_BOWUNLOADWATER;
					break;
				case 4:
					flagaction = EMB_PUMPS_FLAG_STERNUNLOADWATER;
					break;
				default:
					flagaction = 0;
					break;
				}
			} else {
				flagaction = 0;
			}

			switch (posizione) {
			case 0:
				command.d.pumps.pumpsFlag[EMB_PUMPS_LEFT_IDX] = 0;
				command.d.pumps.pumpsFlag[EMB_PUMPS_RIGHT_IDX] = 0;
				break;
			case 1:
				command.d.pumps.pumpsFlag[EMB_PUMPS_LEFT_IDX] = flagaction;
				command.d.pumps.pumpsFlag[EMB_PUMPS_RIGHT_IDX] = 0;
				break;
			case 2:
				command.d.pumps.pumpsFlag[EMB_PUMPS_LEFT_IDX] = 0;
				command.d.pumps.pumpsFlag[EMB_PUMPS_RIGHT_IDX] = flagaction;
				break;
			case 3:
				command.d.pumps.pumpsFlag[EMB_PUMPS_LEFT_IDX] = flagaction;
				command.d.pumps.pumpsFlag[EMB_PUMPS_RIGHT_IDX] = flagaction;
				break;
			default:
				command.d.pumps.pumpsFlag[EMB_PUMPS_LEFT_IDX] = 0;
				command.d.pumps.pumpsFlag[EMB_PUMPS_RIGHT_IDX] = 0;
				break;
			}

			std::cout << "repeat? (1 yes, 0 no): ";
			std::cin >> repeat;
			if (repeat) {
				std::cout << "repetitions: ";
				std::cin >> repetitions;
				std::cout << "delay (ms): ";
				std::cin >> delayMs;
			}

			command.commandType = (uint16_t)CommandType::setpumps;
		}
			break;
		case 11: {
			std::cout << "1: left\n2: right\n3: left+right\n";
			int scelta;
			std::cin >> scelta;

			switch (scelta) {
			case 1:
				command.d.powerButtons.pwrButtonsFlag = EMB_PWRBUTTONS_FLAG_LEFT;
				break;
			case 2:
				command.d.powerButtons.pwrButtonsFlag = EMB_PWRBUTTONS_FLAG_RIGHT;
				break;
			case 3:
				command.d.powerButtons.pwrButtonsFlag = EMB_PWRBUTTONS_FLAG_LEFT | EMB_PWRBUTTONS_FLAG_RIGHT;
				break;
			default:
				command.d.powerButtons.pwrButtonsFlag = 0;
				break;
			}

			command.commandType = (uint16_t)CommandType::setpowerbuttons;
		}
			break;
		default:
			std::cout << "Unsupported choice!" << std::endl;
			continue;
			break;
		}

		if (send) {
			do {
				ret = commandHelper.SendCommand(command);
				if (ret == ORTOS_RV_OK) {
					command.DebugPrint("Sent");

					ret = commandHelper.WaitAnswer(commandAnswer);
					if (ret != ORTOS_RV_OK)
							ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "WaitAnswer returned %d", ret);

				} else {
					ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "SendCommand returned %d", ret);
				}

				if (--repetitions > 0) {
					usleep(delayMs*1000);
				} else
					repeat = false;

			} while (repeat);
		}
	}

	xcom->Release();
	xcom->ReleaseInstance();

	task->Exit();
	return 0;
}
