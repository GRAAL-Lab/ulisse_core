/*
 * taskEESMessagePrinter.cc
 *
 *  Created on: Jun 16, 2016
 *      Author: wonder
 */

#include <stdio.h>
#include <iostream>
#include <ortos/ortos.h>
#include <libconfig.h++>

#include "EESHelper.h"
#include "Functions.h"

using namespace om2ctrl::ees;

struct threadDataViewerData {
	std::string configFile;
};

void* ThreadDataInViewerFunction(void* dataIn) {
	int period = 100;
	bool printSensors = false;
	bool printStatus = false;
	bool printConfig = false;
	bool printMotors = false;
	bool printVersion = false;
	bool printAck = false;
	bool printBattery = false;
	bool printSw485Status = false;


	int sensorsPeriodLoops = 1;
	int statusPeriodLoops = 1;
	int configPeriodLoops = 1;
	int motorsPeriodLoops = 1;
	int versionPeriodLoops = 1;
	int ackPeriodLoops = 1;
	int batteryPeriodLoops = 1;
	int sw485StatusPeriodLoops = 1;


	libconfig::Config confParser;
	threadDataViewerData* dataInData = (threadDataViewerData*)dataIn;

	try {
		confParser.readFile(dataInData->configFile.c_str());
	} catch (const libconfig::FileIOException &fioex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "I/O error reading file. %s", dataInData->configFile.c_str());
#endif
	} catch (libconfig::ParseException& pex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Parse exception when reading %s", dataInData->configFile.c_str());
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Line: %d error: %s", pex.getLine(), pex.getError());
#endif
	}

	try {
		period = confParser.lookup("EESDataViewer.In.period");

		printSensors = confParser.lookup("EESDataViewer.In.sensors.view");
		sensorsPeriodLoops = confParser.lookup("EESDataViewer.In.sensors.periodLoops");

		printStatus = confParser.lookup("EESDataViewer.In.status.view");
		statusPeriodLoops = confParser.lookup("EESDataViewer.In.status.periodLoops");

		printConfig = confParser.lookup("EESDataViewer.In.config.view");
		configPeriodLoops = confParser.lookup("EESDataViewer.In.config.periodLoops");

		printMotors = confParser.lookup("EESDataViewer.In.motors.view");
		motorsPeriodLoops = confParser.lookup("EESDataViewer.In.motors.periodLoops");

		printVersion = confParser.lookup("EESDataViewer.In.version.view");
		versionPeriodLoops = confParser.lookup("EESDataViewer.In.version.periodLoops");

		printAck = confParser.lookup("EESDataViewer.In.ack.view");
		ackPeriodLoops = confParser.lookup("EESDataViewer.In.ack.periodLoops");

		printBattery = confParser.lookup("EESDataViewer.In.battery.view");
		batteryPeriodLoops = confParser.lookup("EESDataViewer.In.battery.periodLoops");

		printSw485Status = confParser.lookup("EESDataViewer.In.sw485Status.view");
		sw485StatusPeriodLoops = confParser.lookup("EESDataViewer.In.sw485Status.periodLoops");

	} catch (libconfig::SettingException& sex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Setting Exception: path = %s, what = %s", sex.getPath(), sex.what());
#endif
	} catch (libconfig::FileIOException& fioex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "FileIO Exception");
#endif
	} catch (libconfig::ConfigException& cex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Config Exception: what = %s", cex.what());
#endif
	}

	int ret;
	ortos::Task* task = ortos::Task::GetInstance();
	task->SetType(ortos::TaskType::user);
	task->SetSampleTime(ortos::constants::oneMillisecond * period);
	ret = task->CreateSync(om2ctrl::tasknames::EESMessagePrinterDataInThreadName);
	ortos::DebugConsole::Write(ortos::LogLevel::info, "thread", "Thread created!");
	if (ret != ORTOS_RV_OK) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "Error creating the thread!");
		task->Exit();
	}

	ortos::xcom::XCOMInterface* xcom = ortos::xcom::XCOMInterface::GetInstance();

	SensorsContainer sensors;
	StatusContainer status;
	ConfigContainer config;
	MotorsContainer motors;
	VersionContainer version;
	AckContainer ack;
	BatteryContainer battery;
	Sw485StatusContainer sw485Status;

	ortos::ortostime_t lastValidSensors = 0;
	ortos::ortostime_t lastValidStatus = 0;
	ortos::ortostime_t lastValidConfig = 0;
	ortos::ortostime_t lastValidMotors = 0;
	ortos::ortostime_t lastValidVersion = 0;
	ortos::ortostime_t lastValidAck = 0;
	ortos::ortostime_t lastValidBattery = 0;
	ortos::ortostime_t lastValidSw485Status = 0;

	uint64_t loops = 0;

	while (task->Continue()) {
		xcom->Synchronize();
		if (printSensors) {
			ret = xcom->ReadIf(topicnames::sensors, sensors);
			if (ret == ORTOS_RV_OK) {
				if (sensorsPeriodLoops > 0) {
					if (loops % sensorsPeriodLoops == 0) {
						if (::om2ctrl::utils::CheckNewData(lastValidSensors, sensors.timestamp)) {
							sensors.d.DebugPrint("IN: ");
						}
					}
				}
			} else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "An error occurred while reading sensors");
			}
		}

		if (printStatus) {
			ret = xcom->ReadIf(topicnames::status, status);
			if (ret == ORTOS_RV_OK) {
				if (statusPeriodLoops > 0) {
					if (loops % statusPeriodLoops == 0) {
						if (::om2ctrl::utils::CheckNewData(lastValidStatus, status.timestamp)) {
							status.d.DebugPrint("IN: ");
						}
					}
				}
			} else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "An error occurred while reading status");
			}
		}

		if (printConfig) {
			ret = xcom->ReadIf(topicnames::config, config);
			if (ret == ORTOS_RV_OK) {
				if (configPeriodLoops > 0) {
					if (loops % configPeriodLoops == 0) {
						if (::om2ctrl::utils::CheckNewData(lastValidConfig, config.timestamp)) {
							config.d.DebugPrint("IN: ");
						}
					}
				}
			} else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "An error occurred while reading config");
			}
		}

		if (printMotors) {
			ret = xcom->ReadIf(topicnames::motors, motors);
			if (ret == ORTOS_RV_OK) {
				if (motorsPeriodLoops > 0) {
					if (loops % motorsPeriodLoops == 0) {
						if (::om2ctrl::utils::CheckNewData(lastValidMotors, motors.timestamp)) {
							motors.d.DebugPrint("IN: ");
						}
					}
				}
			} else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "An error occurred while reading motors");
			}
		}

		if (printVersion) {
			ret = xcom->ReadIf(topicnames::version, version);
			if (ret == ORTOS_RV_OK) {
				if (versionPeriodLoops > 0) {
					if (loops % versionPeriodLoops == 0) {
						if (::om2ctrl::utils::CheckNewData(lastValidVersion, version.timestamp)) {
							version.d.DebugPrint("IN: ");
						}
					}
				}
			} else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "An error occurred while reading version");
			}
		}

		if (printAck) {
			ret = xcom->ReadIf(topicnames::ack, ack);
			if (ret == ORTOS_RV_OK) {
				if (ackPeriodLoops > 0) {
					if (loops % ackPeriodLoops == 0) {
						if (::om2ctrl::utils::CheckNewData(lastValidAck, ack.timestamp)) {
							ack.d.DebugPrint("IN: ");
						}
					}
				}
			} else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "An error occurred while reading ack");
			}
		}

		if (printBattery) {
			ret = xcom->ReadIf(topicnames::battery, battery);
			if (ret == ORTOS_RV_OK) {
				if (batteryPeriodLoops > 0) {
					if (loops % batteryPeriodLoops == 0) {
						if (::om2ctrl::utils::CheckNewData(lastValidBattery, battery.timestamp)) {
							battery.d.DebugPrint("IN: ");
						}
					}
				}
			} else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "An error occurred while reading ack");
			}
		}

		if (printSw485Status) {
			ret = xcom->ReadIf(topicnames::sw485Status, sw485Status);
			if (ret == ORTOS_RV_OK) {
				if (sw485StatusPeriodLoops > 0) {
					if (loops % sw485StatusPeriodLoops == 0) {
						if (::om2ctrl::utils::CheckNewData(lastValidSw485Status, sw485Status.timestamp)) {
							sw485Status.d.DebugPrint("IN: ");
						}
					}
				}
			} else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "An error occurred while reading ack");
			}
		}

		++loops;
		task->WaitPeriod();
	}

	xcom->Release();
	xcom->ReleaseInstance();

	ortos::Thread::Exit();
	return NULL;
}

void* ThreadDataOutViewerFunction(void* dataIn) {
	int period = 100;

	bool printReferences = false;
	bool printCommands = false;

	int referencesLoops = 0;

	ortos::ortostime_t lastValidReferences = -1;

	libconfig::Config confParser;
	threadDataViewerData* dataInData = (threadDataViewerData*)dataIn;

	try {
		confParser.readFile(dataInData->configFile.c_str());
	} catch (const libconfig::FileIOException &fioex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "I/O error reading file. %s", dataInData->configFile.c_str());
#endif
	} catch (libconfig::ParseException& pex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Parse exception when reading %s", dataInData->configFile.c_str());
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Line: %d error: %s", pex.getLine(), pex.getError());
#endif
	}

	try {
		period = confParser.lookup("EESDataViewer.Out.period");

		printReferences = confParser.lookup("EESDataViewer.Out.references.view");
		referencesLoops = confParser.lookup("EESDataViewer.Out.references.periodLoops");

		printCommands = confParser.lookup("EESDataViewer.Out.commands.view");
	} catch (libconfig::SettingException& sex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Setting Exception: path = %s, what = %s", sex.getPath(), sex.what());
#endif
	} catch (libconfig::FileIOException& fioex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "FileIO Exception");
#endif
	} catch (libconfig::ConfigException& cex) {
#if LOG_LEVEL >= LOG_LEVEL_ERROR
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Config Exception: what = %s", cex.what());
#endif
	}

	int ret;
	ortos::Task* task = ortos::Task::GetInstance();
	task->SetType(ortos::TaskType::user);
	task->SetSampleTime(ortos::constants::oneMillisecond * period);
	ret = task->CreateSync(om2ctrl::tasknames::EESMessagePrinterDataOutThreadName);
	ortos::DebugConsole::Write(ortos::LogLevel::info, "thread", "Thread created!");
	if (ret != ORTOS_RV_OK) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "Error creating the thread!");
		task->Exit();
	}

	ortos::xcom::XCOMInterface* xcom = ortos::xcom::XCOMInterface::GetInstance();

	ReferencesContainer references;
	CommandContainer command;
	CommandAnswerContainer commandAnswer;

	uint64_t loops = 0;

	if (printCommands) {
		ret = xcom->Subscribe(topicnames::eescommands, command);
		if (ret != ORTOS_RV_OK) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Error while subscribing to the commands topic %s", topicnames::eescommands);
			printCommands = false;
		}

		ret = xcom->Subscribe(topicnames::eeslogCommandAnswers, commandAnswer);
		if (ret != ORTOS_RV_OK) {
			ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Error while subscribing to the command answers topic %s", topicnames::eeslogCommandAnswers);
			printCommands = false;
		}
	}

	while (task->Continue()) {
		xcom->Synchronize();

		if (printReferences) {
			ret = xcom->ReadIf(topicnames::references, references);
			if (ret == ORTOS_RV_OK) {
				if (referencesLoops > 0) {
					if (loops % referencesLoops == 0) {
						if (::om2ctrl::utils::CheckNewData(lastValidReferences, references.timestamp)) {
							references.d.DebugPrint("OUT: ");
						}
					}
				}
			} else if ((ret != ORTOS_RV_ENOBLOCK) && (ret != ORTOS_RV_ENONEWVALUE)) {
				ortos::DebugConsole::Write(ortos::LogLevel::error, "thread", "An error occurred while reading references");
			}
		}

		if (printCommands) {
			while(xcom->ReadIf(topicnames::eescommands, command) == ORTOS_RV_OK) {
				command.DebugPrint("IN");
			}

			while(xcom->ReadIf(topicnames::eeslogCommandAnswers, commandAnswer) == ORTOS_RV_OK) {
				commandAnswer.DebugPrint("OUT");
			}
		}

		++loops;
		task->WaitPeriod();
	}

	xcom->Release();
	xcom->ReleaseInstance();

	ortos::Thread::Exit();
	return NULL;
}

int main (int argc, char *argv[]) {

	int ret;
	threadDataViewerData dataViewerData;
	dataViewerData.configFile = om2ctrl::constants::defaultConfig;

	for (int i = 1; i < argc; i++) {
		if ((strcmp(argv[i], "--cfg") == 0) && (i + 1 < argc)) {
			dataViewerData.configFile.assign(argv[i + 1]);
		}

		if ((strcmp(argv[i], "--help") == 0) || (strcmp(argv[i], "-h") == 0)) {
			ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Usage: %s [options]", argv[0]);
			ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Options");
			ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "--cfg file\t: 'file' is the config file (default %s)",
					om2ctrl::constants::defaultConfig);
			return 0;
		}
	}

	ortos::Task* task = ortos::Task::GetInstance();
	task->SetType(ortos::TaskType::user);
	ret = task->CreateAsync(om2ctrl::tasknames::EESMessagePrinterTaskName);
	ortos::DebugConsole::Write(ortos::LogLevel::info, "main", "Task created");
	if (ret != ORTOS_RV_OK) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "main", "Error creating the task");

		task->Exit();
	}

	ortos::Thread ttestdataout;
	ttestdataout.Create(ThreadDataOutViewerFunction, 20000000, (void*)&dataViewerData);

	ortos::Thread ttestdatain;
	ttestdatain.Create(ThreadDataInViewerFunction, 20000000, (void*)&dataViewerData);

//	ttestinput.Join();
	ttestdatain.Join();
	ttestdataout.Join();

	task->Exit();
	return 0;
}




