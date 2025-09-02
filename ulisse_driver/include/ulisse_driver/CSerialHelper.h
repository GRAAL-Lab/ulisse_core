/*
 * CSerialHelper.h
 *
 *  Created on: May 28, 2010
 *      Author: asperi
 */

#ifndef CSERIALHELPER_H_
#define CSERIALHELPER_H_

#define SERIAL_PORT_NAME            ("/dev/ttyUSB0")
#define SERIAL_PORT_BAUDRATE        (19200)
#define SERIAL_ERROR				(-1)
#define SERIAL_TIMEOUT				(0)
#define SERIAL_OK				(1)

#define SERIAL_RAISINGEDGE      (1)
#define SERIAL_FALLINGEDGE      (2)

#include <memory>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <map>
#include <cstring>
#include <string>

namespace ulisse {

class CSerialHelper {
private:

	static std::map<std::string, CSerialHelper*> instances_;

	pthread_mutex_t critSecSem_;

	int fd_;
	struct termios oldtio_;
	struct termios newtio_;
	bool opened_;

protected:
	//CSerialHelper();
	CSerialHelper(const char *serialPort, int baudRate);
	int WaitLineChange(int edgeType, int ctrlSignal);
	int SetLine(int ctrlSignal, int level);

public:
	~CSerialHelper();
	static CSerialHelper *getInstance(void);
    static CSerialHelper *getInstance(const char *serialPort, int baudRate = SERIAL_PORT_BAUDRATE);

	bool IsOpen(void);

	void ChangeBaudRate(uint32_t baudRate);

    ssize_t Write(const unsigned char *buffer, size_t size);
	ssize_t Write(const char *buffer, size_t size);
	int SetRTS(int level);
	int SetDTR(int level);
	int SetCD(int level);
	int ReadBlocking(char *buffer, int size);
	int ReadLine(char *buffer, int size);
	int ReadNonBlocking(char *buffer, int size, struct timeval timeout);

	int WaitCDLineRaisingSignal();
	int WaitCDLineFallingSignal();
	int WaitDTRLineRaisingSignal();
	int WaitDTRLineFallingSignal();
	int WaitDSRLineRaisingSignal();
	int WaitDSRLineFallingSignal();
	int WaitRTSLineRaisingSignal();
	int WaitRTSLineFallingSignal();
	int WaitCTSLineRaisingSignal();
	int WaitCTSLineFallingSignal();
};

} //namespace ulisse

#endif /* CSERIALHELPER_H_ */
