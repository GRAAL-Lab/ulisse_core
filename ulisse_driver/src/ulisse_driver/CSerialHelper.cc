/*
 * CSerialHelper.cpp
 *
 *  Created on: May 28, 2010
 *      Author: asperi
 */

#include <iostream>
#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <ortos/ortos.h>

#include "ulisse_driver/CSerialHelper.h"

namespace ulisse {

std::map<std::string, CSerialHelper*> CSerialHelper::instances_;

CSerialHelper::CSerialHelper(const char *serialPort, int baudRate) {
	// Crea il riferimento unico alla sezione critica
	// per la condivisione di variabili tra threads
	pthread_mutex_init(&critSecSem_, NULL);

	this->opened_ = false;
	this->fd_ = open(serialPort, O_RDWR | O_SYNC);

	if (this->fd_ == -1) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "CSerialHelper", "Unable to open %s port!", serialPort);
		return;
	}

	tcgetattr(this->fd_, &this->oldtio_); // save current port settings

	switch (baudRate) {
	case 4800:
		cfsetispeed(&this->newtio_, B4800);
		cfsetospeed(&this->newtio_, B4800);
		break;
	case 9600:
		cfsetispeed(&this->newtio_, B9600);
		cfsetospeed(&this->newtio_, B9600);
		break;
	case 19200:
		cfsetispeed(&this->newtio_, B19200);
		cfsetospeed(&this->newtio_, B19200);
		break;
	case 38400:
		cfsetispeed(&this->newtio_, B38400);
		cfsetospeed(&this->newtio_, B38400);
		break;
	case 57600:
		cfsetispeed(&this->newtio_, B57600);
		cfsetospeed(&this->newtio_, B57600);
		break;
	case 115200:
		cfsetispeed(&this->newtio_, B115200);
		cfsetospeed(&this->newtio_, B115200);
		break;
	case 230400:
		cfsetispeed(&this->newtio_, B230400);
		cfsetospeed(&this->newtio_, B230400);
		break;
	default:
		fprintf(stderr, "Unsupported baudrate ( %d ), defaulting to 9600", baudRate);
		cfsetispeed(&this->newtio_, B9600);
		cfsetospeed(&this->newtio_, B9600);
		break;
	}

	// Data bits
	this->newtio_.c_cflag = (newtio_.c_cflag & ~CSIZE) | CS8;
	this->newtio_.c_cflag |= CLOCAL | CREAD;
	// Parity
	this->newtio_.c_cflag &= ~(PARENB | PARODD);
	this->newtio_.c_cflag &= ~CRTSCTS;
	// Stop Bits
	this->newtio_.c_cflag &= ~CSTOPB;
	this->newtio_.c_iflag = IGNBRK;
	this->newtio_.c_iflag &= ~(IXON | IXOFF | IXANY);
	this->newtio_.c_lflag = 0;
	this->newtio_.c_oflag = 0;

	this->newtio_.c_cc[VTIME] = 1;
	this->newtio_.c_cc[VMIN] = 1;
	this->newtio_.c_lflag &= ~(ECHONL | NOFLSH);
	this->newtio_.c_cflag &= ~CRTSCTS;

	tcflush(this->fd_, TCIFLUSH);
	tcsetattr(this->fd_, TCSANOW, &this->newtio_);

	this->opened_ = true;
}

CSerialHelper::~CSerialHelper() {
	close(fd_);
}

////////////////////////////////////////////////////////////////////////////////////////////
// NOTA IMPORTANTE: se usato in software multi-threaded e' necessario che la PRIMA chiamata
// a GetInstance sia fatta dal thread principale prima di creare eventuali altri thread,
// altrimenti senza meccanismi di locking delle risorse si potrebbero avere dei problemi
// di consistenza della variabile statica "instance_".
//
CSerialHelper *CSerialHelper::getInstance(void) {
	if (instances_.count(SERIAL_PORT_NAME) == 0) {
		CSerialHelper* tmp = new CSerialHelper(SERIAL_PORT_NAME, SERIAL_PORT_BAUDRATE);
		instances_[SERIAL_PORT_NAME] = tmp;

	}

	return instances_[SERIAL_PORT_NAME];
}

////////////////////////////////////////////////////////////////////////////////////////////
// NOTA IMPORTANTE: se usato in software multi-threaded e' necessario che la PRIMA chiamata
// a GetInstance sia fatta dal thread principale prima di creare eventuali altri thread,
// altrimenti senza meccanismi di locking delle risorse si potrebbero avere dei problemi
// di consistenza della variabile statica "instance_".
//
CSerialHelper *CSerialHelper::getInstance(const char *serialPort, int baudRate) {
	if (instances_.count(serialPort) == 0) {
		ortos::DebugConsole::Write(ortos::LogLevel::warning, "CSerialHelper::getInstance", "creating new instance %s %d", serialPort, baudRate);

		CSerialHelper* tmp = new CSerialHelper(serialPort, baudRate);
		instances_[serialPort] = tmp;

	} else {
		ortos::DebugConsole::Write(ortos::LogLevel::warning, "CSerialHelper::getInstance", "instance %s %d found", serialPort, baudRate);
	}

	return instances_[serialPort];
}

bool CSerialHelper::IsOpen(void) {
	bool ret = false;

	pthread_mutex_lock(&critSecSem_);
	{
		ret = this->opened_;
	}
	pthread_mutex_unlock(&critSecSem_);

	return ret;
}

void CSerialHelper::ChangeBaudRate(uint32_t baudRate) {

	pthread_mutex_lock(&critSecSem_);

	if (this->fd_ == -1) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "CSerialHelper::ChangeBaudRate", "Port is closed!");
		return;
	}

	tcgetattr(this->fd_, &this->oldtio_); // save current port settings

	switch (baudRate) {
	case 4800:
		cfsetispeed(&this->newtio_, B4800);
		cfsetospeed(&this->newtio_, B4800);
		break;
	case 9600:
		cfsetispeed(&this->newtio_, B9600);
		cfsetospeed(&this->newtio_, B9600);
		break;
	case 19200:
		cfsetispeed(&this->newtio_, B19200);
		cfsetospeed(&this->newtio_, B19200);
		break;
	case 38400:
		cfsetispeed(&this->newtio_, B38400);
		cfsetospeed(&this->newtio_, B38400);
		break;
	case 57600:
		cfsetispeed(&this->newtio_, B57600);
		cfsetospeed(&this->newtio_, B57600);
		break;
	case 115200:
		cfsetispeed(&this->newtio_, B115200);
		cfsetospeed(&this->newtio_, B115200);
		break;
	case 230400:
		cfsetispeed(&this->newtio_, B230400);
		cfsetospeed(&this->newtio_, B230400);
		break;
	default:
		ortos::DebugConsole::Write(ortos::LogLevel::warning, "CSerialHelper::ChangeBaudRate", "Unsupported baudrate ( %d ), defaulting to 9600", baudRate);
		cfsetispeed(&this->newtio_, B9600);
		cfsetospeed(&this->newtio_, B9600);
		break;
	}

	tcflush(this->fd_, TCIFLUSH);
	tcsetattr(this->fd_, TCSANOW, &this->newtio_);

	pthread_mutex_unlock(&critSecSem_);
}

int CSerialHelper::Write(const char *buffer, int size) {
	int ret = SERIAL_ERROR;
	int sent = 0;

	while (sent < size) {
		ret = write(this->fd_, &buffer[sent], size - sent);
		if (ret > 0) {
			fsync(this->fd_);
			sent += ret;
		} else if (ret == -1) {
			ortos::DebugConsole::Write(ortos::LogLevel::warning, "CSerialHelper::Write", "Error on serial writing");

			return SERIAL_ERROR;
		}
	}

	return ret;
}

int CSerialHelper::SetRTS(int level) {
	return SetLine(TIOCM_RTS, level);
}

int CSerialHelper::SetDTR(int level) {
	return SetLine(TIOCM_DTR, level);
}

int CSerialHelper::SetCD(int level) {
	return SetLine(TIOCM_CD, level);
}

int CSerialHelper::ReadBlocking(char *buffer, int size) {
	int ret = 0;
	int readed = 0;

	memset(buffer, 0x00, size);

	while ((readed < size) && (ret != -1)) {
		ret = read(this->fd_, &(buffer[readed]), (size - readed));
		if (ret != -1) {
			readed += ret;
		} else {
			ortos::DebugConsole::Write(ortos::LogLevel::warning, "CSerialHelper::ReadBlocking", "Error on serial reading");
			return SERIAL_ERROR;
		}
	}

	return ret;
}

int CSerialHelper::ReadLine(char *buffer, int size) {
	int ret = 0;
	int i = 0;
	char ch;
	bool endline = false;

	memset(buffer, 0x00, size);

	while ((endline == false) && (i < size)) {
		ret = read(this->fd_, &ch, 1);
		buffer[i] = ch;
		i++;
		if (ch == '\r') {
			ret = read(this->fd_, &ch, 1);
			buffer[i] = ch;
			i++;
			endline = true;
		}
	}

	return ret;
}

int CSerialHelper::ReadNonblocking(char *buffer, int size, struct timeval timeout) {
	int ret = 0;
	int readed = 0;
	fd_set input;

	/* Initialize the input set */
	FD_ZERO(&input);
	FD_SET(this->fd_, &input);

	memset(buffer, 0x00, size);

	while ((readed < size) && (ret != -1)) {
		/* Do the select */
		ret = select(this->fd_ + 1, &input, NULL, NULL, &timeout);

		/* See if there was an error */
		if (ret < 0) {
			ortos::DebugConsole::Write(ortos::LogLevel::warning, "CSerialHelper::ReadNonblocking", "Error on select call");
			return SERIAL_ERROR;
		} else if (ret == 0) {
			ortos::DebugConsole::Write(ortos::LogLevel::verbose, "CSerialHelper::ReadNonblocking", "Timeout");
			return SERIAL_TIMEOUT;
		}

		ret = read(this->fd_, &(buffer[readed]), (size - readed));
		if (ret != -1) {
			readed += ret;
		} else {
			ortos::DebugConsole::Write(ortos::LogLevel::warning, "CSerialHelper::ReadNonblocking", "Error on nonblocking read");
			return SERIAL_ERROR;
		}
	}

	return ret;
}

int CSerialHelper::WaitCDLineFallingSignal() {
	return WaitLineChange(SERIAL_FALLINGEDGE, TIOCM_CD);
}

int CSerialHelper::WaitCDLineRaisingSignal() {
	return WaitLineChange(SERIAL_RAISINGEDGE, TIOCM_CD);
}

int CSerialHelper::WaitDTRLineFallingSignal() {
	return WaitLineChange(SERIAL_FALLINGEDGE, TIOCM_DTR);
}

int CSerialHelper::WaitDTRLineRaisingSignal() {
	return WaitLineChange(SERIAL_RAISINGEDGE, TIOCM_DTR);
}

int CSerialHelper::WaitDSRLineFallingSignal() {
	return WaitLineChange(SERIAL_FALLINGEDGE, TIOCM_DSR);
}

int CSerialHelper::WaitDSRLineRaisingSignal() {
	return WaitLineChange(SERIAL_RAISINGEDGE, TIOCM_DSR);
}

int CSerialHelper::WaitRTSLineFallingSignal() {
	return WaitLineChange(SERIAL_FALLINGEDGE, TIOCM_RTS);
}

int CSerialHelper::WaitRTSLineRaisingSignal() {
	return WaitLineChange(SERIAL_RAISINGEDGE, TIOCM_RTS);
}

int CSerialHelper::WaitCTSLineFallingSignal() {
	return WaitLineChange(SERIAL_FALLINGEDGE, TIOCM_CTS);
}

int CSerialHelper::WaitCTSLineRaisingSignal() {
	return WaitLineChange(SERIAL_RAISINGEDGE, TIOCM_CTS);
}

int CSerialHelper::WaitLineChange(int edgeType, int ctrlSignal) {
	unsigned int status, oldstate, newstate, changed, mask;
	/* To wait for *any* of the 4 control lines to change state,
	 * use this instead of the line below:
	 * mask = TIOCM_CAR | TIOCM_DSR | TIOCM_CTS | TIOCM_RNG;
	 */
	mask = ctrlSignal;

	/* Get current status of the control lines in mask */
	if (ioctl(fd_, TIOCMGET, &status) == -1) {
		return SERIAL_ERROR;
	}
	oldstate = status & mask; /* Save current status of control lines */

	/* The ioctl() call below sleeps, and does not return until one
	 * of the  serial control lines specified in mask changes state,
	 * or it was interrupted by some signal.
	 */
	rewait: /* Goto-label for when we get a signal. */

	if (ioctl(fd_, TIOCMIWAIT, mask) == -1) {
		if (errno == EINTR) {
			/* Signal received. Handle this case. Here we're
			 * just restarting the ioctl-wait call.
			 */
			goto rewait;
		} else {
			/* Error condition */
			return SERIAL_ERROR;
		}
	}

	/* Get new status of the control lines in mask */
	if (ioctl(fd_, TIOCMGET, &status) == -1) {
		return SERIAL_ERROR;
	}
	newstate = status & mask;

	/* XOR old with new status to find the one(s) that changed */
	changed = newstate ^ oldstate;

	/* Do something with the result of al this */
	if (changed & mask) { /* RNG line has changed */
		if (newstate & mask) {
			if (edgeType == SERIAL_RAISINGEDGE) {
//				fprintf(stderr,"PPS raising edge Detected\n");//DEBUG
				return SERIAL_OK;
			}
		} else {
			if (edgeType == SERIAL_FALLINGEDGE) {
//				fprintf(stderr,"PPS falling edge Detected\n");//DEBUG
				return SERIAL_OK;
			}
		}

		oldstate = newstate;
//		fprintf(stderr,"goto rewait (wrong edge)\n");//DEBUG
		goto rewait;
	}
	return 0;
}

int CSerialHelper::SetLine(int ctrlSignal, int level) {
	int status;

	if (ioctl(this->fd_, TIOCMGET, &status) == -1) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "CSerialHelper::SetLine", "error on TIOCMGET");
		return SERIAL_ERROR;
	}

	if (level)
		status |= ctrlSignal;
	else
		status &= ~ctrlSignal;
	if (ioctl(this->fd_, TIOCMSET, &status) == -1) {
		ortos::DebugConsole::Write(ortos::LogLevel::error, "CSerialHelper::SetLine", "error on TIOCMSET");
		return SERIAL_ERROR;
	}

	return SERIAL_OK;
}

} //namespace ulisse

