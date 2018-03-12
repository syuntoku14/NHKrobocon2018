#ifndef MYSERIAL
#define MYSERIAL
#include<windows.h>
#include<string>
#include<iostream>

class MySerial{
private:
	HANDLE hComm;
	DCB dcbSerialParams;
	COMMTIMEOUTS timeouts;
	DWORD dNoOfBytestoWrite;
	DWORD dNoOfBytesWritten;
	int Status;

public:
	MySerial(int COMnum, bool timeout_flag);

	void sendData(char data);
	void sendData(int data);
	void recieveData(char SerialBuffer[]);
	~MySerial() {
		CloseHandle(hComm);
	}
};

#endif