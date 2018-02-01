#include"MySerial.h"
MySerial::MySerial(int COMnum) {
	using namespace std;
//open COMport
	string port_name = "COM" + to_string(COMnum);
	hComm = CreateFile(port_name.c_str(), GENERIC_READ | GENERIC_WRITE,
		0, NULL, OPEN_EXISTING, 0, NULL);
	if (hComm == INVALID_HANDLE_VALUE) {
		cout << "error in opening serial port" << endl;
	}
	else {
		cout << "opening serial port successful" << endl;
	}
	
//configure some paramaters
	dcbSerialParams = { 0 };
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(hComm, &dcbSerialParams);
	dcbSerialParams.BaudRate = CBR_9600;
	dcbSerialParams.ByteSize = 8;
	dcbSerialParams.StopBits = ONESTOPBIT;
	dcbSerialParams.Parity = NOPARITY;
	SetCommState(hComm, &dcbSerialParams);

//setting timeouts
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;
}

void MySerial::sendData(unsigned char data) {
	using namespace std;
	char lpBuffer[sizeof(data)];
	*(unsigned*)lpBuffer = data;

	dNoOfBytesWritten = 0;
	dNoOfBytestoWrite = sizeof(lpBuffer);

	Status = WriteFile(hComm,
		lpBuffer,
		dNoOfBytestoWrite,
		&dNoOfBytesWritten,
		NULL);
	cout << dNoOfBytesWritten << endl;
}
void MySerial::sendData(int data) {
	using namespace std;
	char lpBuffer[sizeof(data)];
	*(int*)lpBuffer = data;

	dNoOfBytesWritten = 0;
	dNoOfBytestoWrite = sizeof(lpBuffer);

	Status = WriteFile(hComm,
		lpBuffer,
		dNoOfBytestoWrite,
		&dNoOfBytesWritten,
		NULL);
	cout << dNoOfBytesWritten << endl;
}
void MySerial::recieveData(char SerialBuffer[]) {
	//setting WaiComm Event
	Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception
	if (Status == FALSE) printf("\n\n    Error! in Setting CommMask");
	else printf("\n\n    Setting CommMask successfull");

	//Wait for the character to be received
	printf("\n\n    Waiting for Data Reception");
	DWORD dwEventMask;
	Status = WaitCommEvent(hComm, &dwEventMask, NULL);
	if (Status == FALSE) printf("\n    Error! in Setting WaitCommEvent()");

	char TempChar; //Temporary character used for reading
	DWORD NoBytesRead;
	int i = 0;
	do
	{
		ReadFile(hComm,           //Handle of the Serial port
			&TempChar,       //Temporary character
			sizeof(TempChar),//Size of TempChar
			&NoBytesRead,    //Number of bytes read
			NULL);
		SerialBuffer[i] = TempChar;// Store Tempchar into buffer
		i++;
	} while (NoBytesRead > 0);
}

MySerial::~MySerial() {
	CloseHandle(hComm);
}