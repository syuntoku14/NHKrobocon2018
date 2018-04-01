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
	//if (timeout_flag) {
	//	timeouts = { 0 };
	//	timeouts.ReadTotalTimeoutConstant = 1000;
	//	timeouts.ReadTotalTimeoutMultiplier = 50;
	//	timeouts.WriteTotalTimeoutConstant = 1000;
	//	timeouts.WriteTotalTimeoutMultiplier = 50;
	//	SetCommTimeouts(hComm, &timeouts);
	//}
}

void MySerial::sendData(char data) {
	using namespace std;
	char lpBuffer[sizeof(data)];
	*(char*)lpBuffer = data;

	dNoOfBytesWritten = 0;
	dNoOfBytestoWrite = sizeof(lpBuffer);

	Status = WriteFile(hComm,
		lpBuffer,
		dNoOfBytestoWrite,
		&dNoOfBytesWritten,
		NULL);
	//cout << dNoOfBytesWritten << endl;
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
	//cout << dNoOfBytesWritten << endl;
}
void MySerial::recieveData(char SerialBuffer[]) {
	//GetCommTimeouts(hComm, &timeouts)

	//setting WaiComm Event
	Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception
											//if (Status == FALSE) printf("\n\n    Error! in Setting CommMask");
											//else printf("\n\n    Setting CommMask successfull");

											//Wait for the character to be received
											//printf("\n\n    Waiting for Data Reception\n");
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
		//std::cout << NoBytesRead <<TempChar<< std::endl;
	} while (NoBytesRead > 0);

}
