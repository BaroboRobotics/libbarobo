//
// Copyright (c) Microsoft Corporation.  All rights reserved.
//
//
// Use of this source code is subject to the terms of the Microsoft end-user
// license agreement (EULA) under which you licensed this SOFTWARE PRODUCT.
// If you did not accept the terms of the EULA, you are not authorized to use
// this source code. For a copy of the EULA, please see the LICENSE.RTF on your
// install media.
//


#ifndef _COMUTILITY_
#define _COMUTILITY_

#pragma once
#include <winsock2.h>
#include <ws2bth.h>
#include <bthapi.h>
#include <bthutil.h>
#include <windows.h>
#include <string>
#include <tchar.h>

#define MAX_NAME_SIZE 128
#define MAX_ADDR_SIZE 15
#define MAX_MESSAGE_SIZE 256
#define MAX_NR_BYTES 64

using namespace std;


struct DeviceList
{ 
	BT_ADDR bthAddress;
	TCHAR bthName[40];
	DeviceList *NextDevice;
};

struct DeviceInfo 
{
	WCHAR szDeviceNameAddr[MAX_NAME_SIZE];

}; 

class BthUtils
{
	public:
		BthUtils();
		~BthUtils();
		int DiscoverDevices();
		int GetNumDevices(){return m_iNumDevices;};
		int GetDeviceInfo(DeviceInfo *pPeerDevicesInfo);
		int GetLocalDeviceName(DeviceInfo *pLocalDeviceInfo);
		int GetDeviceInfo(DeviceInfo *pPeerDeviceInfo, int iSelectedItem);
		int OpenServerConnection(BYTE *rgbSdpRecord, int cSdpRecord, int iChannelOffset, void (*funcPtr)( WCHAR*));
		int SendMessageToServer(WCHAR *strGUID, WCHAR *szMessage, int iSelectedDeviceIndex);


	private:
		DeviceList *m_pDeviceList, *m_pStart, *m_pEnd, *m_pCurrentDevice;
		int m_iNumDevices;
		void (*pCallBackFunction)( WCHAR* ) ;
		HANDLE m_hReadThread;
		SOCKET m_socketServer, m_socketClient;
		SOCKADDR_BTH m_saClient;
		DWORD m_dwBluetoothMode;

		int RegisterService(BYTE *rgbSdpRecord, int cSdpRecord, int iChannelOffset, UCHAR channel);
		int OpenClientConnection(WCHAR *strGUID, int iSelectedDeviceIndex);
		int GetGUID(WCHAR *psz, GUID *pGUID) ;

		static DWORD WINAPI ReadData(LPVOID voidArg);
};



class Serial{

	private:
		HANDLE handle;
		BYTE byte[MAX_NR_BYTES];
		DWORD nrBytes;
		COMMTIMEOUTS timeout;
		DCB dcb;
	
	public:
		Serial();
		char * receive(int length);
		int send(char *command, int bytes);
		int connect(wstring port);
		int connect(wstring port, int set_timeout, int bytesize, int baudrate, int parity);
		int disconnect();
		int flush();
};

#endif