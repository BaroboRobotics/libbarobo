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


#include "stdafx.h"
#include "comutility.h"
//#include "mobotlib.h"
#include "tchar.h"
#include <windows.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <stdlib.h>



//Function: ReadThread
//Purpose: Receives messages from the peer device

DWORD WINAPI BthUtils::ReadData(LPVOID voidArg) 
	{
		int iSize=0, cbBytesRecd=0 ;
		WCHAR szMessage[MAX_MESSAGE_SIZE];
		char pbuf[MAX_MESSAGE_SIZE];
		BthUtils *functionInfo = (BthUtils*) voidArg;
		iSize = sizeof(functionInfo->m_saClient);
		BTH_REMOTE_NAME remotename;
		memset(&remotename, sizeof(remotename), 0);

		SOCKET s = accept (functionInfo->m_socketServer, (SOCKADDR*)&(functionInfo->m_saClient), &iSize);
		remotename.bt = functionInfo->m_saClient.btAddr;

		setsockopt(functionInfo->m_socketServer,SOL_RFCOMM, SO_BTH_SET_READ_REMOTE_NAME, (char*)&remotename, sizeof(remotename));
		
		if (s != INVALID_SOCKET) 
		{
			for ( ; ; ) 
			{
				//receive data in pbuf
				cbBytesRecd = recv (s, pbuf, MAX_MESSAGE_SIZE, 0);
				//if error occured in receiving, return error code
				if (cbBytesRecd == SOCKET_ERROR) 
				{
					return WSAGetLastError();
				}
				else
				{
					// something was received, then copy the contents in szMessage
					if(cbBytesRecd>0)
					{
        				StringCchPrintf (szMessage, sizeof(szMessage), L"%s", pbuf);
						(*functionInfo->pCallBackFunction)(szMessage);
					}
				}
			}
		}
		return 0;
	}
//Function: BthUtils (constructor)
//Purpose:	Initialize: Winsock and class data members
//				Turn on Bluetooth and set it discoverable mode, if not on already
				

BthUtils::BthUtils()
{
	WORD wVersionRequested;
	WSADATA wsaData;
	wVersionRequested = MAKEWORD( 2, 2 );
	WSAStartup( wVersionRequested, &wsaData );

	m_pDeviceList			= NULL;
	m_pStart				= NULL;
	m_pEnd					= NULL;
	m_pCurrentDevice		= NULL;
	m_iNumDevices			= 0;
	pCallBackFunction		= NULL;
	m_hReadThread			= NULL;
	m_socketServer			= INVALID_SOCKET;
	m_socketClient			= INVALID_SOCKET;
	BthGetMode(&m_dwBluetoothMode);
	if(m_dwBluetoothMode==BTH_POWER_OFF)
	{
		BthSetMode(BTH_DISCOVERABLE);
	}
}

//Function: ~BthUtils (destructor)
//Purpose:	Set radio mode back to original state
//				release the linked list, sockets
//				terminate the ReadThread thread
				

BthUtils::~BthUtils()
{
	//Set radio mode back to original state
	BthSetMode(m_dwBluetoothMode);
	if(m_pStart)
	{
		for(m_pCurrentDevice	= m_pStart;m_pCurrentDevice;)
		{
			DeviceList *temp	= m_pCurrentDevice;
			m_pCurrentDevice	= m_pCurrentDevice->NextDevice;
			free(temp);
		}
		m_pStart=NULL;
	}

	if(m_socketClient)
	    closesocket (m_socketClient);
    if(m_socketServer)
        closesocket (m_socketServer);
	
	//Terminate the read thread that receives chat messages from the client
	if(m_hReadThread)
	{
		DWORD dwExitCode = 0;
		TerminateThread(m_hReadThread, dwExitCode);
	}

	WSACleanup();

}

//Function: DiscoverDevices
//Purpose:	Searches Bluetooth devices in range
//				Populates the link list with the name and address of the devices found
//Return: If error occurs, returns the appropriate WSAGetLastError, otherwise returns zero.
				
int BthUtils::DiscoverDevices()
{
	WSAQUERYSET		wsaq;
	HANDLE			hLookup;
	DeviceList *	tempDevice;
	
	union {
		CHAR buf[5000];
		double __unused;	// ensure proper alignment
	};
	
	LPWSAQUERYSET pwsaResults = (LPWSAQUERYSET) buf;
	DWORD dwSize  = sizeof(buf);
	BOOL bHaveName;

    ZeroMemory(&wsaq, sizeof(wsaq));
	wsaq.dwSize = sizeof(wsaq);
	wsaq.dwNameSpace = NS_BTH;
	wsaq.lpcsaBuffer = NULL;

	if (ERROR_SUCCESS != WSALookupServiceBegin (&wsaq, LUP_CONTAINERS, &hLookup))
	{
		return WSAGetLastError();
	}

	ZeroMemory(pwsaResults, sizeof(WSAQUERYSET));
	pwsaResults->dwSize = sizeof(WSAQUERYSET);
	pwsaResults->dwNameSpace = NS_BTH;
	pwsaResults->lpBlob = NULL;
	
	if(m_pStart)
	{
		for(m_pCurrentDevice=m_pStart;m_pCurrentDevice;)
		{
			DeviceList *temp=m_pCurrentDevice;
			m_pCurrentDevice=m_pCurrentDevice->NextDevice;
    		free(temp);
		}
	}
	m_pEnd=m_pStart=NULL;
	m_iNumDevices=0;
	while (true)
	{	
		if(WSALookupServiceNext (hLookup, LUP_RETURN_NAME | LUP_RETURN_ADDR, &dwSize, pwsaResults)!=ERROR_SUCCESS)
			break;
		ASSERT (pwsaResults->dwNumberOfCsAddrs == 1);
		//Populate the link list		
		tempDevice=(DeviceList*)malloc(sizeof(DeviceList));
		tempDevice->NextDevice=NULL;
		if(m_pStart==NULL)
		{
			m_pStart = tempDevice;
			m_pEnd=m_pStart;
		}
		else
		{
			m_pEnd->NextDevice =tempDevice;
			m_pEnd=tempDevice;
		}
		m_iNumDevices++;
		m_pEnd->bthAddress = ((SOCKADDR_BTH *)pwsaResults->lpcsaBuffer->RemoteAddr.lpSockaddr)->btAddr;
		bHaveName = pwsaResults->lpszServiceInstanceName && *(pwsaResults->lpszServiceInstanceName);
		//If device name is available, add to node
		StringCchPrintf(m_pEnd->bthName, sizeof(m_pEnd->bthName),L"%s",bHaveName ? pwsaResults->lpszServiceInstanceName : L"");
	}
    
	WSALookupServiceEnd(hLookup);
//	LeaveCriticalSection(&criticalSection);
	return 0;
}

//Function: GetDeviceInfo
//Purpose:	Returns name and address of all the devices in the link list in DeviceInfo. This is used by the UI to display the names and addresses of the devices found
//Output:	DeviceInfo: name and address
//Return: Success returns zero.

int BthUtils::GetDeviceInfo(DeviceInfo *pPeerDevicesInfo)
{
	int iCtr=0;
	for (m_pCurrentDevice = m_pStart;(m_pCurrentDevice);m_pCurrentDevice=m_pCurrentDevice->NextDevice,iCtr++) 
	{ 
		StringCchPrintf(pPeerDevicesInfo[iCtr].szDeviceNameAddr, sizeof(pPeerDevicesInfo[iCtr].szDeviceNameAddr),  L"%s:(%04x%08x)", m_pCurrentDevice->bthName, GET_NAP(m_pCurrentDevice->bthAddress), GET_SAP(m_pCurrentDevice->bthAddress));		
	} 
	return 0;
}

//Function: OpenServerConnection
//Purpose:	Opens a server socket for listening. Registers the service. Creates a thread, ReadThread for reading incoming messages.
//Input:	The SDP record of the service to register, size of the SDP record, channel offset in the record, pointer to the UI function that displays the messages in the UI
//Return: If error occurs, returns the appropriate WSAGetLastError, otherwise returns zero.


int BthUtils::OpenServerConnection(BYTE *rgbSdpRecord, int cSdpRecord, int iChannelOffset, void (*funcPtr)(WCHAR *))
{
	int iNameLen=0;
	if(m_socketServer==INVALID_SOCKET)
	{
		m_socketServer = socket (AF_BT, SOCK_STREAM, BTHPROTO_RFCOMM);
		if (m_socketServer  == INVALID_SOCKET) 
		{
			return WSAGetLastError ();
		}
	
		SOCKADDR_BTH sa;
		memset (&sa, 0, sizeof(sa));
		sa.addressFamily = AF_BT;
		sa.port = 0;
		if (bind (m_socketServer, (SOCKADDR *)&sa, sizeof(sa))) 
		{
			return WSAGetLastError ();
		}
		iNameLen = sizeof(sa);
		if (getsockname(m_socketServer, (SOCKADDR *)&sa, &iNameLen))	
		{
			return WSAGetLastError ();
		}

		if(RegisterService(rgbSdpRecord, cSdpRecord, iChannelOffset, (UCHAR)sa.port)!=0)
			return WSAGetLastError();

		if (listen (m_socketServer, SOMAXCONN)) 
		{
			return WSAGetLastError ();
		}
	}
	pCallBackFunction=funcPtr;
	m_hReadThread= CreateThread(NULL, 0, ReadData, (LPVOID)this, 0, NULL);
	return 0;
}

//Function: RegisterService
//Purpose:	Publishes the SDP record.
//Input:	The SDP record of the service to register, size of the SDP record, channel offset in the record, channel number assigned automatically by OpenServerConnection
//Return: If error occurs, returns the appropriate WSAGetLastError, otherwise returns zero.

int BthUtils::RegisterService(BYTE *rgbSdpRecord, int cSdpRecord, int iChannelOffset, UCHAR channel)
{
		ULONG recordHandle = 0;
	
	struct bigBlob
	{
		BTHNS_SETBLOB   b;

	}*pBigBlob;

	pBigBlob = (bigBlob *)malloc(sizeof(struct bigBlob)+cSdpRecord);
	ULONG ulSdpVersion = BTH_SDP_VERSION;
	pBigBlob->b.pRecordHandle   = &recordHandle;
	pBigBlob->b.pSdpVersion     = &ulSdpVersion;
	pBigBlob->b.fSecurity       = 0;
	pBigBlob->b.fOptions        = 0;
	pBigBlob->b.ulRecordLength  = cSdpRecord;

	memcpy (pBigBlob->b.pRecord, rgbSdpRecord, cSdpRecord);
	pBigBlob->b.pRecord[iChannelOffset] = (unsigned char)channel;
	BLOB blob;
	blob.cbSize    = sizeof(BTHNS_SETBLOB) + cSdpRecord - 1;
	blob.pBlobData = (PBYTE) pBigBlob;

	WSAQUERYSET Service;
	memset (&Service, 0, sizeof(Service));
	Service.dwSize = sizeof(Service);
	Service.lpBlob = &blob;
	Service.dwNameSpace = NS_BTH;
	if (WSASetService(&Service,RNRSERVICE_REGISTER,0) == SOCKET_ERROR)
	{
		free(pBigBlob);
		return WSAGetLastError();
	}
	else
	{
		free(pBigBlob);
		return 0;
	}
}



//Function: SendMessageToServer
//Purpose:	Opens a client socket to connect to the server. Called when the local device initiates the chat.
//Input:	string containing the GUID of the service running on the server that the client wants to connect.
//			iSelectedDeviceIndex is the selected device in the UI that the local device wants to connect. If the peer device initiates the chat, the this parameter is set to -1.							
//Return: If error occurs, returns the appropriate WSAGetLastError, otherwise returns zero.

int BthUtils::SendMessageToServer(WCHAR *strGUID, WCHAR *szMessage, int iSelectedDeviceIndex)
{
	int iRetVal=0, iLenMessage=0, iBytesSent=0 ;
	if(m_socketClient==INVALID_SOCKET)
	{
		iRetVal=OpenClientConnection(strGUID, iSelectedDeviceIndex);
		if(iRetVal!=0)
		{
			return iRetVal;
		}
	}
	iLenMessage = (wcslen (szMessage) + 1) * sizeof(WCHAR);

    if (iLenMessage > sizeof (WCHAR)) 
    {
		iBytesSent = send (m_socketClient, (char *)szMessage, iLenMessage, 0);
		if (iBytesSent != iLenMessage)
        {
	        return WSAGetLastError ();
        }
	}
	return 0;
}

//Function: OpenClientConnection
//Purpose:	Opens a client socket to connect to the server.
//Input:	string containing the GUID of the service running on the server that the client wants to connect.
//			iSelectedDeviceIndex is the selected device in the UI that the local device wants to connect. If the peer device initiates the chat, the this parameter is set to -1.							
//Return: If error occurs, returns the appropriate WSAGetLastError, otherwise returns zero.
			

int BthUtils::OpenClientConnection(WCHAR *strGUID, int iSelectedDeviceIndex)
{

	if (m_socketClient==INVALID_SOCKET)
	{
		GUID ServerGuid;

		if(GetGUID(strGUID, &ServerGuid))
			return -1;
		m_socketClient = socket (AF_BT, SOCK_STREAM, BTHPROTO_RFCOMM);

		if (m_socketClient == INVALID_SOCKET) 
		{
			return WSAGetLastError();
		}
			
		SOCKADDR_BTH sa;

		memset (&sa, 0, sizeof(sa));
		sa.addressFamily = AF_BT;			
		//Search for the selected device in the list box in the link list
		m_pCurrentDevice=m_pStart;
		sa.serviceClassId=ServerGuid;

		if(iSelectedDeviceIndex==-1)
		{
			sa.btAddr=m_saClient.btAddr;
		}
		else
		{
			for (int iCount = 0 ;(m_pCurrentDevice)&&iCount!=iSelectedDeviceIndex;m_pCurrentDevice=m_pCurrentDevice->NextDevice,iCount++);
			sa.btAddr = m_pCurrentDevice->bthAddress;
		}

		if (connect (m_socketClient, (SOCKADDR *)&sa, sizeof(sa)) == SOCKET_ERROR) 
		{
			m_socketClient=INVALID_SOCKET;
			return WSAGetLastError();
		}
	}
	return 0;
}

//Function: GetGUID
//Purpose:	Conversts a string containing the GUID into a GUID datatype.
//Input:		string cotaining the GUID
//Output:	GUID type
//Return: Returns -1 in case of an error, otherwise returns zero.

int BthUtils::GetGUID(WCHAR *psz, GUID *pGUID) 
{
	int data1, data2, data3;
	int data4[8];

	if (11 ==  swscanf(psz, L"%08x-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x\n",
					&data1, &data2, &data3,
					&data4[0], &data4[1], &data4[2], &data4[3], 
					&data4[4], &data4[5], &data4[6], &data4[7])) {
		pGUID->Data1 = data1;
		pGUID->Data2 = data2 & 0xffff;
		pGUID->Data3 = data3 & 0xffff;

		for (int i = 0 ; i < 8 ; ++i)
			pGUID->Data4[i] = data4[i] & 0xff;

		return 0;
	}
	return -1;
}

//Function: GetLocalDeviceName
//Purpose:	Returns the name of the owner set in the registry
//Output:	DeviceInfo: (only)name
//Return: Returns -1 in case of an error, otherwise returns zero.

int BthUtils::GetLocalDeviceName(DeviceInfo *pLocalDeviceInfo)

{
    HKEY hKey;
    DWORD dwRegType, dwRegSize;
	if(RegOpenKeyEx(HKEY_CURRENT_USER,L"ControlPanel\\Owner",0,0,&hKey)==ERROR_SUCCESS)
	{
		if(RegQueryValueEx(hKey,L"Name",0,&dwRegType,(LPBYTE)pLocalDeviceInfo->szDeviceNameAddr,&dwRegSize)==ERROR_SUCCESS)
		{
			if (dwRegSize>MAX_NAME_SIZE) 
			{
				RegCloseKey(hKey);
				return -1; 
			}
			RegCloseKey(hKey);
		}
		RegCloseKey(hKey);
	}
    return 0;
}

//Function: GetDeviceInfo
//Purpose:	Searches the link list for the specified device and returns address and name in DeviceInfo
//Input:		The current device index selected in the UI
//Output:	DeviceInfo: name and address
//Return: Returns -1 in case of an error, otherwise returns zero.

int BthUtils::GetDeviceInfo(DeviceInfo *pPeerDeviceInfo, int iSelectedItem)
{
	int iCtr=0;

	for (m_pCurrentDevice = m_pStart;(m_pCurrentDevice);m_pCurrentDevice=m_pCurrentDevice->NextDevice,iCtr++) 
	{ 
		if(iCtr==iSelectedItem)
		{
			StringCchPrintf(pPeerDeviceInfo[0].szDeviceNameAddr, sizeof(pPeerDeviceInfo[0].szDeviceNameAddr), L"%s:(%04x%08x)", m_pCurrentDevice->bthName, GET_NAP(m_pCurrentDevice->bthAddress), GET_SAP(m_pCurrentDevice->bthAddress));		
			return 0;
		}
	} 
	return -1;
}



    Serial::Serial(){
    
    }
    
    int Serial::send(char *command, int bytes){

		          
		 int i=0;
         if (handle == INVALID_HANDLE_VALUE){
            return 0;
         }
         while(i<bytes){
            byte[i]=command[i];
            i++;
         }
         if(WriteFile(handle, byte, bytes, &nrBytes, NULL)!=0){
            return 1;      
         
         }
         else
            return 0;
    }    
    
    char *Serial::receive(int length){
        static char answer[MAX_NR_BYTES];
        int i=0;
        nrBytes=0;
        answer[0]='\0';
        if (ReadFile(handle, byte, length, &nrBytes, NULL) == 0){
            answer[0]='e';
            answer[1]='r';
            answer[2]='r';
            answer[3]='o';
            answer[4]='r';
            answer[5]='\0';
            return &answer[0];
        }
        //printf("Number of bytes %d\n",nrBytes);
        while(i<nrBytes){
            answer[i]=byte[i];            
            //printf("%d ",answer[i]);
            i++;
        }
        //printf("\n");
        answer[i]='\0';
        return &answer[0];
    }
    
    int Serial::flush(){
        int i=0;
//        clock_t endwait;
        //endwait = clock () + 1000 * CLK_TCK ;
        while(nrBytes > 0){
        
             if(ReadFile(handle, byte, 1, &nrBytes, NULL)==0){
                 return 0;
             }
             i++;
        }
        return i;
    }
    
    int Serial::connect(wstring port){
		return connect(port, 200, 8, 19200, 0); 
		    }

    int Serial::connect(wstring port, int set_timeout, int bytesize, int baudrate, int parity){
        /***************************************************************************       
        *** See http://www.robbayer.com/files//serial-win.pdf for documentation  ****
        ***************************************************************************/    
		handle = CreateFile(L"COM7:", GENERIC_READ | GENERIC_WRITE, 0,0,OPEN_EXISTING,0,0);
        if (handle == INVALID_HANDLE_VALUE){
			//(LPCWSTR)&port
			//L"COM7:"
			MessageBeep(MB_ICONASTERISK);
            return 0;
        }
        dcb.ByteSize = bytesize;/*sets bit-size*/
        dcb.BaudRate = baudrate;/*sets baudrate*/
        /*Sets timeout for reading - 0 = deactivate*/
        timeout.ReadIntervalTimeout=set_timeout;
        timeout.ReadTotalTimeoutConstant=set_timeout;
        dcb.Parity=parity;
        timeout.ReadTotalTimeoutMultiplier=set_timeout;
        SetCommTimeouts(handle, &timeout);
        SetCommState(handle, &dcb);
        return 1;
    }
    
    int Serial::disconnect(){
          if (handle){
            CloseHandle(handle);
            return 1;
         }
         else{
            return 0;
        }
    }
