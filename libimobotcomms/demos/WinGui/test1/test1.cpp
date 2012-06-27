// test1.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "test1.h"
#include "windows.h"
#include <windowsx.h>
#include "comutility.h"
#include <mobot.h>
#include <cstdlib>
#include "gait.h"

#define MAX_LOADSTRING 100
#define IN_TEXT_SIZE 256

#define SET_ANGLES(angles, a, b, c, d) \
	angles[0] = a; \
	angles[1] = b; \
	angles[2] = c; \
	angles[3] = d

typedef struct imobotMotion_s {
	WCHAR name[100];
	double angles[4];
	unsigned char motorMask;
} imobotMotion_t;


//define serial/bluetooth connection
Serial bluetooth;
BthUtils objBthUtils;
CMobot samplebt ; 

imobotMotion_t g_imobotPoses[50];
int g_numPoses;
imobotMotion_t g_imobotMoves[50];
int g_numMoves;

Gait* g_gaits[50];
int g_numGaits;

char btadd[20];


WSADATA wsd;
SOCKADDR_BTH sa;
//SOCKET client_socket = socket (AF_BT, SOCK_STREAM, BTHPROTO_RFCOMM);


// Global Variables:
HINSTANCE			g_hInst;			// current instance
HWND				g_hWndMenuBar;		// menu bar handle

// Forward declarations of functions included in this code module:
ATOM			MyRegisterClass(HINSTANCE, LPTSTR);
BOOL			InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	first_dia(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	pose_dia(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	move_dia(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	gait_dia(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	newgait_dia(HWND, UINT, WPARAM, LPARAM);
int getDialogAngles(
					HWND hDlg,
					int dialogID1, 
					int dialogID2, 
					int dialogID3, 
					int dialogID4, 
					float angles[4],  /* OUT */
					unsigned char* motorMask /* OUT */);

int addGait(Gait* gait);
int initGaits();

int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPTSTR    lpCmdLine,
                   int       nCmdShow)
{
	MSG msg;

	g_numPoses = 0;
	g_numMoves = 0;
	g_numGaits = 0;

	initGaits();
	

	// Perform application initialization:
	if (!InitInstance(hInstance, nCmdShow)) 
	{
		return FALSE;
	}

	HACCEL hAccelTable;
	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_TEST1));

	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0)) 
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg)) 
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int) msg.wParam;
}

//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
//  COMMENTS:
//
ATOM MyRegisterClass(HINSTANCE hInstance, LPTSTR szWindowClass)
{
	WNDCLASS wc;

	wc.style         = CS_HREDRAW | CS_VREDRAW;
	wc.lpfnWndProc   = WndProc;
	wc.cbClsExtra    = 0;
	wc.cbWndExtra    = 0;
	wc.hInstance     = hInstance;
	wc.hIcon         = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_TEST1));
	wc.hCursor       = 0;
	wc.hbrBackground = (HBRUSH) GetStockObject(WHITE_BRUSH);
	wc.lpszMenuName  = 0;
	wc.lpszClassName = szWindowClass;

	return RegisterClass(&wc);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
    HWND hWnd;
    TCHAR szTitle[MAX_LOADSTRING];		// title bar text
    TCHAR szWindowClass[MAX_LOADSTRING];	// main window class name

    g_hInst = hInstance; // Store instance handle in our global variable

    // SHInitExtraControls should be called once during your application's initialization to initialize any
    // of the device specific controls such as CAPEDIT and SIPPREF.
    SHInitExtraControls();

    LoadString(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING); 
    LoadString(hInstance, IDC_TEST1, szWindowClass, MAX_LOADSTRING);

    //If it is already running, then focus on the window, and exit
    hWnd = FindWindow(szWindowClass, szTitle);	
    if (hWnd) 
    {
        // set focus to foremost child window
        // The "| 0x00000001" is used to bring any owned windows to the foreground and
        // activate them.
        SetForegroundWindow((HWND)((ULONG) hWnd | 0x00000001));
        return 0;
    } 

    if (!MyRegisterClass(hInstance, szWindowClass))
    {
    	return FALSE;
    }

    hWnd = CreateWindow(szWindowClass, szTitle, WS_VISIBLE,
        CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, NULL, NULL, hInstance, NULL);

    if (!hWnd)
    {
        return FALSE;
    }

    // When the main window is created using CW_USEDEFAULT the height of the menubar (if one
    // is created is not taken into account). So we resize the window after creating it
    // if a menubar is present
    if (g_hWndMenuBar)
    {
        RECT rc;
        RECT rcMenuBar;

        GetWindowRect(hWnd, &rc);
        GetWindowRect(g_hWndMenuBar, &rcMenuBar);
        rc.bottom -= (rcMenuBar.bottom - rcMenuBar.top);
		
        MoveWindow(hWnd, rc.left, rc.top, rc.right-rc.left, rc.bottom-rc.top, FALSE);
    }

    ShowWindow(hWnd, nCmdShow);
    UpdateWindow(hWnd);


    return TRUE;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    int wmId, wmEvent;
    PAINTSTRUCT ps;
    HDC hdc;

    static SHACTIVATEINFO s_sai;
	
    switch (message) 
    {
        case WM_COMMAND:
            wmId    = LOWORD(wParam); 
            wmEvent = HIWORD(wParam); 
            // Parse the menu selections:
            switch (wmId)
            {
                case IDM_HELP_ABOUT:
                    DialogBox(g_hInst, (LPCTSTR)IDD_ABOUTBOX, hWnd, About);
                    break;
				case ID_HELP_DIA1:
                    DialogBox(g_hInst, (LPCTSTR)IDD_DIA2, hWnd, About);
                    break;
				case ID_HELP_DIA2:
                    DialogBox(g_hInst, (LPCTSTR)IDD_DIA3, hWnd, About);
                    break;
                case IDM_OK:
                    SendMessage (hWnd, WM_CLOSE, 0, 0);				
                    break;
                default:
                    return DefWindowProc(hWnd, message, wParam, lParam);
            }
            break;

        case WM_CREATE:
            SHMENUBARINFO mbi;

            memset(&mbi, 0, sizeof(SHMENUBARINFO));
            mbi.cbSize     = sizeof(SHMENUBARINFO);
            mbi.hwndParent = hWnd;
            mbi.nToolBarId = IDR_MENU;
            mbi.hInstRes   = g_hInst;

            if (!SHCreateMenuBar(&mbi)) 
            {
                g_hWndMenuBar = NULL;
            }
            else
            {
                g_hWndMenuBar = mbi.hwndMB;
            }

            // Initialize the shell activate info structure
            memset(&s_sai, 0, sizeof (s_sai));
            s_sai.cbSize = sizeof (s_sai);
			DialogBox(g_hInst, (LPCTSTR)IDD_ABOUTBOX, hWnd, About);
            break;
        case WM_PAINT:
            hdc = BeginPaint(hWnd, &ps);
            
            // TODO: Add any drawing code here...
            
            EndPaint(hWnd, &ps);
            break;
        case WM_DESTROY:
            CommandBar_Destroy(g_hWndMenuBar);
            PostQuitMessage(0);
            break;

        case WM_ACTIVATE:
            // Notify shell of our activate message
            SHHandleWMActivate(hWnd, wParam, lParam, &s_sai, FALSE);
            break;
        case WM_SETTINGCHANGE:
            SHHandleWMSettingChange(hWnd, wParam, lParam, &s_sai);
            break;

        default:
            return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	WCHAR posenotoset[IN_TEXT_SIZE],enc2[IN_TEXT_SIZE], enc3[IN_TEXT_SIZE], enc4[IN_TEXT_SIZE];
	WCHAR readdata[20];
	WCHAR enc1[IN_TEXT_SIZE];
	WCHAR gaits[30];
	float tempf[5];
	int temp[5];

	//initialize arrays for commands

	char getpos[7] = {'G','E','T','_','P', 'O', 'S'} ;
	char status[6] = {'S','T','A','T','U', 'S'} ;
	char check[3] = {'C','H','K'} ;
	

	int channel = 20;

	//strcpy(btadd,"00:80:37:27:03:D8");
	//strcpy(btadd,"00:80:37:2E:45:D2");
	//strcpy(btadd, "00:19:88:19:FB:9E"); //new iMobot module
	strcpy(btadd, "00:06:66:45:D9:D3"); //new Mobot module

	//initialise command character arrays

	switch (message)
    {
        case WM_INITDIALOG:
            {
                // Create a Done button and size it.  
                SHINITDLGINFO shidi;
                shidi.dwMask = SHIDIM_FLAGS;
                shidi.dwFlags = SHIDIF_DONEBUTTON | SHIDIF_SIPDOWN | SHIDIF_SIZEDLGFULLSCREEN | SHIDIF_EMPTYMENU;
                shidi.hDlg = hDlg;
                SHInitDialog(&shidi);

				WSAStartup(MAKEWORD(1,0), &wsd);
				memset (&sa, 0, sizeof(sa));
				sa.btAddr = (BT_ADDR)0x0080372703D8; 
				sa.port = channel & 0xff;

				//bluetooth.connect(L"COM7");

            }
            return (INT_PTR)TRUE;

        case WM_COMMAND:
		    wmId    = LOWORD(wParam); 
            wmEvent = HIWORD(wParam); 
            // Parse the menu selections:
            switch (wmId)
            {
                case IDOK:
                    EndDialog(hDlg, LOWORD(wParam));
					return TRUE;
                    break;
                case IDC_CONNECT:
 					
					if (samplebt.connect(btadd,20)) 
					{
						MessageBox(hDlg, L"Did not Connect", L"Error", MB_OK);
						//Perform error handling.
						//closesocket (client_socket);
						return 0;
					}
					else
						MessageBox(hDlg, L"Connected to Device", L"Success", MB_OK);

					samplebt.setJointSpeed(0, 40);
					samplebt.setJointSpeed(1, 40);
					samplebt.setJointSpeed(2, 40);
					samplebt.setJointSpeed(3, 40);


					break;

				case IDC_MOVE:
					GetDlgItemText(hDlg, IDC_POSENO2,enc1,IN_TEXT_SIZE);
					//swscanf(enc1, L"%d", &temp[0]) ;
					//samplebt.goToPose(temp[0]);

			        break;

				case IDC_GAIT:
					GetDlgItemText(hDlg, IDC_GAITBOX,gaits,30);
					/*wchar_t* wc;
					wc = wcstok(gaits, L", ");
					while(wc != NULL) {
						swscanf(wc, L"%d", &temp[0]);
						samplebt.goToPose(temp[0]);
						samplebt.moveWait();
						wc = wcstok(NULL, L", ");
					} */

					MessageBox(hDlg, L"Gaits Completed", L"Success", MB_OK);

			        break;

				case IDC_SETPOSE:
					
#if 0
					GetDlgItemText(hDlg, IDC_POSENO,posenotoset,IN_TEXT_SIZE);
					GetDlgItemText(hDlg, IDC_ENC1,enc1,IN_TEXT_SIZE);
					GetDlgItemText(hDlg, IDC_ENC2,enc2,IN_TEXT_SIZE);
					GetDlgItemText(hDlg, IDC_ENC3,enc3,IN_TEXT_SIZE);
					GetDlgItemText(hDlg, IDC_ENC4,enc4,IN_TEXT_SIZE);
					
					//wcstombs (tempchar, enc1, 3);
#ifdef UNICODE
					swscanf(posenotoset, L"%d", &temp[0]) ;
					swscanf(enc1, L"%f", &tempf[0]) ;
					swscanf(enc2, L"%f", &tempf[1]) ;
					swscanf(enc3, L"%f", &tempf[2]) ;
					swscanf(enc4, L"%f", &tempf[3]) ;

#else
					sscanf(posenotoset, "%d", &temp[0]) ;
#endif
				    samplebt.setPoseAngles(temp[0],tempf) ;
					samplebt.goToPose(temp[0]);
#endif
					break;
				
				case IDC_GETPOS:

					double tempenc ;
					
					samplebt.getJointAngle(0,tempenc);
					swprintf(readdata,L"%lf",tempenc);
					SetDlgItemText(hDlg, IDC_ENC5, readdata);
					SetDlgItemText(hDlg, IDC_ENC1, readdata);
					Sleep(500);
					
					samplebt.getJointAngle(1,tempenc);
					swprintf(readdata,L"%lf",tempenc);
					SetDlgItemText(hDlg, IDC_ENC6, readdata);
					SetDlgItemText(hDlg, IDC_ENC2, readdata);
					Sleep(500);

					samplebt.getJointAngle(2,tempenc);
					swprintf(readdata,L"lf",tempenc);
					SetDlgItemText(hDlg, IDC_ENC7, readdata);
					SetDlgItemText(hDlg, IDC_ENC3, readdata);
					Sleep(500);

					samplebt.getJointAngle(3,tempenc);
					swprintf(readdata,L"lf",tempenc);
					SetDlgItemText(hDlg, IDC_ENC8, readdata);
					SetDlgItemText(hDlg, IDC_ENC4, readdata);
					Sleep(500);

					
                    break;

				case IDC_PLAY:
#if 0
					if(samplebt.play())
						MessageBox(hDlg, L"Did not Send Message", L"Error", MB_OK);
#endif
                    break;
				case IDC_PAUSE:
                    //if (samplebt.pause())
					//	MessageBox(hDlg, L"Did not Send Message", L"Error", MB_OK);
                    break;
				case IDC_STOP:
                    if (samplebt.stop())
						MessageBox(hDlg, L"Did not Send Message", L"Error", MB_OK);
					break;
				case IDC_QUIT:
					DialogBox(g_hInst, (LPCTSTR)IDD_DIA2, hDlg, first_dia);
                    //if(samplebt.quit())
					//	MessageBox(hDlg, L"Did not Send Message", L"Error", MB_OK);
                    break;

                //default:
                  //  MessageBox(hDlg, (LPCTSTR)"Test", (LPCTSTR)"Caption", MB_OK);
				   //return DefWindowProc(hWnd, message, wParam, lParam);
            }
            
            break;

        case WM_CLOSE:
            EndDialog(hDlg, message);
            return TRUE;

    }
    return (INT_PTR)FALSE;
}



// Message handler for dia2 box.
INT_PTR CALLBACK first_dia(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	WCHAR readdata[20];
	WCHAR enc1[IN_TEXT_SIZE];
	WCHAR gaits[30];
	int temp[5];

	//initialize arrays for commands

	char getpos[7] = {'G','E','T','_','P', 'O', 'S'} ;
	char status[6] = {'S','T','A','T','U', 'S'} ;
	char check[3] = {'C','H','K'} ;
	

	int channel = 20;

	//strcpy(btadd,"00:80:37:27:03:D8");
	//strcpy(btadd,"00:80:37:2E:45:D2");
	//strcpy(btadd,"00:19:88:19:FB:9E");
	strcpy(btadd,"00:06:66:45:D9:D3");

	switch (message)
    {
        case WM_INITDIALOG:
            {
                // Create a Done button and size it.  
                SHINITDLGINFO shidi;
                shidi.dwMask = SHIDIM_FLAGS;
                shidi.dwFlags = SHIDIF_DONEBUTTON | SHIDIF_SIPDOWN | SHIDIF_SIZEDLGFULLSCREEN | SHIDIF_EMPTYMENU;
                shidi.hDlg = hDlg;
                SHInitDialog(&shidi);

				WSAStartup(MAKEWORD(1,0), &wsd);
				memset (&sa, 0, sizeof(sa));

            }
            return (INT_PTR)TRUE;

        case WM_COMMAND:
		    wmId    = LOWORD(wParam); 
            wmEvent = HIWORD(wParam); 
            // Parse the menu selections:
            switch (wmId)
            {
                case IDOK:
                    EndDialog(hDlg, LOWORD(wParam));
					return TRUE;
                    break;
				case IDC_BUTTON1: /* Manage Gaits button */
					DialogBox(g_hInst, (LPCTSTR)IDD_GAIT, hDlg, gait_dia);
					break;
				case IDC_MANAGEPOSE:
 					DialogBox(g_hInst, (LPCTSTR)IDD_POSE, hDlg, pose_dia);
					break;
				case IDC_MANAGEMOVE:
					DialogBox(g_hInst, (LPCTSTR)IDD_MOVE, hDlg, move_dia);
					break;
				
				case IDC_CONNECT:
 					
					if (samplebt.connect(btadd,20)) 
					{
						MessageBox(hDlg, L"Did not Connect", L"Error", MB_OK);
						//Perform error handling.
						//closesocket (client_socket);
						return 0;
					}
					else
						MessageBox(hDlg, L"Connected to Device", L"Success", MB_OK);
					break;

				case IDC_GAIT:
#if 0
					GetDlgItemText(hDlg, IDC_GAITBOX,gaits,30);
					swscanf(gaits, L"%d %d %d %d %d", &temp[0], &temp[1],&temp[2],&temp[3],&temp[4]);
					samplebt.goToPose(temp[0]);
					samplebt.moveWait();
					samplebt.goToPose(temp[1]);
					samplebt.moveWait();
					samplebt.goToPose(temp[2]);
					samplebt.moveWait();
					samplebt.goToPose(temp[3]);
					samplebt.moveWait();
					samplebt.goToPose(temp[4]);
					samplebt.moveWait();

					MessageBox(hDlg, L"Gaits Completed", L"Success", MB_OK);
#endif

			        break;

				case IDC_MOVE:
#if 0
					GetDlgItemText(hDlg, IDC_POSENO2,enc1,IN_TEXT_SIZE);
					swscanf(enc1, L"%d", &temp[0]) ;
					samplebt.goToPose(temp[0]);
#endif

			        break;

				case IDC_PLAY:
#if 0
					if(samplebt.play())
						MessageBox(hDlg, L"Did not Send Message", L"Error", MB_OK);
#endif
                    break;
				case IDC_PAUSE:
                    //if (samplebt.pause())
					//	MessageBox(hDlg, L"Did not Send Message", L"Error", MB_OK);
                    break;
				case IDC_STOP:
                    if (samplebt.stop())
						MessageBox(hDlg, L"Did not Send Message", L"Error", MB_OK);
					break;
				case IDC_QUIT:
					//DialogBox(g_hInst, (LPCTSTR)IDD_DIA2, hDlg, first_dia);
                    if(samplebt.disconnect())
						MessageBox(hDlg, L"Did not Send Message", L"Error", MB_OK);
                    break;

				case IDC_GETPOS:

					double tempenc ;
					
					samplebt.getJointAngle(0,tempenc);
					swprintf(readdata,L"%lf",tempenc);
					SetDlgItemText(hDlg, IDC_ENC5, readdata);
					
					
					samplebt.getJointAngle(1,tempenc);
					swprintf(readdata,L"%lf",tempenc);
					SetDlgItemText(hDlg, IDC_ENC6, readdata);
					

					samplebt.getJointAngle(2,tempenc);
					swprintf(readdata,L"%lf",tempenc);
					SetDlgItemText(hDlg, IDC_ENC7, readdata);
					

					samplebt.getJointAngle(3,tempenc);
					swprintf(readdata,L"%lf",tempenc);
					SetDlgItemText(hDlg, IDC_ENC8, readdata);
					

					
                    break;
				case IDC_BUTTON2: /* DEMO button */
#if 0
					if(samplebt.runDemo()) {
						MessageBox(hDlg, L"Did not Send Message", L"Error", MB_OK);
					}
#endif
					break;
	
               //default:
                  //  MessageBox(hDlg, (LPCTSTR)"Test", (LPCTSTR)"Caption", MB_OK);
				   //return DefWindowProc(hWnd, message, wParam, lParam);
            }
            
            break;

        case WM_CLOSE:
            EndDialog(hDlg, message);
            return TRUE;

    }
    return (INT_PTR)FALSE;
}




// Message handler for dia3 box.
INT_PTR CALLBACK pose_dia(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent, i;

	switch (message)
    {
		case WM_INITDIALOG:
			{
			/* Fill the textbox with pose names */
			HWND hListBox = GetDlgItem(hDlg, IDC_LIST2);
			for(int i = 0; i < g_numPoses; i++) {
				ListBox_AddString(hListBox, g_imobotPoses[i].name);
			}
			break;
			}
		case WM_COMMAND:
		    wmId    = LOWORD(wParam); 
            wmEvent = HIWORD(wParam); 
            // Parse the menu selections:
            switch (wmId)
            {
                case IDC_BUTTON7: /* Close Button */
                    EndDialog(hDlg, LOWORD(wParam));
					return TRUE;
                
				case IDC_BUTTON5: /* Custom Pose button */
					{
						double angles[4];
						unsigned char motorMask;

						getDialogAngles(hDlg, IDC_EDIT2, IDC_EDIT3, IDC_EDIT4, IDC_EDIT5, angles, &motorMask);
						
						//samplebt.poseJointAngles(angles, motorMask);
						for(i = 0; i < 4; i++) {
							if((1<<i) & motorMask) {
								samplebt.moveJointTo(i, angles[i]);
							}
						}
						return TRUE;
					}
				case IDC_BUTTON6: /* Save pose as button */
					{
						/* Get the pose name */
						WCHAR name[80];
						GetDlgItemText(hDlg, IDC_EDIT6, name, 80);
						if(wcslen(name) == 0) {
							break;
						}
						wcscpy(g_imobotPoses[g_numPoses].name, name);
						getDialogAngles(hDlg, IDC_EDIT2, IDC_EDIT3, IDC_EDIT4, IDC_EDIT5, 
							g_imobotPoses[g_numPoses].angles,
							&g_imobotPoses[g_numPoses].motorMask);
						/* Add it to the listbox */
						HWND hListBox = GetDlgItem(hDlg, IDC_LIST2);
						ListBox_AddString(hListBox, g_imobotPoses[g_numPoses].name);

						g_numPoses++;
						return TRUE;
					}
				case IDC_BUTTON4: /* Go to saved pose */
					{
						/* Get the index of the selected item */
						HWND hListBox = GetDlgItem(hDlg, IDC_LIST2);
						int index;
						index = ListBox_GetCurSel(hListBox);
						if(index == LB_ERR) {
							break;
						}
						//samplebt.poseJointAngles(
						//	g_imobotPoses[index].angles, 
						//	g_imobotPoses[index].motorMask);
						for(i = 0; i < 4; i++) {
							if((1<<i) & g_imobotPoses[index].motorMask)
							{
								samplebt.moveJointTo(i,g_imobotPoses[index].angles[i]);
							}
						}
						return TRUE;
					}
				
	
               //default:
                  //  MessageBox(hDlg, (LPCTSTR)"Test", (LPCTSTR)"Caption", MB_OK);
				   //return DefWindowProc(hWnd, message, wParam, lParam);
            }
			break;
	case WM_CLOSE:
            EndDialog(hDlg, message);
            return TRUE;

    }
    return (INT_PTR)FALSE;
}
INT_PTR CALLBACK move_dia(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent, i;

	switch (message)
    {
		case WM_INITDIALOG:
			{
			/* Fill the textbox with move names */
			HWND hListBox = GetDlgItem(hDlg, IDC_LIST2);
			for(int i = 0; i < g_numMoves; i++) {
				ListBox_AddString(hListBox, g_imobotMoves[i].name);
			}
			break;
			}
		case WM_COMMAND:
		    wmId    = LOWORD(wParam); 
            wmEvent = HIWORD(wParam); 
            // Parse the menu selections:
            switch (wmId)
            {
                case IDC_BUTTON7: /* Close Button */
                    EndDialog(hDlg, LOWORD(wParam));
					return TRUE;
                
				case IDC_BUTTON5: /* Custom Move button */
					{
						double angles[4];
						double angle;
						unsigned char motorMask;

						getDialogAngles(hDlg, IDC_EDIT2, IDC_EDIT3, IDC_EDIT4, IDC_EDIT5, angles, &motorMask);
						
						for(i = 0; i < 4; i++) {
							if((1<<i) & motorMask) {
								samplebt.getJointAngle(i, angle);
								angle += angles[i];
								samplebt.moveJointTo(i, angle);
							}
						}
						return TRUE;
					}
				case IDC_BUTTON6: /* Save pose as button */
					{
						/* Get the pose name */
						WCHAR name[80];
						GetDlgItemText(hDlg, IDC_EDIT6, name, 80);
						if(wcslen(name) == 0) {
							break;
						}
						wcscpy(g_imobotMoves[g_numMoves].name, name);
						getDialogAngles(hDlg, IDC_EDIT2, IDC_EDIT3, IDC_EDIT4, IDC_EDIT5, 
							g_imobotMoves[g_numMoves].angles,
							&g_imobotMoves[g_numMoves].motorMask);
						/* Add it to the listbox */
						HWND hListBox = GetDlgItem(hDlg, IDC_LIST2);
						ListBox_AddString(hListBox, g_imobotMoves[g_numMoves].name);

						g_numMoves++;
						return TRUE;
					}
				case IDC_BUTTON4: /* Go to saved pose */
					{
						/* Get the index of the selected item */
						HWND hListBox = GetDlgItem(hDlg, IDC_LIST2);
						int index;
						index = ListBox_GetCurSel(hListBox);
						if(index == LB_ERR) {
							break;
						}
						double angle;
						for(i = 0; i < 4; i++) {
							if((1<<i) & g_imobotMoves[index].motorMask) {
								samplebt.getJointAngle(i, angle);
								angle += g_imobotMoves[index].angles[i];
								samplebt.moveJointTo(i, angle);
							}
						}
						/*samplebt.moveJointAngles(
							g_imobotMoves[index].angles, 
							g_imobotMoves[index].motorMask); */
						return TRUE;
					}
				
	
               //default:
                  //  MessageBox(hDlg, (LPCTSTR)"Test", (LPCTSTR)"Caption", MB_OK);
				   //return DefWindowProc(hWnd, message, wParam, lParam);
            }
			break;
	case WM_CLOSE:
            EndDialog(hDlg, message);
            return TRUE;

    }
    return (INT_PTR)FALSE;
}

INT_PTR CALLBACK newgait_dia(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;
	static Motion* motions[50];
	static int numMotions;

	switch (message)
    {
		case WM_INITDIALOG:
			{
			numMotions = 0;
			/* Fill the pose textbox with pose names */
			HWND hListBox = GetDlgItem(hDlg, IDC_LIST2);
			ListBox_ResetContent(hListBox);
			for(int i = 0; i < g_numPoses; i++) {
				ListBox_AddString(hListBox, g_imobotPoses[i].name);
			}
			/* Fill the move textbox with move names */
			hListBox = GetDlgItem(hDlg, IDC_LIST3);
			ListBox_ResetContent(hListBox);
			for(int i = 0; i < g_numMoves; i++) {
				ListBox_AddString(hListBox, g_imobotMoves[i].name);
			}
			break;
			}
		case WM_COMMAND:
		    wmId    = LOWORD(wParam); 
            wmEvent = HIWORD(wParam); 
            // Parse the menu selections:
            switch (wmId)
            {
				case IDC_BUTTON2: /* Add Pose Button Clicked */
					{
						HWND hListBox = GetDlgItem(hDlg, IDC_LIST2);
						/* Get the index of the item */
						int index = ListBox_GetCurSel(hListBox);
						if(index == LB_ERR) {
							return TRUE;
						}
						/* Create a motion based on that item */
						motions[numMotions] = new Motion(
							MOTION_POSE,
							g_imobotPoses[index].angles,
							g_imobotPoses[index].motorMask );
						numMotions++;
						/* Add it to the list box */
						hListBox = GetDlgItem(hDlg, IDC_LIST4);
						ListBox_AddString(hListBox, g_imobotPoses[index].name);
						return TRUE;
					}
				case IDC_BUTTON3: /* Add Move Button Clicked */
					{
						HWND hListBox = GetDlgItem(hDlg, IDC_LIST3);
						/* Get the index of the item */
						int index = ListBox_GetCurSel(hListBox);
						if(index == LB_ERR) {
							return TRUE;
						}
						/* Create a motion based on that item */
						motions[numMotions] = new Motion(
							MOTION_MOVE,
							g_imobotMoves[index].angles,
							g_imobotMoves[index].motorMask );
						numMotions++;
						/* Add it to the list box */
						hListBox = GetDlgItem(hDlg, IDC_LIST4);
						ListBox_AddString(hListBox, g_imobotMoves[index].name);
						return TRUE;
					}
				case IDC_BUTTON5: /* Save button clicked */
					{
						/* Get the gait name */
						HWND editbox = GetDlgItem(hDlg, IDC_EDIT1);
						WCHAR gaitName[80];
						Edit_GetText(editbox, gaitName, 80);
						if(wcslen(gaitName) == 0) {
							return TRUE;
						}

						/* Create a gait with each of the motions */
						Gait* gait = new Gait(gaitName);
						for(int i = 0; i < numMotions; i++) {
							gait->addMotion(motions[i]);
						}
						g_gaits[g_numGaits] = gait;
						g_numGaits++;
						EndDialog(hDlg, LOWORD(wParam));
						return TRUE;
					}
				case IDC_BUTTON4: /* Cancel button clicked */
					{
						EndDialog(hDlg, LOWORD(wParam));
						return TRUE;
					}
            }
			break;
	case WM_CLOSE:
            EndDialog(hDlg, message);
            return TRUE;

    }
    return (INT_PTR)FALSE;
}

INT_PTR CALLBACK gait_dia(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;

	switch (message)
    {
		case WM_INITDIALOG:
			{
			/* TODO: Fill the gait textbox with gait names */
			HWND hListBox = GetDlgItem(hDlg, IDC_LIST1);
			ListBox_ResetContent(hListBox);
			for(int i = 0; i < g_numGaits; i++) {
				ListBox_AddString(hListBox, g_gaits[i]->getName());
			}
			break;
			}
		case WM_COMMAND:
		    wmId    = LOWORD(wParam); 
            wmEvent = HIWORD(wParam); 
            // Parse the menu selections:
            switch (wmId)
            {
				case IDC_BUTTON1: /* Play Gait button clicked */
					{
						/* Get the index of the gait to play */
						HWND hListBox = GetDlgItem(hDlg, IDC_LIST1);
						int index = ListBox_GetCurSel(hListBox);
						if(index == LB_ERR) {
							return TRUE;
						}
						int numMotions = g_gaits[index]->getNumMotions();
						const Motion *motion;
						for(int i = 0; i < numMotions; i++) {
							motion = g_gaits[index]->getMotion(i);
							if(motion->getType() == MOTION_POSE) {
								poseJoints(motion->getAngles(), motion->getMotorMask());
							} else if (motion->getType() == MOTION_MOVE) {
								moveJoints(motion->getAngles(), motion->getMotorMask());
							}
							moveWait();
						}
						break;
					}
				case IDC_BUTTON2: /* New Gait button clicked */
					{
						DialogBox(g_hInst, (LPCTSTR)IDD_NEWGAIT, hDlg, newgait_dia);
						HWND hListBox = GetDlgItem(hDlg, IDC_LIST1);
						ListBox_ResetContent(hListBox);
						for(int i = 0; i < g_numGaits; i++) {
							ListBox_AddString(hListBox, g_gaits[i]->getName());
						}
						break;
					}
				case IDC_BUTTON3: /* Cancel button clicked */
					{
						EndDialog(hDlg, LOWORD(wParam));
						return TRUE;
					}
            }
			break;
	case WM_CLOSE:
            EndDialog(hDlg, message);
            return TRUE;

    }
    return (INT_PTR)FALSE;
}

int getDialogAngles(
					HWND hDlg,
					int dialogID1, 
					int dialogID2, 
					int dialogID3, 
					int dialogID4, 
					double angles[4],  /* OUT */
					unsigned char* motorMask /* OUT */)
{
	// Custom Pose button on Pose dialog 
					// Get the angles and go to that pose
					*motorMask = 0;
					WCHAR buf[80];
					/* Get each one of the entry box texts */
					GetDlgItemText(hDlg, dialogID1, buf, 80);
					if(wcslen(buf) > 0) {
						swscanf(buf, L"%f", &angles[0]);
						*motorMask |= 1<<0;
					}
					GetDlgItemText(hDlg, dialogID2, buf, 80);
					if(wcslen(buf) > 0) {
						swscanf(buf, L"%f", &angles[1]);
						*motorMask |= 1<<1;
					}
					GetDlgItemText(hDlg, dialogID3, buf, 80);
					if(wcslen(buf) > 0) {
						swscanf(buf, L"%f", &angles[2]);
						*motorMask |= 1<<2;
					}
					GetDlgItemText(hDlg, dialogID4, buf, 80);
					if(wcslen(buf) > 0) {
						swscanf(buf, L"%f", &angles[3]);
						*motorMask |= 1<<3;
					}
					return 0;
}

int addGait(Gait* gait)
{
	g_gaits[g_numGaits] = gait;
	g_numGaits++;
	return 0;
}

int initGaits()
{
	/* Add some pre-programmed gaits into the mobot */

	/* Rotate Left */
	Gait* gait;
	gait = new Gait(L"Rotate Left");
	double angles[4] = {0, 0, 90, 90};
	unsigned char motorMask = 0;
	motorMask |= (1<<2);
	motorMask |= (1<<3);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Rotate Right */
	gait = new Gait(L"Rotate Right");
	SET_ANGLES(angles, 0, 0, -90, -90);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Roll Forward */
	gait = new Gait(L"Roll Forward");
	SET_ANGLES(angles, 0, 0, 90, -90);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Roll Backward */
	gait = new Gait(L"Roll Backward");
	SET_ANGLES(angles, 0, 0, -90, 90);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Arch */
	gait = new Gait(L"Arch");
	SET_ANGLES(angles, -30, 30, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion( new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Flat */
	gait = new Gait (L"Flat");
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Inch Right */
	gait = new Gait(L"Inch Right");
	SET_ANGLES(angles, -50, 0, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, -50, 50, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 50, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Inch Left */
	gait = new Gait(L"Inch Left");
	SET_ANGLES(angles, 0, 50, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, -50, 50, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, -50, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Stand */
	gait = new Gait(L"Stand");
	SET_ANGLES(angles, 0, 0, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, -85, 80, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 45, 0);
	motorMask = (1<<2);
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	SET_ANGLES(angles, 20, 0, 0, 0);
	motorMask = (1<<0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Stand turn Right */
	gait = new Gait(L"Stand Turn Right");
	SET_ANGLES(angles, 0, 0, -30, 0);
	motorMask = 1<<2;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Stand turn left */
	gait = new Gait(L"Stand Turn Left");
	SET_ANGLES(angles, 0, 0, 30, 0);
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Right Faceplate Forward */
	gait = new Gait(L"Right Face Forward");
	SET_ANGLES(angles, 0, 0, -10, 0);
	motorMask = 1<<2;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Right Faceplate Backward */
	gait = new Gait(L"Right Face Backward");
	SET_ANGLES(angles, 0, 0, 10, 0);
	motorMask = 1<<2;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Left Faceplate Forward */
	gait = new Gait(L"Left Face Backward");
	SET_ANGLES(angles, 0, 0, 0, -10);
	motorMask = 1<<3;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Left Faceplate Backward */
	gait = new Gait(L"Left Face Backward");
	SET_ANGLES(angles, 0, 0, 0, 10);
	motorMask = 1<<3;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* UnmotionStand */
	gait = new Gait(L"UnmotionStand");
	SET_ANGLES(angles, -85, 80, 0, 0);
	motorMask = 1<<0; motorMask |= 1<<1;
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Skinny */
	gait = new Gait(L"Skinny");
	SET_ANGLES(angles, 85, 80, 0, 0);
	motorMask = 1<<0; motorMask |= 1<<1;
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	return 0;
}

int poseJoints(const double *angles, unsigned char motorMask)
{
	int i;
	for(i = 0; i < 4; i++) {
		if((1<<i) & motorMask) {
			samplebt.moveJointTo(i, angles[i]);
		}
	}
	return 0;
}

int moveJoints(const double *angles, unsigned char motorMask)
{
	int i;
	double angle;
	for(i = 0; i < 4; i++) {
		if((1<<i) & motorMask) {
			samplebt.getJointAngle(i, angle);
			angle += angles[i];
			samplebt.moveJointTo(i, angle);
		}
	}
	return 0;
}

int moveWait()
{
	int i;
	for(i = 0; i < 4; i++) {
		samplebt.moveJointWait(i);
	}
	return 0;
}
