// DialogConnect.cpp : implementation file
//

#include "stdafx.h"
#include "iMobotController_Windows.h"
#include "DialogConnect.h"
#include "TabbedDialog.h"


// CDialogConnect dialog

IMPLEMENT_DYNAMIC(CDialogConnect, CDialog)

CDialogConnect::CDialogConnect(CWnd* pParent /*=NULL*/)
	: CDialog(CDialogConnect::IDD, pParent)
{
  CTabbedDialog* td = (CTabbedDialog*)pParent;
  if(td != NULL) {
    m_mobotManager = td->GetRobotManager();
  } else {
    m_mobotManager = NULL;
  }
}

CDialogConnect::~CDialogConnect()
{
}

BOOL CDialogConnect::OnInitDialog()
{
  CDialog::OnInitDialog();
  return TRUE;
}

void CDialogConnect::refreshLists()
{
	int i;
	USES_CONVERSION;
	/* Clear the list controls */
	m_listCtrl_availableBots.DeleteAllItems();
	/* Populate available bots listctrl */
	for(i = 0; i < m_mobotManager->numEntries(); i++) {
		if(!m_mobotManager->isConnected(i)) {
			m_listCtrl_availableBots.InsertItem(
				m_listCtrl_availableBots.GetItemCount(),
				A2T(m_mobotManager->getEntry(i))
				);
		}
	}
	/* Populate connected bots listctrl */
	m_listCtrl_connectedBots.DeleteAllItems();
	for(i = 0; i < m_mobotManager->numConnected(); i++) {
		m_listCtrl_connectedBots.InsertItem(
			i,
			A2T(m_mobotManager->getConnected(i))
			);
	}
}

void CDialogConnect::DoDataExchange(CDataExchange* pDX)
{
  CDialog::DoDataExchange(pDX);
  DDX_Control(pDX, IDC_LIST_AVAILABLEBOTS, m_listCtrl_availableBots);
  DDX_Control(pDX, IDC_LIST_CONNECTEDBOTS, m_listCtrl_connectedBots);

  /* Set up list controls */
  m_listCtrl_availableBots.InsertColumn(
    0, 
    TEXT("Mobot Address"),
    LVCFMT_LEFT,
    120,
    -1);
  m_listCtrl_connectedBots.InsertColumn(
    0,
    TEXT("Mobot Address"),
    LVCFMT_LEFT,
    120,
    -1);
  refreshLists();
  DDX_Control(pDX, IDC_EDIT_ROBOTADDRESS, m_edit_newRobotAddress);
}


BEGIN_MESSAGE_MAP(CDialogConnect, CDialog)
  ON_BN_CLICKED(IDC_BUTTON_TEACHING_CONNECT, &CDialogConnect::OnBnClickedButtonTeachingConnect)
  ON_BN_CLICKED(IDC_BUTTON_TEACHING_DISCONNECT, &CDialogConnect::OnBnClickedButtonTeachingDisconnect)
  ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEUP, &CDialogConnect::OnBnClickedButtonTeachingMoveup)
  ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEDOWN, &CDialogConnect::OnBnClickedButtonTeachingMovedown)
  ON_BN_CLICKED(IDC_BUTTON_ADDNEWBOT, &CDialogConnect::OnBnClickedButtonAddnewbot)
END_MESSAGE_MAP()


// CDialogConnect message handlers

void CDialogConnect::OnBnClickedButtonTeachingConnect()
{
	/* Get the selected item */
	int index;
  int err;
	index = m_listCtrl_availableBots.GetSelectionMark();
	if(index == -1) {
		return;
	}
	err = m_mobotManager->connect(index);
  if(err) {
    switch (err) {
      case -1:
        MessageBox( TEXT("Error connecting."), TEXT("Error"), MB_OK | MB_ICONINFORMATION );
        break;
      case -2:
        MessageBox( TEXT("Error connecting. Another application is already connected to this Mobot."), TEXT("Error"), MB_OK | MB_ICONINFORMATION );
        break;
      case -3:
        MessageBox( TEXT("Error connecting. Address format is incorrect."), TEXT("Error"), MB_OK | MB_ICONINFORMATION );
        break;
      case -4:
        MessageBox( TEXT("Error connecting. Not enough Mobot address entries in configuration file."), TEXT("Error"), MB_OK | MB_ICONINFORMATION );
        break;
      case -5:
        MessageBox( TEXT("Error connecting. Bluetooth dongle/device not found on local computer."), TEXT("Error"), MB_OK | MB_ICONINFORMATION );
        break;
      case -6:
        MessageBox( TEXT("Error connecting. Mobot firmware version mismatch."), TEXT("Error"), MB_OK | MB_ICONINFORMATION );
        break;
      default:
        MessageBox( TEXT("Error connecting."), TEXT("Error"), MB_OK | MB_ICONINFORMATION );
        break;
    }
  }
	refreshLists();
}

void CDialogConnect::OnBnClickedButtonTeachingDisconnect()
{
	int connectIndex;
	connectIndex = m_listCtrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) { return; }
	m_mobotManager->disconnect(connectIndex);
	refreshLists();
}

void CDialogConnect::OnBnClickedButtonTeachingMoveup()
{
  // TODO: Add your control notification handler code here
}

void CDialogConnect::OnBnClickedButtonTeachingMovedown()
{
  // TODO: Add your control notification handler code here
}

void CDialogConnect::OnBnClickedButtonAddnewbot()
{
  /* Get the text from the correct edit and put it in the config file */
  USES_CONVERSION;
  TCHAR text[256];
  LPCSTR address;
  int count;
  memset(text, 0, sizeof(TCHAR)*256);
  ((DWORD*)text)[0] = 256;
  count = m_edit_newRobotAddress.GetLine(0, text, 255);
  text[count] = (TCHAR)'\0';
  /* Convert from wide char to normal char */
  //size_t convertedChars = 0;
  //size_t origsize = wcslen(text)+1;
  //wcstombs_s(&convertedChars, address, origsize, text, _TRUNCATE);
  address = T2CA(text);
  m_mobotManager->addEntry(address);
  m_mobotManager->write();
  refreshLists();
}
