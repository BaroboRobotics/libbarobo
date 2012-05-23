// DialogConnect.cpp : implementation file
//

#include "stdafx.h"
#include "iMobotController_Windows.h"
#include "DialogConnect.h"


// CDialogConnect dialog

IMPLEMENT_DYNAMIC(CDialogConnect, CDialog)

CDialogConnect::CDialogConnect(CWnd* pParent /*=NULL*/)
	: CDialog(CDialogConnect::IDD, pParent)
{
  char path[MAX_PATH];
  /* Read the config file */
  if(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path) != S_OK) 
  {
    /* Could not get the user's app data directory */
  } else {
    //MessageBox((LPCTSTR)path, (LPCTSTR)"Test");
    //fprintf(fp, "%s", path); 
  }
  strcat(path, "\\Barobo.config");
  m_robotManager.read(path);
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
	for(i = 0; i < m_robotManager.numEntries(); i++) {
		if(!m_robotManager.isConnected(i)) {
			m_listCtrl_availableBots.InsertItem(
				m_listCtrl_availableBots.GetItemCount(),
				A2T(m_robotManager.getEntry(i))
				);
		}
	}
	/* Populate connected bots listctrl */
	m_listCtrl_connectedBots.DeleteAllItems();
	for(i = 0; i < m_robotManager.numConnected(); i++) {
		m_listCtrl_connectedBots.InsertItem(
			i,
			A2T(m_robotManager.getConnected(i))
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
}


BEGIN_MESSAGE_MAP(CDialogConnect, CDialog)
  ON_BN_CLICKED(IDC_BUTTON_TEACHING_CONNECT, &CDialogConnect::OnBnClickedButtonTeachingConnect)
  ON_BN_CLICKED(IDC_BUTTON_TEACHING_DISCONNECT, &CDialogConnect::OnBnClickedButtonTeachingDisconnect)
  ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEUP, &CDialogConnect::OnBnClickedButtonTeachingMoveup)
  ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEDOWN, &CDialogConnect::OnBnClickedButtonTeachingMovedown)
END_MESSAGE_MAP()


// CDialogConnect message handlers

void CDialogConnect::OnBnClickedButtonTeachingConnect()
{
	/* Get the selected item */
	int index;
	index = m_listCtrl_availableBots.GetSelectionMark();
	if(index == -1) {
		return;
	}
	m_robotManager.connect(index);
	refreshLists();
}

void CDialogConnect::OnBnClickedButtonTeachingDisconnect()
{
	int connectIndex;
	connectIndex = m_listCtrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) { return; }
	m_robotManager.disconnect(connectIndex);
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
