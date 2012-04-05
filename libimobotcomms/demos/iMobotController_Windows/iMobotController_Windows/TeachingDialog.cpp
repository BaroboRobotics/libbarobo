// TeachingDialog.cpp : implementation file
//

#include "stdafx.h"
#include "iMobotController_Windows.h"
#include "TeachingDialog.h"


// CTeachingDialog dialog

IMPLEMENT_DYNAMIC(CTeachingDialog, CDialog)

CTeachingDialog::CTeachingDialog(CWnd* pParent /*=NULL*/)
	: CDialog(CTeachingDialog::IDD, pParent)
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
  _robotManager.read(path);

}

CTeachingDialog::~CTeachingDialog()
{
}

void CTeachingDialog::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST_AVAILABLEBOTS, listctrl_availableBots);
	DDX_Control(pDX, IDC_LIST_CONNECTEDBOTS, listctrl_connectedBots);
	DDX_Control(pDX, IDC_LIST_RECORDEDMOTIONS, listctrl_recordedMotions);
	DDX_Control(pDX, IDC_EDIT_TEACHING_DELAY, edit_teachingDelay);
	DDX_Control(pDX, IDC_CHECK_TEACHING_LOOPED, button_teachingLoopCheck);
	/* Set up the list ctrls */
	listctrl_availableBots.InsertColumn(
		0, 
		TEXT("Mobot Address"),
		LVCFMT_LEFT,
		120,
		-1);
	listctrl_connectedBots.InsertColumn(
		0,
		TEXT("Mobot Address"),
		LVCFMT_LEFT,
		120,
		-1);
	//listctrl_availableBots.InsertItem(0, TEXT("Test Item"));
	refresh();
}


BEGIN_MESSAGE_MAP(CTeachingDialog, CDialog)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_CONNECT, &CTeachingDialog::OnBnClickedButtonTeachingConnect)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEUP, &CTeachingDialog::OnBnClickedButtonTeachingMoveup)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEDOWN, &CTeachingDialog::OnBnClickedButtonTeachingMovedown)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_DISCONNECT, &CTeachingDialog::OnBnClickedButtonTeachingDisconnect)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_RECORD, &CTeachingDialog::OnBnClickedButtonTeachingRecord)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_ADDDELAY, &CTeachingDialog::OnBnClickedButtonTeachingAdddelay)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_DELETEPOS, &CTeachingDialog::OnBnClickedButtonTeachingDeletepos)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_SAVE, &CTeachingDialog::OnBnClickedButtonTeachingSave)
END_MESSAGE_MAP()


// CTeachingDialog message handlers

void CTeachingDialog::OnBnClickedButtonTeachingConnect()
{
	// TODO: Add your control notification handler code here
	/* Get the selected item */
	int index;
	index = listctrl_availableBots.GetSelectionMark();
	if(index == -1) {
		return;
	}
	_robotManager.connect(index);
	refresh();
}

void CTeachingDialog::OnBnClickedButtonTeachingMoveup()
{
	int connectIndex = listctrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) {return;}
	if(connectIndex == 0) {return;}
	_robotManager.moveUp(connectIndex);
	refresh();
	listctrl_connectedBots.SetSelectionMark(connectIndex-1);
}

void CTeachingDialog::OnBnClickedButtonTeachingMovedown()
{
	int connectIndex = listctrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) {return;}
	if(connectIndex >= (_robotManager.numConnected()-1)) {return;}
	_robotManager.moveDown(connectIndex);
	refresh();
	listctrl_connectedBots.SetSelectionMark(connectIndex+1);
}

void CTeachingDialog::OnBnClickedButtonTeachingDisconnect()
{
	int connectIndex;
	connectIndex = listctrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) { return; }
	_robotManager.disconnect(connectIndex);
	refresh();
}

void CTeachingDialog::OnBnClickedButtonTeachingRecord()
{
	int i;
	for(i = 0; i < _robotManager.numConnected(); i++) {
		_robotManager.getMobot(i)->record();
	}
}

void CTeachingDialog::OnBnClickedButtonTeachingAdddelay()
{
	// TODO: Add your control notification handler code here
}

void CTeachingDialog::OnBnClickedButtonTeachingDeletepos()
{
	// TODO: Add your control notification handler code here
}

void CTeachingDialog::OnBnClickedButtonTeachingSave()
{
	// TODO: Add your control notification handler code here
}

void CTeachingDialog::refresh()
{
	int i;
	USES_CONVERSION;
	/* Clear the list controls */
	listctrl_availableBots.DeleteAllItems();
	/* Populate available bots listctrl */
	for(i = 0; i < _robotManager.numEntries(); i++) {
		if(!_robotManager.isConnected(i)) {
			listctrl_availableBots.InsertItem(
				listctrl_availableBots.GetItemCount(),
				A2T(_robotManager.getEntry(i))
				);
		}
	}
	/* Populate connected bots listctrl */
	listctrl_connectedBots.DeleteAllItems();
	for(i = 0; i < _robotManager.numConnected(); i++) {
		listctrl_connectedBots.InsertItem(
			i,
			A2T(_robotManager.getConnected(i))
			);
	}
}