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
  /* Set up the list ctrls */
  listctrl_availableBots
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
}

void CTeachingDialog::OnBnClickedButtonTeachingMoveup()
{
	// TODO: Add your control notification handler code here
}

void CTeachingDialog::OnBnClickedButtonTeachingMovedown()
{
	// TODO: Add your control notification handler code here
}

void CTeachingDialog::OnBnClickedButtonTeachingDisconnect()
{
	// TODO: Add your control notification handler code here
}

void CTeachingDialog::OnBnClickedButtonTeachingRecord()
{
	// TODO: Add your control notification handler code here
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
