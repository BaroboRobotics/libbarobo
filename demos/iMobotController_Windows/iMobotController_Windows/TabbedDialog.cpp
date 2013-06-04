// TabbedDialog.cpp : implementation file
//

#include "stdafx.h"
#include "iMobotController_Windows.h"
#include "TabbedDialog.h"


// CTabbedDialog dialog

IMPLEMENT_DYNAMIC(CTabbedDialog, CDialog)

CTabbedDialog::CTabbedDialog(CWnd* pParent /*=NULL*/)
	: CDialog(CTabbedDialog::IDD, pParent)
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
  m_mobotManager = new RobotManager;
  m_mobotManager->read(path);

  m_connectDlg = new CDialogConnect(this);
  m_programDlg = new CDialogProgram(this);
  m_teachingDlg = new CDialogTeaching(this);
  m_mobotControllerDlg = new CiMobotController_WindowsDlg(this);
}

CTabbedDialog::~CTabbedDialog()
{
  delete m_connectDlg;
  delete m_programDlg;
  delete m_teachingDlg;
  delete m_mobotControllerDlg;
}

BOOL CTabbedDialog::OnInitDialog()
{
  CDialog::OnInitDialog();

  CString text = TEXT("Connect");
  m_tabCtrl.InsertItem(
    TCIF_TEXT,
    0,
    text,
    0, 0, 0, 0);
  text = TEXT("Program");
  m_tabCtrl.InsertItem(
    TCIF_TEXT,
    1,
    text,
    0, 0, 0, 0);
  text = TEXT("Pose Teaching");
  m_tabCtrl.InsertItem(
    TCIF_TEXT,
    2,
    text,
    0, 0, 0, 0);
  text = TEXT("Robot Controller");
  m_tabCtrl.InsertItem(
    TCIF_TEXT,
    3,
    text,
    0, 0, 0, 0);

  m_connectDlg->Create(CDialogConnect::IDD, this);
  m_programDlg->Create(CDialogProgram::IDD, this);
  m_teachingDlg->Create(CDialogTeaching::IDD, this);
  m_mobotControllerDlg->Create(CiMobotController_WindowsDlg::IDD, this);

  RefreshTabContent();
  return TRUE;
}

void CTabbedDialog::DoDataExchange(CDataExchange* pDX)
{
  CDialog::DoDataExchange(pDX);
  DDX_Control(pDX, IDC_TAB1, m_tabCtrl);
}


BEGIN_MESSAGE_MAP(CTabbedDialog, CDialog)
  ON_NOTIFY(TCN_SELCHANGE, IDC_TAB1, &CTabbedDialog::OnTcnSelchangeTab1)
END_MESSAGE_MAP()

RobotManager* CTabbedDialog::GetRobotManager()
{
  return m_mobotManager;
}  

void CTabbedDialog::RefreshTabContent()
{
  /* Get the selected tab */
  int activeTab = m_tabCtrl.GetCurSel();
  m_connectDlg->ShowWindow(activeTab == 0? SW_SHOW : SW_HIDE);
  m_programDlg->ShowWindow(activeTab == 1? SW_SHOW : SW_HIDE);
  m_teachingDlg->ShowWindow(activeTab == 2? SW_SHOW : SW_HIDE);
  m_mobotControllerDlg->ShowWindow(activeTab == 3? SW_SHOW : SW_HIDE);
}

// CTabbedDialog message handlers

void CTabbedDialog::OnTcnSelchangeTab1(NMHDR *pNMHDR, LRESULT *pResult)
{
  RefreshTabContent();
  *pResult = 0;
}
