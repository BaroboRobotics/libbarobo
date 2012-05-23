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

}

CTabbedDialog::~CTabbedDialog()
{
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

  m_connectDlg.Create(CDialogConnect::IDD, this);
  m_programDlg.Create(CDialogProgram::IDD, this);

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


void CTabbedDialog::RefreshTabContent()
{
  /* Get the selected tab */
  int activeTab = m_tabCtrl.GetCurSel();
  m_connectDlg.ShowWindow(activeTab == 0? SW_SHOW : SW_HIDE);
  m_programDlg.ShowWindow(activeTab == 1? SW_SHOW : SW_HIDE);
}

// CTabbedDialog message handlers

void CTabbedDialog::OnTcnSelchangeTab1(NMHDR *pNMHDR, LRESULT *pResult)
{
  RefreshTabContent();
  *pResult = 0;
}
