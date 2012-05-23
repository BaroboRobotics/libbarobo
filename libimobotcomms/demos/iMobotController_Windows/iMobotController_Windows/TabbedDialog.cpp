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
  return TRUE;
}

void CTabbedDialog::DoDataExchange(CDataExchange* pDX)
{
  CDialog::DoDataExchange(pDX);
  DDX_Control(pDX, IDC_TAB1, m_tabCtrl);
}


BEGIN_MESSAGE_MAP(CTabbedDialog, CDialog)
END_MESSAGE_MAP()


// CTabbedDialog message handlers
