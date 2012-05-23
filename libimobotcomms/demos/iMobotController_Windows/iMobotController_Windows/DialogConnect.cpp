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

}

CDialogConnect::~CDialogConnect()
{
}

void CDialogConnect::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CDialogConnect, CDialog)
END_MESSAGE_MAP()


// CDialogConnect message handlers
