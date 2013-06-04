// DialogProgram.cpp : implementation file
//

#include "stdafx.h"
#include "iMobotController_Windows.h"
#include "DialogProgram.h"


// CDialogProgram dialog

IMPLEMENT_DYNAMIC(CDialogProgram, CDialog)

CDialogProgram::CDialogProgram(CWnd* pParent /*=NULL*/)
	: CDialog(CDialogProgram::IDD, pParent)
{

}

CDialogProgram::~CDialogProgram()
{
}

void CDialogProgram::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CDialogProgram, CDialog)
END_MESSAGE_MAP()


// CDialogProgram message handlers
