#pragma once
#include "robotManager.h"

// CTeachingDialog dialog

class CTeachingDialog : public CDialog
{
	DECLARE_DYNAMIC(CTeachingDialog)

public:
	CTeachingDialog(CWnd* pParent = NULL);   // standard constructor
	virtual ~CTeachingDialog();

// Dialog Data
	enum { IDD = IDD_DIALOG_TEACHING };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
  RobotManager _robotManager;

	DECLARE_MESSAGE_MAP()

};
