#pragma once
#include "afxcmn.h"
#include "DialogConnect.h"
#include "DialogProgram.h"
#include "DialogTeaching.h"
#include "iMobotController_WindowsDlg.h"


// CTabbedDialog dialog

class CTabbedDialog : public CDialog
{
	DECLARE_DYNAMIC(CTabbedDialog)

public:
	CTabbedDialog(CWnd* pParent = NULL);   // standard constructor
	virtual ~CTabbedDialog();
  virtual BOOL OnInitDialog();

// Dialog Data
	enum { IDD = IDD_DIALOG_MAINTABCTRL };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
private:
  RobotManager *m_robotManager;
public:
  CTabCtrl m_tabCtrl;
  CDialogConnect *m_connectDlg;
  CDialogProgram *m_programDlg;
  CDialogTeaching *m_teachingDlg;
  CiMobotController_WindowsDlg  *m_robotControllerDlg;
  RobotManager* GetRobotManager();
  void RefreshTabContent();
  afx_msg void OnTcnSelchangeTab1(NMHDR *pNMHDR, LRESULT *pResult);
};
