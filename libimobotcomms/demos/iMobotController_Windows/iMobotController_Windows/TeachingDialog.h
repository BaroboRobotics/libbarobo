#pragma once
#include "robotManager.h"
#include "afxcmn.h"
#include "afxwin.h"

#ifdef __cplusplus
extern "C" {
#endif

void* playThread(void *arg);

#ifdef __cplusplus
}
#endif


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

public:
	CListCtrl listctrl_availableBots;
	CListCtrl listctrl_connectedBots;
	CListCtrl listctrl_recordedMotions;
	CEdit edit_teachingDelay;
	CButton button_teachingLoopCheck;
	afx_msg void OnBnClickedButtonTeachingConnect();
	afx_msg void OnBnClickedButtonTeachingMoveup();
	afx_msg void OnBnClickedButtonTeachingMovedown();
	afx_msg void OnBnClickedButtonTeachingDisconnect();
	afx_msg void OnBnClickedButtonTeachingRecord();
	afx_msg void OnBnClickedButtonTeachingAdddelay();
	afx_msg void OnBnClickedButtonTeachingDeletepos();
	afx_msg void OnBnClickedButtonTeachingSave();
	void refresh();
	void refreshRecordedMotions(int highlightedIndex);
	afx_msg void OnBnClickedButtonplay();
};
