#pragma once
#include "afxcmn.h"
#include "robotManager.h"
#include "afxwin.h"


// CDialogConnect dialog

class CDialogConnect : public CDialog
{
	DECLARE_DYNAMIC(CDialogConnect)

public:
	CDialogConnect(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDialogConnect();
  virtual BOOL OnInitDialog();
  void refreshLists(); // Refresh the available bots and connect lists

// Dialog Data
	enum { IDD = IDD_DIALOG_CONNECT };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()

private:
  RobotManager* m_robotManager;

public:
  CListCtrl m_listCtrl_availableBots;
  CListCtrl m_listCtrl_connectedBots;
  CEdit m_edit_newRobotAddress;
  afx_msg void OnBnClickedButtonTeachingConnect();
  afx_msg void OnBnClickedButtonTeachingDisconnect();
  afx_msg void OnBnClickedButtonTeachingMoveup();
  afx_msg void OnBnClickedButtonTeachingMovedown();
  afx_msg void OnBnClickedButtonAddnewbot();
};
