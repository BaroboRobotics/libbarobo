#pragma once
#include "afxcmn.h"


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
public:
  CTabCtrl m_tabCtrl;
};
