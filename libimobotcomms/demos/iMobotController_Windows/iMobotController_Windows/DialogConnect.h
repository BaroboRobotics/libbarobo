#pragma once


// CDialogConnect dialog

class CDialogConnect : public CDialog
{
	DECLARE_DYNAMIC(CDialogConnect)

public:
	CDialogConnect(CWnd* pParent = NULL);   // standard constructor
	virtual ~CDialogConnect();

// Dialog Data
	enum { IDD = IDD_DIALOG_CONNECT };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	DECLARE_MESSAGE_MAP()
};
