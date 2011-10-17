// iMobotController_WindowsDlg.h : header file
//

#pragma once
#include "afxwin.h"
#include "gait.h"
#include <imobotcomms.h>
#include "afxcmn.h"

#define IDT_TIMER1 100

// CiMobotController_WindowsDlg dialog
class CiMobotController_WindowsDlg : public CDialog
{
// Construction
public:
	CiMobotController_WindowsDlg(CWnd* pParent = NULL);	// motionStandard constructor

// Dialog Data
	enum { IDD = IDD_IMOBOTCONTROLLER_WINDOWS_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
  CMobot iMobotComms;
	HICON m_hIcon;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
	bool isConnected;
  Gait* m_gaits[50];
  int m_numGaits;
  int m_positions[4]; // Store last known slider position
  int m_speeds[4];    // Store last known slider position
	void InitIcons();
  void InitGaits();
  void InitSliders();
  void UpdateSliders();
  int addGait(Gait* gait);
  int poseJoints(const double *angles, unsigned char motorMask);
  int moveJoints(const double *angles, unsigned char motorMask);
  
public:
	afx_msg void OnNMCustomdrawSlider3(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnBnClickedButtonMotor1forward();
	afx_msg void OnTimer(UINT nIDEvent);
	CButton m_button_Motor1Forward;
	CButton m_button_Motor2Forward;
	CButton m_button_Motor3Forward;
	CButton m_button_Motor4Forward;
	CButton m_button_Motor1Stop;
	CButton m_button_Motor2Stop;
	CButton m_button_Motor3Stop;
	CButton m_button_Motor4Stop;
	CButton m_button_Motor1Backward;
	CButton m_button_Motor2Backward;
	CButton m_button_Motor3Backward;
	CButton m_button_Motor4Backward;
	CListBox m_list_gaits;
	afx_msg void OnBnClickedButtonplay();
	CEdit m_edit_Motor1Position;
	CEdit m_edit_Motor2Position;
	CEdit m_edit_Motor3Position;
	CEdit m_edit_Motor4Position;
	CEdit *m_edit_MotorPositions[4];
	afx_msg void OnBnClickedButtonconnect();
	CEdit m_edit_Address;
	CEdit m_edit_Channel;
	CSliderCtrl m_slider_Speed1;
	CSliderCtrl m_slider_Speed2;
	CSliderCtrl m_slider_Speed3;
	CSliderCtrl m_slider_Speed4;
	CSliderCtrl m_slider_Position1;
	CSliderCtrl m_slider_Position2;
	CSliderCtrl m_slider_Position3;
	CSliderCtrl m_slider_Position4;
	CSliderCtrl *m_slider_Speeds[4];
	CSliderCtrl *m_slider_Positions[4];
	afx_msg void OnBnClickedButtonMotor4forward();
	afx_msg void OnLbnSelchangeListgaits();
	afx_msg void OnEnChangeEditposition4();
};
