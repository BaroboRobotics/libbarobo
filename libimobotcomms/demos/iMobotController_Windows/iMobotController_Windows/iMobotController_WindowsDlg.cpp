// iMobotController_WindowsDlg.cpp : implementation file
//
#include <stdio.h>
#include <stdlib.h>
#include "stdafx.h"
#include "iMobotController_Windows.h"
#include "iMobotController_WindowsDlg.h"
#include "gait.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
END_MESSAGE_MAP()


// CiMobotController_WindowsDlg dialog




CiMobotController_WindowsDlg::CiMobotController_WindowsDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CiMobotController_WindowsDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_numGaits = 0;
	isConnected = false;
	/* Initialize lists of controls */
	m_edit_MotorPositions[0] = &m_edit_Motor1Position;
	m_edit_MotorPositions[1] = &m_edit_Motor2Position;
	m_edit_MotorPositions[2] = &m_edit_Motor3Position;
	m_edit_MotorPositions[3] = &m_edit_Motor4Position;
	m_slider_Speeds[0] = &m_slider_Speed1;
	m_slider_Speeds[1] = &m_slider_Speed2;
	m_slider_Speeds[2] = &m_slider_Speed3;
	m_slider_Speeds[3] = &m_slider_Speed4;
	m_slider_Positions[0] = &m_slider_Position1;
	m_slider_Positions[1] = &m_slider_Position2;
	m_slider_Positions[2] = &m_slider_Position3;
	m_slider_Positions[3] = &m_slider_Position4;
}

void CiMobotController_WindowsDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_BUTTON_Motor1Forward, m_button_Motor1Forward);
	DDX_Control(pDX, IDC_BUTTON_Motor2Forward, m_button_Motor2Forward);
	DDX_Control(pDX, IDC_BUTTON_Motor3Forward, m_button_Motor3Forward);
	DDX_Control(pDX, IDC_BUTTON_Motor4Forward, m_button_Motor4Forward);
	DDX_Control(pDX, IDC_BUTTON_Motor1Stop, m_button_Motor1Stop);
	DDX_Control(pDX, IDC_BUTTON_Motor2Stop, m_button_Motor2Stop);
	DDX_Control(pDX, IDC_BUTTON_Motor3Stop, m_button_Motor3Stop);
	DDX_Control(pDX, IDC_BUTTON_Motor4Stop, m_button_Motor4Stop);
	DDX_Control(pDX, IDC_BUTTON_Motor1Backward, m_button_Motor1Backward);
	DDX_Control(pDX, IDC_BUTTON_Motor2Backward, m_button_Motor2Backward);
	DDX_Control(pDX, IDC_BUTTON_Motor3Backward, m_button_Motor3Backward);
	DDX_Control(pDX, IDC_BUTTON_Motor4Backward, m_button_Motor4Backward);
	DDX_Control(pDX, IDC_LIST_gaits, m_list_gaits);
	DDX_Control(pDX, IDC_EDIT_position1, m_edit_Motor1Position);
	DDX_Control(pDX, IDC_EDIT_position2, m_edit_Motor2Position);
	DDX_Control(pDX, IDC_EDIT_position3, m_edit_Motor3Position);
	DDX_Control(pDX, IDC_EDIT_position4, m_edit_Motor4Position);
	//DDX_Control(pDX, IDC_EDIT_address, m_edit_Address);
	//DDX_Control(pDX, IDC_EDIT_channel, m_edit_Channel);
	DDX_Control(pDX, IDC_SLIDER_speed1, m_slider_Speed1);
	DDX_Control(pDX, IDC_SLIDER_speed2, m_slider_Speed2);
	DDX_Control(pDX, IDC_SLIDER_speed3, m_slider_Speed3);
	DDX_Control(pDX, IDC_SLIDER_speed4, m_slider_Speed4);
	DDX_Control(pDX, IDC_SLIDER_position1, m_slider_Position1);
	DDX_Control(pDX, IDC_SLIDER_position2, m_slider_Position2);
	DDX_Control(pDX, IDC_SLIDER_position3, m_slider_Position3);
	DDX_Control(pDX, IDC_SLIDER_position4, m_slider_Position4);
}

BEGIN_MESSAGE_MAP(CiMobotController_WindowsDlg, CDialog)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	//}}AFX_MSG_MAP
	
	ON_BN_CLICKED(IDC_BUTTON_Motor1Forward, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor1forward)
	ON_BN_CLICKED(IDC_BUTTON_play, &CiMobotController_WindowsDlg::OnBnClickedButtonplay)
	ON_BN_CLICKED(IDC_BUTTON_connect, &CiMobotController_WindowsDlg::OnBnClickedButtonconnect)
	ON_BN_CLICKED(IDC_BUTTON_Motor4Forward, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor4forward)
	ON_LBN_SELCHANGE(IDC_LIST_gaits, &CiMobotController_WindowsDlg::OnLbnSelchangeListgaits)
	ON_EN_CHANGE(IDC_EDIT_position4, &CiMobotController_WindowsDlg::OnEnChangeEditposition4)
END_MESSAGE_MAP()


// CiMobotController_WindowsDlg message handlers

BOOL CiMobotController_WindowsDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here

	/* Set button icon(s) */
	InitIcons();
	/* Initialize the Gait list */
	InitGaits();
	/* Init the timer */
	SetTimer(
		1,
		500,
		(TIMERPROC) NULL );
	/* Initialize the sliders */
	InitSliders();

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CiMobotController_WindowsDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CiMobotController_WindowsDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CiMobotController_WindowsDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


void CiMobotController_WindowsDlg::OnBnClickedButtonMotor1forward()
{
	// TODO: Add your control notification handler code here
	m_button_Motor1Forward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_up) );
}

void CiMobotController_WindowsDlg::InitIcons()
{
	m_button_Motor1Forward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_up));
	m_button_Motor2Forward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_up));
	m_button_Motor3Forward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_up));
	m_button_Motor4Forward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_up));

	m_button_Motor1Stop.SetIcon( AfxGetApp()->LoadIconW(IDI_ICON_stop));
	m_button_Motor2Stop.SetIcon( AfxGetApp()->LoadIconW(IDI_ICON_stop));
	m_button_Motor3Stop.SetIcon( AfxGetApp()->LoadIconW(IDI_ICON_stop));
	m_button_Motor4Stop.SetIcon( AfxGetApp()->LoadIconW(IDI_ICON_stop));

	m_button_Motor1Backward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_down));
	m_button_Motor2Backward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_down));
	m_button_Motor3Backward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_down));
	m_button_Motor4Backward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_down));
}

#define SET_ANGLES(angles, a, b, c, d) \
	angles[0] = a; \
	angles[1] = b; \
	angles[2] = c; \
	angles[3] = d

void CiMobotController_WindowsDlg::InitGaits()
{
	/* Add some pre-programmed gaits into the robot */

	/* Rotate Left */
	Gait* gait;
	gait = new Gait(L"Rotate Left");
	double angles[4] = {0, 0, 90, 90};
	unsigned char motorMask = 0;
	motorMask |= (1<<2);
	motorMask |= (1<<3);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Rotate Right */
	gait = new Gait(L"Rotate Right");
	SET_ANGLES(angles, 0, 0, -90, -90);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Roll Forward */
	gait = new Gait(L"Roll Forward");
	SET_ANGLES(angles, 0, 0, 90, -90);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Roll Backward */
	gait = new Gait(L"Roll Backward");
	SET_ANGLES(angles, 0, 0, -90, 90);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Arch */
	gait = new Gait(L"Arch");
	SET_ANGLES(angles, -30, 30, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion( new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Flat */
	gait = new Gait (L"Flat");
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Inch Right */
	gait = new Gait(L"Inch Right");
	SET_ANGLES(angles, -50, 0, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, -50, 50, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 50, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Inch Left */
	gait = new Gait(L"Inch Left");
	SET_ANGLES(angles, 0, 50, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, -50, 50, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, -50, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Stand */
	gait = new Gait(L"Stand");
	SET_ANGLES(angles, 0, 0, 0, 0);
	motorMask = 0;
	motorMask |= (1<<0); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, -85, 80, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 45, 0);
	motorMask = (1<<2);
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	SET_ANGLES(angles, 20, 0, 0, 0);
	motorMask = (1<<0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Stand turn Right */
	gait = new Gait(L"Stand Turn Right");
	SET_ANGLES(angles, 0, 0, -30, 0);
	motorMask = 1<<2;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Stand turn left */
	gait = new Gait(L"Stand Turn Left");
	SET_ANGLES(angles, 0, 0, 30, 0);
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Right Faceplate Forward */
	gait = new Gait(L"Right Face Forward");
	SET_ANGLES(angles, 0, 0, -10, 0);
	motorMask = 1<<2;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Right Faceplate Backward */
	gait = new Gait(L"Right Face Backward");
	SET_ANGLES(angles, 0, 0, 10, 0);
	motorMask = 1<<2;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Left Faceplate Forward */
	gait = new Gait(L"Left Face Backward");
	SET_ANGLES(angles, 0, 0, 0, -10);
	motorMask = 1<<3;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Left Faceplate Backward */
	gait = new Gait(L"Left Face Backward");
	SET_ANGLES(angles, 0, 0, 0, 10);
	motorMask = 1<<3;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Unstand */
	gait = new Gait(L"Unstand");
	SET_ANGLES(angles, -85, 80, 0, 0);
	motorMask = 1<<0; motorMask |= 1<<1;
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Skinny */
	gait = new Gait(L"Skinny");
	SET_ANGLES(angles, 85, 80, 0, 0);
	motorMask = 1<<0; motorMask |= 1<<1;
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Add the names to the listbox */
	for(int i = 0; i < m_numGaits; i++) {
		m_list_gaits.AddString(m_gaits[i]->getName());
	}
}

int CiMobotController_WindowsDlg::addGait(Gait* gait)
{
	m_gaits[m_numGaits] = gait;
	m_numGaits++;
	return 0;
}

void CiMobotController_WindowsDlg::OnBnClickedButtonplay()
{
	/* Get the index of the gait to play */
	int index;
	index = m_list_gaits.GetCurSel();
	if(index == LB_ERR) {
		return;
	}

	int numMotions = m_gaits[index]->getNumMotions();
	const Motion *motion;
	for(int i = 0; i < numMotions; i++) {
		motion = m_gaits[index]->getMotion(i);
		if(motion->getType() == MOTION_POSE) {
			poseJoints(motion->getAngles(), motion->getMotorMask());
		} else if (motion->getType() == MOTION_MOVE) {
			moveJoints(motion->getAngles(), motion->getMotorMask());
		}
		iMobotComms.moveWait();
	}
}

afx_msg void CiMobotController_WindowsDlg::OnTimer(UINT nIDEvent)
{
	/* Perform iMobot update actions */
	if(isConnected == false) {
		/* Call base class handler */
		CDialog::OnTimer(nIDEvent);
		return;
	}

	double value;
	wchar_t buf[200];
	for(int i = 0; i < 4; i++) {
		/* Get all of the motor angles */
		iMobotComms.getMotorPosition(i, value);
		swprintf(buf, L"%lf", value);
		m_edit_MotorPositions[i]->SetWindowTextW(buf);
	}
	/* Check the Speed and Position sliders. If any 
	 * of them have moved, send the appropriate message 
	 * to the iMobot. */
	int position;
	int speed;
	for(int i = 0; i < 4; i++) {
		/* Check the position */
		position = m_slider_Positions[i]->GetPos();
		if(position != m_positions[i]) {
			iMobotComms.setMotorPosition(i, (double) position);
			m_positions[i] = position;
		}

		/* Check the speed */
		speed = m_slider_Speeds[i]->GetPos();
		if(speed != m_speeds[i]) {
			iMobotComms.setMotorSpeed(i, speed);
			m_speeds[i] = speed;
		}
	}
	/* Call base class handler */
	CDialog::OnTimer(nIDEvent);
}
void CiMobotController_WindowsDlg::OnBnClickedButtonconnect()
{
	/* "Connect" button clicked */
#if 0
	/* Get the address */
	wchar_t address[200];
	char paddress[200];
	wchar_t channel[80];
	m_edit_Address.GetLine(0, address);
	m_edit_Channel.GetLine(0, channel);
	/* Try to connect */
	int chan;
	swscanf(channel, L"%d", &chan);
	/* Convert "address" from wchar_t* to char* */
	size_t origsize = wcslen(address) + 1;
	const size_t newsize = 200;
	size_t convertedChars = 0;
	wcstombs_s(&convertedChars, paddress, origsize, address, _TRUNCATE); 
	//if(iMobotComms.connectAddress(paddress, chan)) {
#endif
	if(iMobotComms.connect()) {
		/* Error connecting */
		MessageBox(L"Error connecting to iMobot.", L"Error");
	} else {
		isConnected = true;
		/* Update the positions of all the sliders and such */
		UpdateSliders();
	}
}

void CiMobotController_WindowsDlg::InitSliders()
{
	for(int i = 0; i < 2; i++) {
		m_slider_Speeds[i]->SetRange(0, 100, TRUE);
		m_slider_Speeds[i]->SetPos(50);
		m_slider_Positions[i]->SetRange(-90, 90, TRUE);
		m_slider_Positions[i]->SetPos(0);
	}
	for(int i = 2; i < 4; i++) {
		m_slider_Speeds[i]->SetRange(0, 100, TRUE);
		m_slider_Speeds[i]->SetPos(50);
		m_slider_Positions[i]->SetRange(-180, 180, TRUE);
		m_slider_Positions[i]->SetPos(0);
	}
}

void CiMobotController_WindowsDlg::UpdateSliders()
{
	double position;
	int speed;

	for(int i = 0; i < 4; i++) {
		iMobotComms.getMotorPosition(i, position);
		m_slider_Positions[i]->SetPos( (int) position );
		m_positions[i] = (int) position;

		iMobotComms.getMotorSpeed(i, speed);
		m_slider_Speeds[i]->SetPos( speed );
		m_speeds[i] = speed;
	}
}
void CiMobotController_WindowsDlg::OnBnClickedButtonMotor4forward()
{
	// TODO: Add your control notification handler code here
}

void CiMobotController_WindowsDlg::OnLbnSelchangeListgaits()
{
	// TODO: Add your control notification handler code here
}

int CiMobotController_WindowsDlg::poseJoints(const double *angles, unsigned char motorMask)
{
	for(int i = 0; i < 4; i++) {
		if(motorMask & (1<<i)) {
			iMobotComms.setMotorPosition(i, angles[i]);
		}
	}
	return 0;
}

int CiMobotController_WindowsDlg::moveJoints(const double *angles, unsigned char motorMask)
{
	double pos;
	for(int i = 0; i < 4; i++) {
		if(motorMask & (1<<i)) {
			/* Get the motor position first */
			iMobotComms.getMotorPosition(i, pos);
			/* Set the motor to an offset position */
			iMobotComms.setMotorPosition(i, pos + angles[i]);
		}
	}
	return 0;
}
void CiMobotController_WindowsDlg::OnEnChangeEditposition4()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
}
