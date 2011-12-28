// iMobotController_WindowsDlg.cpp : implementation file
//
#include <stdio.h>
#include <stdlib.h>
#include "stdafx.h"
#include "iMobotController_Windows.h"
#include "iMobotController_WindowsDlg.h"
#include "configFileDialog.h"
#include "gait.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

CRITICAL_SECTION UpdateGuiCriticalSection;
buttonState_t g_buttonState[B_NUMBUTTONS];


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
  m_edit_MotorSpeeds[0] = &m_edit_MotorSpeed1;
  m_edit_MotorSpeeds[1] = &m_edit_MotorSpeed2;
  m_edit_MotorSpeeds[2] = &m_edit_MotorSpeed3;
  m_edit_MotorSpeeds[3] = &m_edit_MotorSpeed4;
	m_slider_Speeds[0] = &m_slider_Speed1;
	m_slider_Speeds[1] = &m_slider_Speed2;
	m_slider_Speeds[2] = &m_slider_Speed3;
	m_slider_Speeds[3] = &m_slider_Speed4;
	m_slider_Positions[0] = &m_slider_Position1;
	m_slider_Positions[1] = &m_slider_Position2;
	m_slider_Positions[2] = &m_slider_Position3;
	m_slider_Positions[3] = &m_slider_Position4;
  /* Initialize critical section */
  InitializeCriticalSection(&UpdateGuiCriticalSection);
  /* Initialize button states */
  g_buttonState[B_PLAY].handlerFunc = &CiMobotController_WindowsDlg::OnBnClickedButtonrollback;
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
	//DDX_Control(pDX, IDC_STATIC_IMOBOT_PICTURE, m_staticicon_iMobotPicture);
	DDX_Control(pDX, IDC_BUTTON_rollForward, m_button_rollForward);
	DDX_Control(pDX, IDC_BUTTON_rollStop, m_button_rollStop);
	DDX_Control(pDX, IDC_BUTTON_rollBack, m_button_rollBack);
	DDX_Control(pDX, IDC_BUTTON_rollLeft, m_button_rollLeft);
	DDX_Control(pDX, IDC_BUTTON_rollRight, m_button_rollRight);
	DDX_Control(pDX, IDC_EDIT_speed1, m_edit_MotorSpeed1);
	DDX_Control(pDX, IDC_EDIT_speed2, m_edit_MotorSpeed2);
	DDX_Control(pDX, IDC_EDIT_speed3, m_edit_MotorSpeed3);
	DDX_Control(pDX, IDC_EDIT_speed4, m_edit_MotorSpeed4);
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
	//ON_STN_CLICKED(IDC_STATIC_IMOBOT_PICTURE, &CiMobotController_WindowsDlg::OnStnClickedStaticImobotPicture)
	ON_BN_CLICKED(IDC_BUTTON_Motor2Forward, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor2forward)
	ON_BN_CLICKED(IDC_BUTTON_Motor3Forward, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor3forward)
	ON_BN_CLICKED(IDC_BUTTON_Motor1Stop, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor1stop)
	ON_BN_CLICKED(IDC_BUTTON_Motor2Stop, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor2stop)
	ON_BN_CLICKED(IDC_BUTTON_Motor3Stop, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor3stop)
	ON_BN_CLICKED(IDC_BUTTON_Motor4Stop, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor4stop)
	ON_BN_CLICKED(IDC_BUTTON_Motor1Backward, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor1backward)
	ON_BN_CLICKED(IDC_BUTTON_Motor2Backward, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor2backward)
	ON_BN_CLICKED(IDC_BUTTON_Motor3Backward, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor3backward)
	ON_BN_CLICKED(IDC_BUTTON_Motor4Backward, &CiMobotController_WindowsDlg::OnBnClickedButtonMotor4backward)
	ON_BN_CLICKED(IDC_BUTTON_rollForward, &CiMobotController_WindowsDlg::OnBnClickedButtonrollforward)
	ON_BN_CLICKED(IDC_BUTTON_rollStop, &CiMobotController_WindowsDlg::OnBnClickedButtonrollstop)
	ON_BN_CLICKED(IDC_BUTTON_rollLeft, &CiMobotController_WindowsDlg::OnBnClickedButtonrollleft)
	ON_BN_CLICKED(IDC_BUTTON_rollRight, &CiMobotController_WindowsDlg::OnBnClickedButtonrollright)
	ON_BN_CLICKED(IDC_BUTTON_rollBack, &CiMobotController_WindowsDlg::OnBnClickedButtonrollback)
	ON_NOTIFY(NM_CUSTOMDRAW, IDC_SLIDER_position2, &CiMobotController_WindowsDlg::OnNMCustomdrawSliderposition2)
	ON_NOTIFY(TRBN_THUMBPOSCHANGING, IDC_SLIDER_position2, &CiMobotController_WindowsDlg::OnTRBNThumbPosChangingSliderposition2)
	ON_NOTIFY(TRBN_THUMBPOSCHANGING, IDC_SLIDER_position1, &CiMobotController_WindowsDlg::OnTRBNThumbPosChangingSliderposition1)
	ON_NOTIFY(TRBN_THUMBPOSCHANGING, IDC_SLIDER_position3, &CiMobotController_WindowsDlg::OnTRBNThumbPosChangingSliderposition3)
	ON_NOTIFY(TRBN_THUMBPOSCHANGING, IDC_SLIDER_position4, &CiMobotController_WindowsDlg::OnTRBNThumbPosChangingSliderposition4)
	ON_COMMAND(ID_ROBOT_CONFIGUREROBOTBLUETOOTH, &CiMobotController_WindowsDlg::OnRobotConfigurerobotbluetooth)
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

  m_button_rollForward.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_up));
  m_button_rollLeft.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_left));
  m_button_rollRight.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_right));
  m_button_rollStop.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_stop));
  m_button_rollBack.SetIcon( AfxGetApp()->LoadIcon(IDI_ICON_down));
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
	double angles[4] = {90, 0, 0, 90};
	unsigned char motorMask = 0;
	motorMask |= (1<<0);
	motorMask |= (1<<3);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Rotate Right */
	gait = new Gait(L"Rotate Right");
	SET_ANGLES(angles, -90, 0, 0, -90);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Roll Forward */
	gait = new Gait(L"Roll Forward");
	SET_ANGLES(angles, 90, 0, 0, -90);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Roll Backward */
	gait = new Gait(L"Roll Backward");
	SET_ANGLES(angles, -90, 0, 0, 90);
	gait->addMotion( new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Arch */
	gait = new Gait(L"Arch");
	SET_ANGLES(angles, 0, -30, 30, 0);
	motorMask = 0;
	motorMask |= (1<<1); motorMask |= (1<<2);
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
	SET_ANGLES(angles, 0, -50, 0, 0);
	motorMask = 0;
	motorMask |= (1<<1); motorMask |= (1<<2);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, -50, 50, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 50, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Inch Left */
	gait = new Gait(L"Inch Left");
	SET_ANGLES(angles, 0, 0, 50, 0);
	motorMask = 0;
	motorMask |= (1<<1); motorMask |= (1<<2);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, -50, 50, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, -50, 0, 0);
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
	motorMask |= (1<<2); motorMask |= (1<<1);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 0, -85, 80, 0);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	SET_ANGLES(angles, 45, 0, 0, 0);
	motorMask = (1<<0);
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	SET_ANGLES(angles, 0, 20, 0, 0);
	motorMask = (1<<1);
	gait->addMotion(new Motion(
		MOTION_POSE, angles, motorMask));
	addGait(gait);

	/* Stand turn Right */
	gait = new Gait(L"Stand Turn Right");
	SET_ANGLES(angles, -30, 0, 0, 0);
	motorMask = 1<<0;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Stand turn left */
	gait = new Gait(L"Stand Turn Left");
	SET_ANGLES(angles, 30, 0, 0, 0);
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);

	/* Right Faceplate Forward */
	gait = new Gait(L"Right Face Forward");
	SET_ANGLES(angles, -10, 0, 0, 0);
	motorMask = 1<<0;
	gait->addMotion(new Motion(
		MOTION_MOVE, angles, motorMask));
	addGait(gait);
  /* TODO: Repair gaits */

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

	/* UnmotionStand */
	gait = new Gait(L"UnmotionStand");
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
  g_buttonState[B_PLAY].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerPlay()
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
	/* Call base class handler */
	CDialog::OnTimer(nIDEvent);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonconnect()
{
	/* "Connect" button clicked */
	if(iMobotComms.connect()) {
		/* Error connecting */
		MessageBox(L"Error connecting to iMobot.", L"Error");
	} else {
		isConnected = true;
		/* Update the positions of all the sliders and such */
		UpdateSliders();
    /* Start the handler thread */
    CreateThread(
        NULL,
        0,
		(LPTHREAD_START_ROUTINE)(HandlerThread),
        this,
        0,
        NULL );
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
	double speed;

	for(int i = 0; i < 4; i++) {
		iMobotComms.getJointAngle((mobotJointId_t)(i+1), position);
		m_slider_Positions[i]->SetPos( (int) RAD2DEG(position) );
		m_positions[i] = (int) position;

		iMobotComms.getJointSpeed((mobotJointId_t)(i+1), speed);
		m_slider_Speeds[i]->SetPos( speed*100 );
		m_speeds[i] = speed*100;
	}
}

void CiMobotController_WindowsDlg::OnLbnSelchangeListgaits()
{
	// TODO: Add your control notification handler code here
}

int CiMobotController_WindowsDlg::poseJoints(const double *angles, unsigned char motorMask)
{
	for(int i = 0; i < 4; i++) {
		if(motorMask & (1<<i)) {
			iMobotComms.moveJointToNB((mobotJointId_t)(i+1), DEG2RAD(angles[i]));
		}
	}
  iMobotComms.moveWait();
	return 0;
}

int CiMobotController_WindowsDlg::moveJoints(const double *angles, unsigned char motorMask)
{
	double pos;
	for(int i = 0; i < 4; i++) {
		if(motorMask & (1<<i)) {
			/* Get the motor position first */
			iMobotComms.getJointAngle((mobotJointId_t)(i+1), pos);
			/* Set the motor to an offset position */
			iMobotComms.moveJointToNB((mobotJointId_t)(i+1), pos + DEG2RAD(angles[i]));
		}
	}
  iMobotComms.moveWait();
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

void CiMobotController_WindowsDlg::OnStnClickedStaticImobotPicture()
{
	// TODO: Add your control notification handler code here
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor1forward()
{
  g_buttonState[B_M1F].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM1F()
{
	/* get the speed from the slider control */
	int speed;
	speed = m_slider_Speed1.GetPos();
	iMobotComms.setJointSpeedRatio(MOBOT_JOINT1, (double)speed/100.0);
	iMobotComms.moveJointContinuousNB(MOBOT_JOINT1, MOBOT_FORWARD);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor2forward()
{
  g_buttonState[B_M2F].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM2F()
{
	/* get the speed from the slider control */
	int speed;
	speed = m_slider_Speed2.GetPos();
	iMobotComms.setJointSpeedRatio(MOBOT_JOINT2, (double)speed/100.0);
	iMobotComms.moveJointContinuousNB(MOBOT_JOINT2, MOBOT_FORWARD);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor3forward()
{
  g_buttonState[B_M3F].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM3F()
{
	/* get the speed from the slider control */
	int speed;
	speed = m_slider_Speed3.GetPos();
	iMobotComms.setJointSpeed(MOBOT_JOINT3, (double)speed/100.0);
	iMobotComms.moveJointContinuousNB(MOBOT_JOINT3, MOBOT_FORWARD);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor4forward()
{
  g_buttonState[B_M4F].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM4F()
{
	/* get the speed from the slider control */
	int speed;
	speed = m_slider_Speed4.GetPos();
	iMobotComms.setJointSpeed(MOBOT_JOINT4, (double)speed/100.0);
	iMobotComms.moveJointContinuousNB(MOBOT_JOINT4, MOBOT_FORWARD);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor1stop()
{
  g_buttonState[B_M1S].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM1S()
{
	iMobotComms.setJointSpeed(MOBOT_JOINT1, 0);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor2stop()
{
  g_buttonState[B_M2S].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM2S()
{
	iMobotComms.setJointSpeed(MOBOT_JOINT2, 0);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor3stop()
{
  g_buttonState[B_M3S].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM3S()
{
	iMobotComms.setJointSpeed(MOBOT_JOINT3, 0);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor4stop()
{
  g_buttonState[B_M4S].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM4S()
{
	iMobotComms.setJointSpeed(MOBOT_JOINT4, 0);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor1backward()
{
  g_buttonState[B_M1B].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM1B()
{
	/* get the speed from the slider control */
	int speed;
	speed = m_slider_Speed1.GetPos();
	iMobotComms.setJointSpeedRatio(MOBOT_JOINT1, (double)speed/100.0);
	iMobotComms.moveJointContinuousNB(MOBOT_JOINT1, MOBOT_BACKWARD);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor2backward()
{
  g_buttonState[B_M2B].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM2B()
{
	/* get the speed from the slider control */
	int speed;
	speed = m_slider_Speed2.GetPos();
	iMobotComms.setJointSpeedRatio(MOBOT_JOINT2, (double)speed/100.0);
	iMobotComms.moveJointContinuousNB(MOBOT_JOINT2, MOBOT_BACKWARD);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor3backward()
{
  g_buttonState[B_M3B].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM3B()
{
	/* get the speed from the slider control */
	int speed;
	speed = m_slider_Speed3.GetPos();
	iMobotComms.setJointSpeedRatio(MOBOT_JOINT3, (double)speed/100.0);
	iMobotComms.moveJointContinuousNB(MOBOT_JOINT3, MOBOT_BACKWARD);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonMotor4backward()
{
  MessageBox(L"Error connecting to iMobot.", L"Error");
  g_buttonState[B_M4B].clicked = 1;
}

void CiMobotController_WindowsDlg::handlerM4B()
{
	/* get the speed from the slider control */
	int speed;
	speed = m_slider_Speed4.GetPos();
	iMobotComms.setJointSpeedRatio(MOBOT_JOINT4, (double)speed/100.0);
	iMobotComms.moveJointContinuousNB(MOBOT_JOINT4, MOBOT_BACKWARD);
}

void CiMobotController_WindowsDlg::OnBnClickedButtonrollforward()
{
	// TODO: Add your control notification handler code here
}

void CiMobotController_WindowsDlg::OnBnClickedButtonrollstop()
{
  iMobotComms.stop();
}

void CiMobotController_WindowsDlg::OnBnClickedButtonrollleft()
{
	// TODO: Add your control notification handler code here
}

void CiMobotController_WindowsDlg::OnBnClickedButtonrollright()
{
	// TODO: Add your control notification handler code here
}

void CiMobotController_WindowsDlg::OnBnClickedButtonrollback()
{
	// TODO: Add your control notification handler code here
}

void CiMobotController_WindowsDlg::OnNMCustomdrawSliderposition2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMCUSTOMDRAW pNMCD = reinterpret_cast<LPNMCUSTOMDRAW>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
}

void CiMobotController_WindowsDlg::OnTRBNThumbPosChangingSliderposition1(NMHDR *pNMHDR, LRESULT *pResult)
{
  MessageBox(L"Error connecting to iMobot.", L"Error");
	// This feature requires Windows Vista or greater.
	// The symbol _WIN32_WINNT must be >= 0x0600.
	NMTRBTHUMBPOSCHANGING *pNMTPC = reinterpret_cast<NMTRBTHUMBPOSCHANGING *>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
  g_buttonState[S_M1P].clicked = 1;
}

void CiMobotController_WindowsDlg::OnTRBNThumbPosChangingSliderposition2(NMHDR *pNMHDR, LRESULT *pResult)
{
	// This feature requires Windows Vista or greater.
  MessageBox(L"Error connecting to iMobot.", L"Error");
	// The symbol _WIN32_WINNT must be >= 0x0600.
	NMTRBTHUMBPOSCHANGING *pNMTPC = reinterpret_cast<NMTRBTHUMBPOSCHANGING *>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
  g_buttonState[S_M2P].clicked = 1;
}

void CiMobotController_WindowsDlg::OnTRBNThumbPosChangingSliderposition3(NMHDR *pNMHDR, LRESULT *pResult)
{
	// This feature requires Windows Vista or greater.
  MessageBox(L"Error connecting to iMobot.", L"Error");
	// The symbol _WIN32_WINNT must be >= 0x0600.
	NMTRBTHUMBPOSCHANGING *pNMTPC = reinterpret_cast<NMTRBTHUMBPOSCHANGING *>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
  g_buttonState[S_M3P].clicked = 1;
}

void CiMobotController_WindowsDlg::OnTRBNThumbPosChangingSliderposition4(NMHDR *pNMHDR, LRESULT *pResult)
{
	// This feature requires Windows Vista or greater.
  MessageBox(L"Error connecting to iMobot.", L"Error");
	// The symbol _WIN32_WINNT must be >= 0x0600.
	NMTRBTHUMBPOSCHANGING *pNMTPC = reinterpret_cast<NMTRBTHUMBPOSCHANGING *>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
  g_buttonState[S_M4P].clicked = 1;
}

DWORD WINAPI HandlerThread(void* arg)
{
  static double lastPosition[4];
  static int lastSpeed[4];
  CiMobotController_WindowsDlg* dlg;
  CMobot *mobot;
  dlg = (CiMobotController_WindowsDlg*) arg;
  mobot = &dlg->iMobotComms;
  /* Initialize values */
  for(int i = 0; i < 4; i++) {
    lastPosition[i] = dlg->m_slider_Positions[i]->GetPos();
    lastSpeed[i] = dlg->m_slider_Speeds[i]->GetPos();
  }
  while(1) {
    /* Perform iMobot update actions */
    if(mobot->isConnected() == false) {
      Sleep(250);
      continue;
    }

    double value;
    wchar_t buf[200];
    /* Check the Speed and Position sliders. If any 
     * of them have moved, send the appropriate message 
     * to the iMobot. */
    double position;
    int speed;
    for(int i = 0; i < 4; i++) {
      mobot->getJointAngle((mobotJointId_t)(i+1), value);
      swprintf(buf, L"%lf", RAD2DEG(value));
      EnterCriticalSection(&UpdateGuiCriticalSection);
      dlg->m_edit_MotorPositions[i]->SetWindowTextW(buf);
      /* See if the position slider has been clicked */
      position = DEG2RAD(dlg->m_slider_Positions[i]->GetPos());
      if(lastPosition[i] != position) {
        mobot->moveJointToPIDNB((mobotJointId_t)(i+1), (double) position);
        g_buttonState[S_M1P + i].clicked = 0;
        lastPosition[i] = position;
      } else {
        /* Move the slider into position */
        dlg->m_slider_Positions[i]->SetPos((int) RAD2DEG(value));
        lastPosition[i] = DEG2RAD(dlg->m_slider_Positions[i]->GetPos());
      }

      /* Check the speed */
      speed = dlg->m_slider_Speeds[i]->GetPos();
      if(speed != dlg->m_speeds[i]) {
        mobot->setJointSpeedRatio((mobotJointId_t)(i+1), (double)speed/100.0);
        dlg->m_speeds[i] = speed;
        swprintf(buf, L"%lf", speed);
        dlg->m_edit_MotorSpeeds[i]->SetWindowTextW(buf);
      }
      LeaveCriticalSection(&UpdateGuiCriticalSection);
    }
    /* Handle any button presses */
    for(int i = 0; i < B_NUMBUTTONS; i++) {
      if(g_buttonState[i].clicked) {
        switch(i) {
          case B_PLAY: dlg->handlerPlay(); break;
          case B_M1F: dlg->handlerM1F(); break;
          case B_M2F: dlg->handlerM2F(); break;
          case B_M3F: dlg->handlerM3F(); break;
          case B_M4F: dlg->handlerM4F(); break;
          case B_M1S: dlg->handlerM1S(); break;
          case B_M2S: dlg->handlerM2S(); break;
          case B_M3S: dlg->handlerM3S(); break;
          case B_M4S: dlg->handlerM4S(); break;
          case B_M1B: dlg->handlerM1B(); break;
          case B_M2B: dlg->handlerM2B(); break;
          case B_M3B: dlg->handlerM3B(); break;
          case B_M4B: dlg->handlerM4B(); break;
          default: break;
        }
        g_buttonState[i].clicked = 0;
      }
    }
  }
}

void CiMobotController_WindowsDlg::OnRobotConfigurerobotbluetooth()
{
  INT_PTR nRet = -1;
  ConfigFileDialog cfDialog;
  nRet = cfDialog.DoModal();
  char path[MAX_PATH];
  switch(nRet) {
    case IDOK:
      iMobotComms.connect();
      break;
    case IDABORT:
    case IDCANCEL:   
    default:  
      1+1;
  }
}
