// TeachingDialog.cpp : implementation file
//

#include "stdafx.h"
#include "iMobotController_Windows.h"
#include "TeachingDialog.h"


// CTeachingDialog dialog

IMPLEMENT_DYNAMIC(CTeachingDialog, CDialog)

CTeachingDialog::CTeachingDialog(CWnd* pParent /*=NULL*/)
	: CDialog(CTeachingDialog::IDD, pParent)
{
  haltPlayFlag = 0;
  char path[MAX_PATH];
  /* Read the config file */
  if(SHGetFolderPathA(NULL, CSIDL_LOCAL_APPDATA, NULL, 0, path) != S_OK) 
  {
    /* Could not get the user's app data directory */
  } else {
    //MessageBox((LPCTSTR)path, (LPCTSTR)"Test");
    //fprintf(fp, "%s", path); 
  }
  strcat(path, "\\Barobo.config");
  _robotManager.read(path);

}

CTeachingDialog::~CTeachingDialog()
{
}

void CTeachingDialog::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST_AVAILABLEBOTS, listctrl_availableBots);
	DDX_Control(pDX, IDC_LIST_CONNECTEDBOTS, listctrl_connectedBots);
	DDX_Control(pDX, IDC_LIST_RECORDEDMOTIONS, listctrl_recordedMotions);
	DDX_Control(pDX, IDC_EDIT_TEACHING_DELAY, edit_teachingDelay);
	DDX_Control(pDX, IDC_CHECK_TEACHING_LOOPED, button_teachingLoopCheck);
	/* Set up the list ctrls */
	listctrl_availableBots.InsertColumn(
		0, 
		TEXT("Mobot Address"),
		LVCFMT_LEFT,
		120,
		-1);
	listctrl_connectedBots.InsertColumn(
		0,
		TEXT("Mobot Address"),
		LVCFMT_LEFT,
		120,
		-1);
	listctrl_recordedMotions.InsertColumn(
		0,
		TEXT("Recorded Motions"),
		LVCFMT_LEFT,
		120,
		-1);
	//listctrl_availableBots.InsertItem(0, TEXT("Test Item"));
	refresh();
	DDX_Control(pDX, IDC_BUTTON_play, button_play);
	DDX_Control(pDX, IDC_BUTTON_stop, button_stop);
}


BEGIN_MESSAGE_MAP(CTeachingDialog, CDialog)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_CONNECT, &CTeachingDialog::OnBnClickedButtonTeachingConnect)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEUP, &CTeachingDialog::OnBnClickedButtonTeachingMoveup)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEDOWN, &CTeachingDialog::OnBnClickedButtonTeachingMovedown)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_DISCONNECT, &CTeachingDialog::OnBnClickedButtonTeachingDisconnect)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_RECORD, &CTeachingDialog::OnBnClickedButtonTeachingRecord)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_ADDDELAY, &CTeachingDialog::OnBnClickedButtonTeachingAdddelay)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_DELETEPOS, &CTeachingDialog::OnBnClickedButtonTeachingDeletepos)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_SAVE, &CTeachingDialog::OnBnClickedButtonTeachingSave)
	ON_BN_CLICKED(IDC_BUTTON_play, &CTeachingDialog::OnBnClickedButtonplay)
	ON_BN_CLICKED(IDC_BUTTON_stop, &CTeachingDialog::OnBnClickedButtonstop)
	ON_NOTIFY(LVN_ITEMCHANGED, IDC_LIST_RECORDEDMOTIONS, &CTeachingDialog::OnLvnItemchangedListRecordedmotions)
	ON_NOTIFY(LVN_ITEMACTIVATE, IDC_LIST_RECORDEDMOTIONS, &CTeachingDialog::OnLvnItemActivateListRecordedmotions)
	ON_NOTIFY(LVN_ENDLABELEDIT, IDC_LIST_RECORDEDMOTIONS, &CTeachingDialog::OnLvnEndlabeleditListRecordedmotions)
END_MESSAGE_MAP()


// CTeachingDialog message handlers

void CTeachingDialog::OnBnClickedButtonTeachingConnect()
{
	// TODO: Add your control notification handler code here
	/* Get the selected item */
	int index;
	index = listctrl_availableBots.GetSelectionMark();
	if(index == -1) {
		return;
	}
	_robotManager.connect(index);
	refresh();
}

void CTeachingDialog::OnBnClickedButtonTeachingMoveup()
{
	int connectIndex = listctrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) {return;}
	if(connectIndex == 0) {return;}
	_robotManager.moveUp(connectIndex);
	refresh();
	listctrl_connectedBots.SetSelectionMark(connectIndex-1);
}

void CTeachingDialog::OnBnClickedButtonTeachingMovedown()
{
	int connectIndex = listctrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) {return;}
	if(connectIndex >= (_robotManager.numConnected()-1)) {return;}
	_robotManager.moveDown(connectIndex);
	refresh();
	listctrl_connectedBots.SetSelectionMark(connectIndex+1);
}

void CTeachingDialog::OnBnClickedButtonTeachingDisconnect()
{
	int connectIndex;
	connectIndex = listctrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) { return; }
	_robotManager.disconnect(connectIndex);
	refresh();
}

void CTeachingDialog::OnBnClickedButtonTeachingRecord()
{
	int i;
	for(i = 0; i < _robotManager.numConnected(); i++) {
		_robotManager.getMobot(i)->record();
	}
	refreshRecordedMotions(-1);
}

void CTeachingDialog::OnBnClickedButtonTeachingAdddelay()
{
	// TODO: Add your control notification handler code here
	TCHAR buf[80];
	edit_teachingDelay.GetLine(0, buf, 80);
	if(_tcslen(buf) <= 0) { return; }
	double delay;
	_stscanf(buf, TEXT("%lf"), &delay);
	if(delay <= 0) {return;}
	int i;
	for(i = 0; i < _robotManager.numConnected(); i++) {
		_robotManager.getMobot(i)->addDelay(delay);
	}
	refreshRecordedMotions(-1);
}

void CTeachingDialog::OnBnClickedButtonTeachingDeletepos()
{
	/* Get the highlighted line */
	int index;
	index = listctrl_recordedMotions.GetSelectionMark();
	if(index < 0) return;
	int i;
	for(i = 0; i < _robotManager.numConnected(); i++) {
		_robotManager.getMobot(i)->removeMotion(index);
	}
	refreshRecordedMotions(-1);
}

void CTeachingDialog::OnBnClickedButtonTeachingSave()
{
	// TODO: Add your control notification handler code here
}

void CTeachingDialog::refresh()
{
	int i;
	USES_CONVERSION;
	/* Clear the list controls */
	listctrl_availableBots.DeleteAllItems();
	/* Populate available bots listctrl */
	for(i = 0; i < _robotManager.numEntries(); i++) {
		if(!_robotManager.isConnected(i)) {
			listctrl_availableBots.InsertItem(
				listctrl_availableBots.GetItemCount(),
				A2T(_robotManager.getEntry(i))
				);
		}
	}
	/* Populate connected bots listctrl */
	listctrl_connectedBots.DeleteAllItems();
	for(i = 0; i < _robotManager.numConnected(); i++) {
		listctrl_connectedBots.InsertItem(
			i,
			A2T(_robotManager.getConnected(i))
			);
	}
}

void CTeachingDialog::refreshRecordedMotions(int highlightedIndex)
{
	/* Pick the first connect robot in the list, go through all the motions.. */
	TCHAR buf[200];
	if(_robotManager.numConnected() <= 0) {
		return;
	}
	listctrl_recordedMotions.DeleteAllItems();
	int i;
	CRecordMobot *mobot;
	mobot = _robotManager.getMobot(0);
	for(i = 0; i < mobot->numMotions(); i++) {
		if(i == highlightedIndex) {
			swprintf(buf, TEXT("%s <--"), mobot->getMotionName(i)); 
			listctrl_recordedMotions.InsertItem(
				listctrl_recordedMotions.GetItemCount(),
				buf
				);
		} else {
			listctrl_recordedMotions.InsertItem(
				listctrl_recordedMotions.GetItemCount(),
				mobot->getMotionName(i)
				);
		}
	}
	if(highlightedIndex >= 0) {
		listctrl_recordedMotions.SetHotItem(highlightedIndex);
	}
}

void CTeachingDialog::OnBnClickedButtonplay()
{
	THREAD_T thread;
	button_play.EnableWindow(false);
	THREAD_CREATE(&thread, playThread, this);
}

void* playThread(void* arg)
{
	CTeachingDialog *teachingDialog;
	teachingDialog = (CTeachingDialog*)arg;
	RobotManager* robotManager;
	robotManager = teachingDialog->getRobotManager();
	int i, j, done;
	done = 0;
	for(i = 0; !done ; i++) {
		teachingDialog->refreshRecordedMotions(i);
		for(j = 0; j < robotManager->numConnected(); j++) {
			if(robotManager->getMobot(j)->getMotionType(i) == MOTION_SLEEP) {
				robotManager->getMobot(j)->play(i);
				break;
			}
			if(robotManager->getMobot(j)->play(i)) {
				if(teachingDialog->button_teachingLoopCheck.GetCheck() == BST_CHECKED) {
					i = -1;
					break;
				}	else {
					done = 1;
					break;
				}
			}
		}
		for(j = 0; j < robotManager->numConnected(); j++) {
			robotManager->getMobot(j)->moveWait();
		}
		if(teachingDialog->haltPlayFlag) {
			teachingDialog->haltPlayFlag = 0;
			break;
		}
	}
	for(j = 0; j < robotManager->numConnected(); j++) {
		robotManager->getMobot(j)->stop();
	}
	teachingDialog->button_play.EnableWindow(true);
	teachingDialog->refreshRecordedMotions(-1);
	return NULL;
}

RobotManager* CTeachingDialog::getRobotManager()
{
	return &_robotManager;
}
void CTeachingDialog::OnBnClickedButtonstop()
{
	haltPlayFlag = 1;
}

void CTeachingDialog::OnLvnItemchangedListRecordedmotions(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
}

void CTeachingDialog::OnLvnItemActivateListRecordedmotions(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMIA = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
	/*
	listctrl_recordedMotions.EditLabel(
		listctrl_recordedMotions.GetSelectionMark() );
		*/
}

void CTeachingDialog::OnLvnEndlabeleditListRecordedmotions(NMHDR *pNMHDR, LRESULT *pResult)
{
	NMLVDISPINFO *pDispInfo = reinterpret_cast<NMLVDISPINFO*>(pNMHDR);
	// TODO: Add your control notification handler code here
	int i;
	int index = pDispInfo->item.iItem;
	for(i = 0; i < _robotManager.numConnected(); i++) {
		_robotManager.getMobot(i)->setMotionName(
			index,
			pDispInfo->item.pszText );
	}
	refreshRecordedMotions(-1);
	*pResult = 0;
}
