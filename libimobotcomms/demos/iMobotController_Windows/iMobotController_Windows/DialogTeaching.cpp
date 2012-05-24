// TeachingDialog.cpp : implementation file
//

#include "stdafx.h"
#include "iMobotController_Windows.h"
#include "DialogTeaching.h"
#include "TabbedDialog.h"

CDialogTeaching *g_teachingDialog;
// CDialogTeaching dialog

IMPLEMENT_DYNAMIC(CDialogTeaching, CDialog)

CDialogTeaching::CDialogTeaching(CWnd* pParent /*=NULL*/)
	: CDialog(CDialogTeaching::IDD, pParent)
{
  CTabbedDialog *td = (CTabbedDialog*)pParent;
  _robotManager = td->GetRobotManager();
  haltPlayFlag = 0;
  isPlaying = 0;
  listctrl_recordedMotions.setContextMenuCallback(&CDialogTeaching::OnRecordedMotionContextMenu, this);
}

CDialogTeaching::~CDialogTeaching()
{
}

void CDialogTeaching::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_LIST_RECORDEDMOTIONS, listctrl_recordedMotions);
	DDX_Control(pDX, IDC_EDIT_TEACHING_DELAY, edit_teachingDelay);
	DDX_Control(pDX, IDC_CHECK_TEACHING_LOOPED, button_teachingLoopCheck);
	/* Set up the list ctrls */
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
  DDX_Control(pDX, IDC_BUTTONCLEARALL, button_clear);
}


BEGIN_MESSAGE_MAP(CDialogTeaching, CDialog)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_CONNECT, &CDialogTeaching::OnBnClickedButtonTeachingConnect)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEUP, &CDialogTeaching::OnBnClickedButtonTeachingMoveup)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_MOVEDOWN, &CDialogTeaching::OnBnClickedButtonTeachingMovedown)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_DISCONNECT, &CDialogTeaching::OnBnClickedButtonTeachingDisconnect)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_RECORD, &CDialogTeaching::OnBnClickedButtonTeachingRecord)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_ADDDELAY, &CDialogTeaching::OnBnClickedButtonTeachingAdddelay)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_DELETEPOS, &CDialogTeaching::OnBnClickedButtonTeachingDeletepos)
	ON_BN_CLICKED(IDC_BUTTON_TEACHING_SAVE, &CDialogTeaching::OnBnClickedButtonTeachingSave)
	ON_BN_CLICKED(IDC_BUTTON_play, &CDialogTeaching::OnBnClickedButtonplay)
	ON_BN_CLICKED(IDC_BUTTON_stop, &CDialogTeaching::OnBnClickedButtonstop)
  ON_BN_CLICKED(IDC_BUTTONCLEARALL, &CDialogTeaching::OnBnClickedButtonclear)
	ON_NOTIFY(LVN_ITEMCHANGED, IDC_LIST_RECORDEDMOTIONS, &CDialogTeaching::OnLvnItemchangedListRecordedmotions)
	ON_NOTIFY(LVN_ITEMACTIVATE, IDC_LIST_RECORDEDMOTIONS, &CDialogTeaching::OnLvnItemActivateListRecordedmotions)
	ON_NOTIFY(LVN_ENDLABELEDIT, IDC_LIST_RECORDEDMOTIONS, &CDialogTeaching::OnLvnEndlabeleditListRecordedmotions)
	ON_NOTIFY(NM_RCLICK, IDC_LIST_RECORDEDMOTIONS, &CDialogTeaching::OnNMRClickListRecordedmotions)
	ON_COMMAND(ID_RENAME_REMOVE, &CDialogTeaching::OnContextRename)
	ON_COMMAND(ID_RENAME_REMOVE32782, &CDialogTeaching::OnContextRemove)
  ON_COMMAND(ID_RECORDED_MOVEUP, &CDialogTeaching::OnRecordPopupMoveup)
  ON_COMMAND(ID_RECORDED_MOVEDOWN, &CDialogTeaching::OnRecordPopupMovedown)
  ON_COMMAND(ID_RECORDED_GOTOPOSE, &CDialogTeaching::OnRecordedGotopose)
END_MESSAGE_MAP()


// CDialogTeaching message handlers

void CDialogTeaching::OnBnClickedButtonTeachingConnect()
{
	// TODO: Add your control notification handler code here
	/* Get the selected item */
	int index;
	index = listctrl_availableBots.GetSelectionMark();
	if(index == -1) {
		return;
	}
	_robotManager->connect(index);
	refresh();
}

void CDialogTeaching::OnBnClickedButtonTeachingMoveup()
{
	int connectIndex = listctrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) {return;}
	if(connectIndex == 0) {return;}
	_robotManager->moveUp(connectIndex);
	refresh();
	listctrl_connectedBots.SetSelectionMark(connectIndex-1);
}

void CDialogTeaching::OnBnClickedButtonTeachingMovedown()
{
	int connectIndex = listctrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) {return;}
	if(connectIndex >= (_robotManager->numConnected()-1)) {return;}
	_robotManager->moveDown(connectIndex);
	refresh();
	listctrl_connectedBots.SetSelectionMark(connectIndex+1);
}

void CDialogTeaching::OnBnClickedButtonTeachingDisconnect()
{
	int connectIndex;
	connectIndex = listctrl_connectedBots.GetSelectionMark();
	if(connectIndex == -1) { return; }
	_robotManager->disconnect(connectIndex);
	refresh();
}

void CDialogTeaching::OnBnClickedButtonTeachingRecord()
{
	int i;
	for(i = 0; i < _robotManager->numConnected(); i++) {
		_robotManager->getMobot(i)->record();
	}
	refreshRecordedMotions(-1);
}

void CDialogTeaching::OnBnClickedButtonTeachingAdddelay()
{
	// TODO: Add your control notification handler code here
	TCHAR buf[80];
	edit_teachingDelay.GetLine(0, buf, 80);
	if(_tcslen(buf) <= 0) { return; }
	double delay;
	_stscanf(buf, TEXT("%lf"), &delay);
	if(delay <= 0) {return;}
	int i;
	for(i = 0; i < _robotManager->numConnected(); i++) {
		_robotManager->getMobot(i)->addDelay(delay);
	}
	refreshRecordedMotions(-1);
}

void CDialogTeaching::OnBnClickedButtonTeachingDeletepos()
{
	/* Get the highlighted line */
	int index;
	index = listctrl_recordedMotions.GetSelectionMark();
	if(index < 0) return;
	DeleteRecordedMotion(index);
}

void CDialogTeaching::DeleteRecordedMotion(int index)
{
	int i;
	for(i = 0; i < _robotManager->numConnected(); i++) {
		_robotManager->getMobot(i)->removeMotion(index);
	}
	refreshRecordedMotions(-1);
}

void CDialogTeaching::OnBnClickedButtonTeachingSave()
{
  bool looped;
  if(button_teachingLoopCheck.GetCheck() == BST_CHECKED) {
    looped = true;
  } else {
    looped = false;
  }
  /* Pop up the save file dialog */
  CFileDialog dlgFile(
      false,
      _T(".ch"),
      NULL,
      OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT,
      _T("Ch Programs (*.ch)|*.ch|Text Files|*.txt|All Files (*.*)|*.*||"));
  if( dlgFile.DoModal() == IDOK ) {
    CString pathname = dlgFile.GetPathName();
    FILE *fp = _tfopen((LPCTSTR)pathname, _T("w"));
    if(fp == NULL) {
			MessageBox( 
          TEXT("Could not save to file."), 
          TEXT("Error"), 
          MB_OK | MB_ICONINFORMATION );
      return;
    }
    CString *program = _robotManager->generateProgram(looped);
    _ftprintf(fp, (LPCTSTR)*program);
    fclose(fp);
    delete program;
  }
}

void CDialogTeaching::refresh()
{
}

void CDialogTeaching::refreshRecordedMotions(int highlightedIndex)
{
	/* Pick the first connect robot in the list, go through all the motions.. */
	TCHAR buf[200];
	if(_robotManager->numConnected() <= 0) {
		return;
	}
	listctrl_recordedMotions.DeleteAllItems();
	int i;
	CRecordMobot *mobot;
	mobot = _robotManager->getMobot(0);
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

void CDialogTeaching::OnBnClickedButtonplay()
{
	THREAD_T thread;
	button_play.EnableWindow(false);
	THREAD_CREATE(&thread, playThread, this);
}

void* playThread(void* arg)
{
	CDialogTeaching *teachingDialog;
	teachingDialog = (CDialogTeaching*)arg;
  teachingDialog->isPlaying = true;
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
  teachingDialog->isPlaying = false;
	teachingDialog->button_play.EnableWindow(true);
	teachingDialog->refreshRecordedMotions(-1);
	return NULL;
}

RobotManager* CDialogTeaching::getRobotManager()
{
	return _robotManager;
}
void CDialogTeaching::OnBnClickedButtonstop()
{
	haltPlayFlag = 1;
}

void CDialogTeaching::OnLvnItemchangedListRecordedmotions(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMLISTVIEW pNMLV = reinterpret_cast<LPNMLISTVIEW>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
}

void CDialogTeaching::OnLvnItemActivateListRecordedmotions(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMITEMACTIVATE pNMIA = reinterpret_cast<LPNMITEMACTIVATE>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
	/*
	listctrl_recordedMotions.EditLabel(
		listctrl_recordedMotions.GetSelectionMark() );
		*/
}

void CDialogTeaching::OnLvnEndlabeleditListRecordedmotions(NMHDR *pNMHDR, LRESULT *pResult)
{
	NMLVDISPINFO *pDispInfo = reinterpret_cast<NMLVDISPINFO*>(pNMHDR);
	// TODO: Add your control notification handler code here
	int i;
	int index = pDispInfo->item.iItem;
	for(i = 0; i < _robotManager->numConnected(); i++) {
		_robotManager->getMobot(i)->setMotionName(
			index,
			pDispInfo->item.pszText );
	}
	refreshRecordedMotions(-1);
	*pResult = 0;
}

void CDialogTeaching::OnNMRClickListRecordedmotions(NMHDR *pNMHDR, LRESULT *pResult)
{
	//LPNMITEMACTIVATE pNMItemActivate = reinterpret_cast<NMITEMACTIVATE>(pNMHDR);
	// TODO: Add your control notification handler code here
	*pResult = 0;
}

void CDialogTeaching::OnRecordedMotionContextMenu(CPoint point, void *arg)
{
	CDialogTeaching *dlg = (CDialogTeaching*)arg;
	CPoint cl_point;
	cl_point = point;
	dlg->listctrl_recordedMotions.ScreenToClient(&cl_point);
	/* We want to pop up a context menu on the cursor, if it is over an item */
	int i, index = -1;
	CRect rect;
	for(i = 0; i < dlg->listctrl_recordedMotions.GetItemCount(); i++) {
		dlg->listctrl_recordedMotions.GetItemRect(i, &rect, LVIR_BOUNDS);
		if(rect.PtInRect(cl_point)) {
			index = i;
			break;
		}
	}
	if(index == -1) {return;}
	dlg->contextMenuIndex = index;
	CMenu mnuPopupMain;
	mnuPopupMain.LoadMenu(IDR_MENU_RECORDEDMOTIONPOPUP);
	CMenu *mnuPopup = mnuPopupMain.GetSubMenu(0);
	mnuPopup->TrackPopupMenu(TPM_LEFTALIGN | TPM_RIGHTBUTTON, point.x, point.y, dlg);
}

void CDialogTeaching::OnContextRename()
{
	listctrl_recordedMotions.EditLabel(contextMenuIndex);
}

void CDialogTeaching::OnContextRemove()
{
	DeleteRecordedMotion(contextMenuIndex);
}
/*
void CDialogTeaching::MoveMotionUp(int index)
{
} */
void CDialogTeaching::OnRecordPopupMoveup()
{
  int i;
  for(i = 0; i < _robotManager->numConnected(); i++) {
    _robotManager->getMobot(i)->moveMotion(contextMenuIndex, contextMenuIndex-1);
  }
  refreshRecordedMotions(-1);
}

void CDialogTeaching::OnRecordPopupMovedown()
{
  int i;
  for(i = 0; i < _robotManager->numConnected(); i++) {
    _robotManager->getMobot(i)->moveMotion(contextMenuIndex, contextMenuIndex+1);
  }
  refreshRecordedMotions(-1);
}

void CDialogTeaching::OnRecordedGotopose()
{
  int i;
  refreshRecordedMotions(contextMenuIndex);
  for(i = 0; i < _robotManager->numConnected(); i++) {
    _robotManager->getMobot(i)->play(contextMenuIndex);
  }
}

void CDialogTeaching::OnMobotButton(CMobot *robot, int button, int buttonDown)
{
  /* Button A: Record Motion 
   * Button B: Playback / Stop
   * Both Buttons: Reset motions */
  static unsigned int lastState = 0;
  unsigned int newState;
  static bool debounce = false;
  int i;

  /* Calculate the new state */
  if(buttonDown) {
    robot->blinkLED(0.1, 2);
    newState = lastState | (1<<button);
  } else {
    newState = lastState & ~(1<<button);
  }
  if(debounce) {
    /* Must wait until all buttons are up */
    if(newState == 0) {
      debounce = false;
      lastState = newState;
      return;
    } else {
      lastState = newState;
      return;
    }
  }

  /* If a button release event is detected, use the last state to determine the action */
  if(buttonDown == 0) {
    if(lastState == 0x01) {
      /* Button A press/release */
      for(i = 0; i < g_teachingDialog->_robotManager->numConnected(); i++) {
		    g_teachingDialog->_robotManager->getMobot(i)->record();
	    }
	    g_teachingDialog->refreshRecordedMotions(-1);
    }
    if(lastState == 0x02) {
      /* Button B press/release */
      /* If it is not playing play. Otherwise, stop. */
      if(g_teachingDialog->isPlaying) {
        g_teachingDialog->haltPlayFlag = 1;
      } else {
        g_teachingDialog->OnBnClickedButtonplay();
      }
    }
    if(lastState == 0x03) {
      /* Buttons A/B Pressed, one released */
      debounce = true;
      g_teachingDialog->OnBnClickedButtonclear();
    }
  }
  lastState = newState;
}

void CDialogTeaching::OnBnClickedButtonclear()
{
  int i;
	for(i = 0; i < _robotManager->numConnected(); i++) {
		_robotManager->getMobot(i)->clearAllMotions();
	}
	refreshRecordedMotions(-1);
}
