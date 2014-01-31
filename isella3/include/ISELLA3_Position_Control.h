/************************************************************************
Fraunhofer IPA, 2011
ISELLA3 - Position Control 

Author: Attila Achenbach, email: attila.achenbach@googlemail.com
************************************************************************/

// ISELLA3 Position Control.h : main header file for the ISELLA3 application
//
// This file defines the position control and currently also initializes the
// different static parameters - ANNOTATION: This last functionality will be
// moved to loading the parameters from a .cfg file

// NOTE: Indexing is done as follows:
// '-> ArmModules are numbered starting from the base
// '-> ArmModule axes are (currently) numbered 0 for the turning head axis and 1 for the bending axis (!)
// '-> Armmodule components are numbered 0 for left, 1 for right (following the connection sequence of the EPOS2 motor controllers)
// '-> Limit Switches are numbered 1 for "Negative" (back), 2 for "Positive" (front)

// NOTE: Error/Exception Handling is done as follows:
// '-> Using Try, Throw, Catch statements
// '-> On exception, dwErrorCode is thrown.
// '-> Exception objects are caught in ISELLA3PositionControlApp and a central Error Handler is called.

// NOTE: Process Data Monitoring
// '-> file name defined in "ISELLA3 Position Control.h"
// '-> <fstream> for file IO included in "ISELLA3 Position Control.cpp"
// '-> file opened in CISELLA3PositionControlApp::Move()
// '-> Start timestamp and WriteProcessData() called in CISELLA3PositionControlApp::OnTimer()
// '-> file closed in CISELLA3PositionControlApp::Halt()

#pragma once

//#ifndef __AFXWIN_H__
//	#error "include 'stdafx.h' before including this file for PCH"
//#endif

// Includes by IDE
#include "resource.h"		// main symbols

// Includes added by AA
#include "ISELLA3_ArmModule.h" // for the object ArmModule and components
#include "ISELLA3_Process_Monitor.h" // for the object ProcessMonitor and methods
//#include "ISELLA3 Sine Generator.h" // for the object SineGenerator and methods
//#include "stdafx.h"
#include <sys/time.h>
#include "Definitions.h" 
using namespace std;


// controlling function for move thread // TODO: Remove if no threading applied
//UINT pfnThreadProcMove(LPVOID pParamMove);


// CISELLA3PositionControlApp:
// See ISELLA3 Position Control.cpp for the implementation of this class
//

class CISELLA3PositionControlApp //: public CWinApp, CDialog // Reads Actual_Angles and uses DH to calculate Actual_TCP, or reads Target_TCP and uses DH to calculate Target_Angles
{
	//friend UINT pfnThreadProcMove(LPVOID pParamMove); // TODO: Remove im threads no threading applied

	// TODO: Initialization -> InitInstance()
	// '-> TODO: Initialize objects (Parameters from file: ArmModules, incl. all sub-objects = EPOS, Motors, Brakes, Sensors)
	// '-> TODO: Initialize communication to EPOS (return ERROR_Code for error identification)

	// Get private variables and provide for GUI (Show StatusEPOS, StatusMotor, MotorOpMode, StatusBrakes, StatusLimitSwitches, ActualAngles, TargetAngles, ActualPosition, StatusFlags, etc.) // necessary? or better direct from support classes?
	// '-> EPOS2.GetStatus
	// '-> Motor.GetStatus
	// '-> Motor.GetOpMode
	// '-> Brake.GetStatus
	// '-> LimitSwitch.GetStatus
	// '-> AngleSensor.GetActualAngles
	// '-> ISELLA3PositionControlDlg.GetTargetAngles // Better: Dlg writes directly to public target vector. // public? can there be conflicts if public?
	// '-> ISELLA3PositionControl.GetAcualTCP
	// '-> GetStatusFlags...

	// Security System (Check LimitSwitches, AngleLimits, CurrentLimit)
	// '-> ReadLimitSwitches // Read LimitSwitch vector
	// '-> CheckAngleLimits(from ActualAngles-AngleLimitParam)
	// '-> CheckCurrentLimit

	// Kinematics (Angles, for all axes, Positions, Actual, Target) -> CISELLA3PositionControlApp()
	// '-> ReadActualAngles() from EPOS
	//     '-> CalculateActualPosition(from ActualAngles) // direct kinematics (DH)
	// '-> ReadTargetPosition() from GUI // or public: TargetPosition
	//     '-> CalculateTargetAngles(from TargetPosition) // inverse kinematics (DH)
	// '-> '-> CalculateTargetCurrent(from ActualAngles-TargetAngles)

// Constructor -> no initialization here; initialization in InitInstance()
public:
	CISELLA3PositionControlApp();

// Overrides
bool InitInstance(float kp1,float ki1,float kd1,float kp2,float ki2,float kd2,float kp3,float ki3,float kd3,float kp4,float ki4,float kd4);	// Initialize objects (ArmModules) and CommunicationSettings (Data for both from parameters.dat)

//// Implementation
private:
	// ISELLA3 Position Control state ( NOT_CONNECTED || CONNECTED || READY || MOVING || EXCEPTION )
	enum ModuleState e_PosCtrlState;
	// for timing
	timeval Time;
	timeval start_time;
	unsigned long ulTime;

	// for error handling
	unsigned int dwErrorCode;
	string strErrorMessage;
	// Handle for Communication with Motor Controllers
	void* hKeyHandle;

	// robot parameters
public:
	const static int iNbOfArmModules = 2; // number of arm modules
private:
	CISELLA3_ArmModule ArmModule[iNbOfArmModules]; // Initialization of arm modules // TODO: Implement with 'new' operator to dynamically allocate just the necessary memory at runtime

public:
	// Getters and Setters
	unsigned long ulGetTime();
	unsigned int dwGetErrorCode();
	void* hGetKeyHandle();
	enum ModuleState e_GetPosCtrlState();
	enum ModuleState e_GetArmModuleState(int iArmModuleId);
	enum ModuleState e_GetMotorControllerModuleState(int iArmModuleId, int iMotorControllerId);
	string strGetMotorControllerState(int iArmModuleId, int iMotorControllerId);
	string strUpdateMotorControllerState(int iArmModuleId, int iMotorControllerId);
	string strGetOpMode(int iArmModuleId, int iMotorControllerId);
	string strUpdateOpMode(int iArmModuleId, int iMotorControllerId);
	string strGetBrakeStatus(int iArmModuleId, int iMotorControllerId);
	string strUpdateBrakeStatus(int iArmModuleId, int iMotorControllerId);
	short sGetCurrentIs(int iArmModuleId, int iMotorControllerId);
	float fGetAngleAxisIs(int iArmModuleId, int iAxisId);
	float fGetAngleAxisMust(int iArmModuleId, int iAxisId);
	void SetAngleMust(int iArmModuleId, int iAxisId, float fValue); // on change
	void OpenDevice(); // Open port + initialize motors
	void CloseDevice();
	void Enable();
	void Disable();
	void Move();
	void Move(int iArmModuleId);
	void Halt();
	void Halt(int iArmModuleId);
	void OnTimer(unsigned int nIDEvent);
	void EnterManualMode();
	void EnterManualMode(int iArmModuleId);
	void OpenBrakes();
	void CloseBrakes();
private:
	void ResetControl(int iArmModuleId);
	void ControlLoop(int iArmModuleId);
	void ErrorHandler(unsigned int dwErrorCode);

public: // for testing... // TODO: Remove after testing

	// Help Variables
	// '-> for timing
	float fTime;
	float fTimestamp[iNbOfArmModules][2];	//SA -> INDICATORE TEMPO
	float fTimeElapsed[iNbOfArmModules][2][10];	//SA -> TEMPO TRASCORSO
	// '-> for control
	float fPositionActualBuffer[iNbOfArmModules][2][11][6]; // Matrix [time, position_measured, position_smoothed, velocity, acceleration, jerk]
	float fPositionActual[iNbOfArmModules][2][2];
	float fPositionTarget[iNbOfArmModules][2][3];
	float fPositionError[iNbOfArmModules][2][2];
	float fPositionErrorSum[iNbOfArmModules][2];		// For I control // Can be removed if P controller used for position control
	float fPositionErrorDiff[iNbOfArmModules][2];		// For D control // Can be removed if P controller used for position control
	//float fVelocityActualFilter[iNbOfArmModules][2][5];
	/*float fVelocityActual[iNbOfArmModules][2][2];
	float fVelocityTarget[iNbOfArmModules][2];
	float fVelocityError[iNbOfArmModules][2][2];
	float fVelocityErrorSum[iNbOfArmModules][2];		// For I control
	//float fVelocityErrorAdd[iNbOfArmModules][2];		// For NaN check // to solve NaN/-1.#IND problem with fVelocityErrorSum
	float fVelocityErrorDiff[iNbOfArmModules][2];		// For D control // Can be removed if PI controller used for velocity control
	*/
	float fPositionControlOutput[iNbOfArmModules][2];
	//float fVelocityControlOutput[iNbOfArmModules][2];
	short sCurrentMust[iNbOfArmModules][2];
	short sCurrentIs[iNbOfArmModules][2];
	// Parameter for Position Control
	const static short sDecelerateLimit = 4; // K�nnte abh�ngig von der Zuladung dynamisch angepasst werden
	float fK_p[iNbOfArmModules][2];
	float fK_i[iNbOfArmModules][2];
	float fK_d[iNbOfArmModules][2];
	float fK_u[iNbOfArmModules][2]; // for Ziegler-Nichols-Method
	float fT_u[iNbOfArmModules][2]; // for Ziegler-Nichols-Method
public:
	short sT_s; // Abtastzeit [ms]
	float fT_s; // Abtastzeit [s]

	// Process Monitor // TODO: Remove after testing
private:
	CISELLA3ProcessMonitor ProcessMonitor;
};

extern CISELLA3PositionControlApp theApp;



