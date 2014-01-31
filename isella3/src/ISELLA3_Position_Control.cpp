
//#include "stdafx.h"
#include "ISELLA3_Position_Control.h"
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>
#include "ISELLA3_ArmModule.h" // for the object ArmModule and components
#include "ISELLA3_Process_Monitor.h" // for monitoring process data
#include "math.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CISELLA3PositionControlApp construction

CISELLA3PositionControlApp::CISELLA3PositionControlApp()
{
	
}


// The one and only CISELLA3PositionControlApp object

CISELLA3PositionControlApp theApp;


// CISELLA3PositionControlApp INITIALIZATION

bool CISELLA3PositionControlApp::InitInstance(float kp1,float ki1,float kd1,float kp2,float ki2,float kd2,float kp3,float ki3,float kd3,float kp4,float ki4,float kd4)
{
//*************************************************************************************
//* AA code from here...

		e_PosCtrlState = NOT_CONNECTED;
		// for timing
		gettimeofday(&start_time,NULL);
		Time.tv_sec = 0;
		// for error handling
		dwErrorCode = 0;
		hKeyHandle = 0;
		// control variables
		sT_s = 8 * iNbOfArmModules; // SA -> SAMPLING TIME [ms], IT DEPENDS ON THE NUMBER OF ARM MODULES. 8 MS PER MODULE.
		fT_s = sT_s / 1000; // SAMPLING TIME [s]

		// '-> PID-PosCtrl
		
		
		
		//SA -> FIRST ARM MODULE
		fK_p[0][0] = kp1;//0.025;
		fK_i[0][0] = ki1;//0.008;//0.0136; // instable if too big (e.g. 0.136)
		fK_d[0][0] = kd1;//0.0025; // slightly higher value than according to Ziegler-Nichols reduces shoot-over
		fK_p[0][1] = kp2;//0.025;
		fK_i[0][1] = ki2;//0.008;//0.0136; // instable if too big (e.g. 0.136)
		fK_d[0][1] = kd2;//0.0025; // slightly higher value than according to Ziegler-Nichols reduces shoot-over
		//SA -> SECOND ARM MODULE
		fK_p[1][0] = kp3;//0.013;//0.015;
		fK_i[1][0] = ki3;//0.005;//0.118;//0.0136; // instable if too big (e.g. 0.136)
		fK_d[1][0] = kd3;//0.0015;//0.0011; // slightly higher value than according to Ziegler-Nichols reduces shoot-over
		fK_p[1][1] = kp4;//0.013;//0.015;
		fK_i[1][1] = ki4;//0.005;//0.118;//0.0136; // instable if too big (e.g. 0.136)
		fK_d[1][1] = kd4;//0.0015;//0.0011; // slightly higher value than according to Ziegler-Nichols reduces shoot-over
		std::cout << "PID --> K(0,0) =" << endl << fK_p[0][0] << endl << fK_i[0][0] << endl << fK_d[0][0] << endl << "PID --> K(0,1) =" << endl << fK_p[0][1] << endl << fK_i[0][1] << endl << fK_d[0][1] << endl << "PID --> K(1,0) =" << endl << fK_p[1][0] << endl << fK_i[1][0] << endl << fK_d[1][0] << endl << "PID --> K(1,1) =" << endl << fK_p[1][1] << endl << fK_i[1][1] << endl << fK_d[1][1] << endl; 
		// '-> Help variables
		// Init or reset control variables
		// Initialize PositionActualBuffer with current fTimeElapsed, fAngleWheelIs, and zeros
		fTime = (1.e-03 * ((float) this->ulGetTime()));	//[ms]	SA -> GET TIME FROM THE CLOCK

		for (int iArmModuleId = 0; iArmModuleId < iNbOfArmModules; iArmModuleId++)	//SA -> N° OF ARM MODULES = 2
		{
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				fTimestamp[iArmModuleId][iMotorControllerId] = fTime;	//SA -> INDICATORE TEMPO [MS], LO PRENDE DAL CLOCK
				fTimeElapsed[iArmModuleId][iMotorControllerId][0] = fT_s;	//SA -> TEMPO TRASCORSO [S], LO PRENDO DAL TEMPO DI CAMPIONAMENTO IN SECONDI HA 10 RIGHE E QUI INIZIALIZZO LA PRIMA

				// Init Actual Wheel Angles
				ArmModule[iArmModuleId].fAngleAxisIs[0] = 180;
				ArmModule[iArmModuleId].fAngleAxisIs[1] = 180;
				ArmModule[iArmModuleId].fAngleWheelIs[0] = ArmModule[iArmModuleId].fAngleAxisIs[1] + ( ArmModule[iArmModuleId].fAngleAxisIs[0] - 180 ) / double(2.09278351); // 1.685;
				ArmModule[iArmModuleId].fAngleWheelIs[1] = ArmModule[iArmModuleId].fAngleAxisIs[1] - ( ArmModule[iArmModuleId].fAngleAxisIs[0] - 180 ) / double(2.09278351);

				// '-> Init row 1 with fTimeElapsed, fAngleWheelIs, and zeros
				fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][0] = fTimeElapsed[iArmModuleId][iMotorControllerId][0];
				fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1] = ArmModule[iArmModuleId].fAngleWheelIs[iMotorControllerId];
				for (int j = 2; j < 6; j++)
				{
					fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][j] = 0;
				}
				// '-> Copy row 1 to other rows
				for (int i = 0; i < 10; i++)
				{
					for (int j = 0; j < 6; j++)
					{
						fPositionActualBuffer[iArmModuleId][iMotorControllerId][i+1][j] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][i][j];
					}
				}
				// Reset actual control variables
				fPositionActual[iArmModuleId][iMotorControllerId][0] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionActual[iArmModuleId][iMotorControllerId][1] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionTarget[iArmModuleId][iMotorControllerId][0] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionTarget[iArmModuleId][iMotorControllerId][1] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionTarget[iArmModuleId][iMotorControllerId][2] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionError[iArmModuleId][iMotorControllerId][0] = 0;
				fPositionError[iArmModuleId][iMotorControllerId][1] = 0;
				fPositionErrorSum[iArmModuleId][iMotorControllerId] = 0;
				fPositionErrorDiff[iArmModuleId][iMotorControllerId] = 0;
				fPositionControlOutput[iArmModuleId][iMotorControllerId] = 0;
				sCurrentMust[iArmModuleId][iMotorControllerId] = 0;
				sCurrentIs[iArmModuleId][iMotorControllerId] = 0;
			}
		}
	return true;
}


// Getter: Get Time
unsigned long CISELLA3PositionControlApp::ulGetTime()
{
	gettimeofday(&Time,NULL);
	ulTime = 1000000*((unsigned long)(double)(Time.tv_sec)-(unsigned long)(double)(start_time.tv_sec))+((unsigned long)(double)(Time.tv_usec)-(unsigned long)(double)(start_time.tv_usec));
	return ulTime;	//[us]
}

// Getter: Get ErrorCode
unsigned int CISELLA3PositionControlApp::dwGetErrorCode()
{
	return this->dwErrorCode;
}

// Getter: Get KeyHandle
void* CISELLA3PositionControlApp::hGetKeyHandle()
{
	return this->hKeyHandle;
}

// Getter: Get PosCtrlState
enum ModuleState CISELLA3PositionControlApp::e_GetPosCtrlState()
{
	return this->e_PosCtrlState;
}

// Getter: Get ArmModuleState
enum ModuleState CISELLA3PositionControlApp::e_GetArmModuleState(int iArmModuleId)
{
	return ArmModule[iArmModuleId].e_GetModuleState();
}

// Getter: Get MotorControllerModuleState
enum ModuleState CISELLA3PositionControlApp::e_GetMotorControllerModuleState(int iArmModuleId, int iMotorControllerId)
{
	return ArmModule[iArmModuleId].MotorController[iMotorControllerId].e_ModuleState;
}

// Getter: Get Motor Controller State
string CISELLA3PositionControlApp::strGetMotorControllerState(int iArmModuleId, int iMotorControllerId)
{
	try
	{
		switch (ArmModule[iArmModuleId].MotorController[iMotorControllerId].wGetState())
		{
			case 0x0000 : return "Disabled State";
			case 0x0001 : return "Enabled State";
			case 0x0002 : return "Quickstop State";
			case 0x0003 : return "Fault State";
			default : return "Error: Unknown State Machine State!";
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::strGetMotorControllerState..." << endl;
		ErrorHandler(dwErrorCode);
	}
	return "Error: Unknown State Machine State!";
}

// Getter: Update and get Motor Controller State
string CISELLA3PositionControlApp::strUpdateMotorControllerState(int iArmModuleId, int iMotorControllerId)
{
	try
	{
		switch (ArmModule[iArmModuleId].MotorController[iMotorControllerId].wUpdateState(hKeyHandle))
		{
			case 0x0000 : return "Disabled State";
			case 0x0001 : return "Enabled State";
			case 0x0002 : return "Quickstop State";
			case 0x0003 : return "Fault State";
			default : return "Error: Unknown State Machine State!";
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::strGetMotorControllerState..." << endl;
		ErrorHandler(dwErrorCode);
	}
	return "Error: Unknown State Machine State!";
}

// Getter: Get Operation Mode
string CISELLA3PositionControlApp::strGetOpMode(int iArmModuleId, int iMotorControllerId)
{
	try
	{
		switch (ArmModule[iArmModuleId].MotorController[iMotorControllerId].i8GetOpMode())
		{
			case -6 : return "Step Direction Mode";
			case -5 : return "Master Encoder Mode";
			case -3 : return "Current Mode";
			case -2 : return "Velocity Mode";
			case -1 : return "Position Mode";
			case 1 : return "Position Profile Mode";
			case 3 : return "Position Velocity Mode";
			case 6 : return "Homing Mode";
			case 7 : return "Interpolated Position Mode";
			default : return "Error: Unknown Motor Controller Operation Mode!";
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::strGetOpMode..." << endl;
		ErrorHandler(dwErrorCode);
	}
	return "Error: Unknown Motor Controller Operation Mode!";
}

// Getter: Update and get Operation Mode
string CISELLA3PositionControlApp::strUpdateOpMode(int iArmModuleId, int iMotorControllerId)
{
	try
	{
		switch (ArmModule[iArmModuleId].MotorController[iMotorControllerId].i8UpdateOpMode(hKeyHandle))
		{
			case -6 : return "Step Direction Mode";
			case -5 : return "Master Encoder Mode";
			case -3 : return "Current Mode";
			case -2 : return "Velocity Mode";
			case -1 : return "Position Mode";
			case 1 : return "Position Profile Mode";
			case 3 : return "Position Velocity Mode";
			case 6 : return "Homing Mode";
			case 7 : return "Interpolated Position Mode";
			default : return "Error: Unknown Motor Controller Operation Mode!";
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::strGetOpMode..." << endl;
		ErrorHandler(dwErrorCode);
	}
	return "Error: Unknown Motor Controller Operation Mode!";
}

// Getter: Get Brake Status (Open || Close)
string CISELLA3PositionControlApp::strGetBrakeStatus(int iArmModuleId, int iMotorControllerId)
{
	try
	{
		switch (ArmModule[iArmModuleId].MotorController[iMotorControllerId].Brake.bGetStatus())
		{
			case 0 : return "Brake Closed";
			case 1 : return "Brake Open";
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::strGetBrakeStatus..." << endl;
		ErrorHandler(dwErrorCode);
	}
	return "Error: Unknown Brake state!";
}

// Getter: Update and get Brake Status (Open || Close)
string CISELLA3PositionControlApp::strUpdateBrakeStatus(int iArmModuleId, int iMotorControllerId)
{
	try
	{
		switch (ArmModule[iArmModuleId].MotorController[iMotorControllerId].Brake.bUpdateStatus(hKeyHandle))
		{
			case 0 : return "Brake Closed";
			case 1 : return "Brake Open";
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::strGetBrakeStatus..." << endl;
		ErrorHandler(dwErrorCode);
	}
	return "Error: Unknown Brake state!";
}

// Getter: Get Actual Current
short CISELLA3PositionControlApp::sGetCurrentIs(int iArmModuleId, int iMotorControllerId)	//mod. by SA
{
	try
	{
		return ArmModule[iArmModuleId].MotorController[iMotorControllerId].Motor.sGetCurrentIs(hKeyHandle);
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::sGetCurrentIs..." << endl;
		ErrorHandler(dwErrorCode);
	}
	return ArmModule[iArmModuleId].MotorController[iMotorControllerId].Motor.sGetCurrentIs(hKeyHandle);
}

// Getter: Get Actual Angle
float CISELLA3PositionControlApp::fGetAngleAxisIs(int iArmModuleId, int iAxisId)	//mod. by SA
{
	try
	{
		return ArmModule[iArmModuleId].MotorController[iAxisId].AngleSensor.fGetAngleIs(hKeyHandle);
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::fGetAngleAxisIs..." << endl;
		ErrorHandler(dwErrorCode);
	}
	return ArmModule[iArmModuleId].MotorController[iAxisId].AngleSensor.fGetAngleIs(hKeyHandle);
}

// Getter: Get Target Angle
float CISELLA3PositionControlApp::fGetAngleAxisMust(int iArmModuleId, int iAxisId)	//mod. by SA
{
	try
	{
		return ArmModule[iArmModuleId].fGetAngleAxisMust(iAxisId);
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::fGetAngleAxisMust..." << endl;
		ErrorHandler(dwErrorCode);
	}
	return ArmModule[iArmModuleId].fGetAngleAxisMust(iAxisId);
}

// Setter: Set Target Angle
void CISELLA3PositionControlApp::SetAngleMust(int iArmModuleId, int iAxisId, float fValue) // on change
{
	try
	{
		this->ArmModule[iArmModuleId].SetAngleMust(iAxisId, fValue);
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::SetAngleMust..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

// Communication and Initialization: Open Port, get handle and initialize Motor Settings
void CISELLA3PositionControlApp::OpenDevice()
{
	try
	{
		// Open Port
		// Close previous device
		if(this->hKeyHandle) VCS_CloseDevice(hKeyHandle,&dwErrorCode);
		else this->e_PosCtrlState = NOT_CONNECTED;
		// Open Device, get handle 
		if(!(this->hKeyHandle = VCS_OpenDevice("EPOS2","MAXON_RS232","RS232","/dev/ttyS0",&this->dwErrorCode))) throw this->dwErrorCode;
		
		else
		{
			this->e_PosCtrlState = CONNECTED;
			std::cout << "Device opened" << endl;
		}
		// Set Protocol Stack Settings --> by SA
		if(!VCS_SetProtocolStackSettings(hKeyHandle, 1000000, 500, &this->dwErrorCode)) throw this->dwErrorCode;
		else
		{
			std::cout << "SetProtocolStackSettings ok!" << endl;
		}
		// Initialize Motors
		for (int iArmModuleId = 0; iArmModuleId < iNbOfArmModules; iArmModuleId++)
		{
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				ArmModule[iArmModuleId].MotorController[iMotorControllerId].SetNodeId(2 * iArmModuleId + iMotorControllerId + 1);
//				ArmModule[iArmModuleId].MotorController[iMotorControllerId].wGetNodeId();
				std::cout << "NodeId setted for ArmModule " << iArmModuleId << " and MotorController " << iMotorControllerId << endl;
//				ArmModule[iArmModuleId].MotorController[iMotorControllerId].InitState(0x0000);
std::cout << "Entering InitializeSettings..." << endl << "KeyHandle " << hKeyHandle << " ErrorCode " << this->dwErrorCode << endl;
				ArmModule[iArmModuleId].MotorController[iMotorControllerId].InitializeSettings(this->hKeyHandle);
				std::cout << "Settings initialized for ArmModule " << iArmModuleId << " and MotorController " << iMotorControllerId << endl;
			}

			// Set Target Angle = Actual Angle
			this->SetAngleMust(iArmModuleId,0,this->fGetAngleAxisIs(iArmModuleId,0));
			this->SetAngleMust(iArmModuleId,1,this->fGetAngleAxisIs(iArmModuleId,1));
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::OpenDevice..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

// Communication: Close Port
void CISELLA3PositionControlApp::CloseDevice()
{
	try
	{
		if(!VCS_CloseDevice(this->hKeyHandle,&this->dwErrorCode)) throw this->dwErrorCode;
		else
		{
			// Set all ModuleStates to NOT_CONNECTED // TODO: Better use pointers, create new ArmModules on OpenDevice and discard on CloseDevice
			for (int iArmModuleId = 0; iArmModuleId < iNbOfArmModules; iArmModuleId++)
			{
				for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
				{
					this->ArmModule[iArmModuleId].MotorController[iMotorControllerId].e_ModuleState = NOT_CONNECTED;
				}
			}
			// Set PosCtrlState to NOT_CONNECTED
			this->e_PosCtrlState = NOT_CONNECTED;

			// Reset hKeyHandle to 0
			this->hKeyHandle = 0;

// for testing... // TODO: Remove after testing
			// Write ProcessMonitorBuffer to File
			this->ProcessMonitor.WriteBufferToFile();
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::CloseDevice..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

// Enable Motor Controllers // (+ Open brakes) // TODO: Provide Button for Open Brakes instead of opening here, and open on Move, and remove "// (+ Open Brakes)"
void CISELLA3PositionControlApp::Enable()
{
	try
	{
		// Enable Motor Controllers // (+ Open Brakes) // TODO: Remove "// (+ Open Brakes)"
		for (int iArmModuleId = 0; iArmModuleId < this->iNbOfArmModules; iArmModuleId++)
		{
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				ArmModule[iArmModuleId].MotorController[iMotorControllerId].Enable(this->hKeyHandle);
			}
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::Enable..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

// Disable Motor Controllers
void CISELLA3PositionControlApp::Disable()
{
	try
	{
		// Disable Motor Controllers
		for (int iArmModuleId = 0; iArmModuleId < this->iNbOfArmModules; iArmModuleId++)
		{
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				ArmModule[iArmModuleId].MotorController[iMotorControllerId].Motor.Halt(this->hKeyHandle);
				ArmModule[iArmModuleId].MotorController[iMotorControllerId].Disable(this->hKeyHandle);
			}
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::Disable..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

// Start Movement
void CISELLA3PositionControlApp::Move()
{
	try
	{
		for (int iArmModuleId = 0; iArmModuleId < this->iNbOfArmModules; iArmModuleId++)
		{
			this->Move(iArmModuleId);
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::Move..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

void CISELLA3PositionControlApp::Move(int iArmModuleId)
{
	try
	{
		// Move!
		this->ArmModule[iArmModuleId].SetMove();

		// Reset Buffer and Control variables if starting to move
		if ((this->ArmModule[iArmModuleId].e_GetModuleState() == READY) && this->ArmModule[iArmModuleId].bShouldMove)
		{
//			this->ResetControl(iArmModuleId); // TODO: Flag check can probably be omitted

// for testing... // TODO: Remove after testing
			// Start Logging...
			this->ProcessMonitor.ClearBuffer();
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::Move..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

// ResetControl: Init control with actual state as starting point
void CISELLA3PositionControlApp::ResetControl(int iArmModuleId)
{
	{
		try
		{
			// Update Actual Axis Angles
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				// Log time of measurement
				fTime = (1.e-03 * ((float) this->ulGetTime()));
				// Measurement
				ArmModule[iArmModuleId].fAngleAxisIs[iMotorControllerId] = ArmModule[iArmModuleId].MotorController[iMotorControllerId].AngleSensor.fGetAngleIs(hKeyHandle);
				// Calculate time_elapsed since last measurement
				if (fTimestamp[iArmModuleId][iMotorControllerId]) fTimeElapsed[iArmModuleId][iMotorControllerId][0] = fTime - fTimestamp[iArmModuleId][iMotorControllerId];
				else fTimeElapsed[iArmModuleId][iMotorControllerId][0] = fT_s; // for first run after init
				fTimestamp[iArmModuleId][iMotorControllerId] = fTime;
			}
				
			// Calculate and update Actual Wheel Angles
			ArmModule[iArmModuleId].fAngleWheelIs[0] = ArmModule[iArmModuleId].fAngleAxisIs[1] + ( ArmModule[iArmModuleId].fAngleAxisIs[0] - 180 ) / double(2.09278351); // 1.685;
			ArmModule[iArmModuleId].fAngleWheelIs[1] = ArmModule[iArmModuleId].fAngleAxisIs[1] - ( ArmModule[iArmModuleId].fAngleAxisIs[0] - 180 ) / double(2.09278351);

			// Initialize PositionActualBuffer with current fTimeElapsed, fAngleWheelIs, and zeros
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				// '-> Init row 1 with fTimeElapsed, fAngleWheelIs, and zeros
				fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][0] = fTimeElapsed[iArmModuleId][iMotorControllerId][0];
				fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1] = ArmModule[iArmModuleId].fAngleWheelIs[iMotorControllerId];
				for (int j = 2; j < 6; j++)
				{
					fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][j] = 0;
				}
				// '-> Copy row 1 to other rows
				for (int i = 0; i < 10; i++)
				{
					for (int j = 0; j < 6; j++)
					{
						fPositionActualBuffer[iArmModuleId][iMotorControllerId][i+1][j] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][i][j];
					}
				}
				// Reset actual control variables
				fPositionActual[iArmModuleId][iMotorControllerId][0] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionActual[iArmModuleId][iMotorControllerId][1] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionTarget[iArmModuleId][iMotorControllerId][0] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionTarget[iArmModuleId][iMotorControllerId][1] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionTarget[iArmModuleId][iMotorControllerId][2] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];
				fPositionError[iArmModuleId][iMotorControllerId][0] = 0;
				fPositionError[iArmModuleId][iMotorControllerId][1] = 0;
				fPositionErrorSum[iArmModuleId][iMotorControllerId] = 0;
				fPositionErrorDiff[iArmModuleId][iMotorControllerId] = 0;
			}
		}
		catch (unsigned int dwErrorCode)
		{
			std::cout << "Error Handler: CISELLA3PositionControlApp::ResetControl..." << endl;
			ErrorHandler(dwErrorCode);
		}
	}
}

// Move: Call ControlLoop for each ArmModule, and log ProcessData // Currently called by ISELLA3PositionControlDlg::OnTimer
void CISELLA3PositionControlApp::OnTimer(unsigned int nIDEvent)
{
	if (nIDEvent == 1)
	{
		try
		{
			// One ArmModule after another...
			for (int iArmModuleId = 0; iArmModuleId < iNbOfArmModules; iArmModuleId++)
			{
				ProcessMonitor.WriteProcessDataToBuffer("ArmModuleId", &iArmModuleId);
				ProcessMonitor.WriteProcessDataToBuffer("Time", this->ulGetTime());
				ProcessMonitor.WriteProcessDataToBuffer("P0", &fK_p[iArmModuleId][0]);
				ProcessMonitor.WriteProcessDataToBuffer("P1", &fK_p[iArmModuleId][1]);
				ProcessMonitor.WriteProcessDataToBuffer("I0", &fK_i[iArmModuleId][0]);
				ProcessMonitor.WriteProcessDataToBuffer("I1", &fK_i[iArmModuleId][1]);
				ProcessMonitor.WriteProcessDataToBuffer("D0", &fK_d[iArmModuleId][0]);
				ProcessMonitor.WriteProcessDataToBuffer("D1", &fK_d[iArmModuleId][1]);
				ProcessMonitor.WriteProcessDataToBuffer("time_before_control_loop", ProcessMonitor.ulStart());

				// Call ControlLoop
				this->ControlLoop(iArmModuleId);

				// Log process data
				// '-> Time (T1 and T_diff)
				ProcessMonitor.WriteProcessDataToBuffer("time_after_control_loop", ProcessMonitor.ulStop());
				ProcessMonitor.WriteProcessDataToBuffer("T_diff", ProcessMonitor.ulDiff());
				// '-> Angles (Must and Is)
				ProcessMonitor.WriteProcessDataToBuffer("AngleAxisMust1", &ArmModule[iArmModuleId].fAngleAxisMust[1]);
				ProcessMonitor.WriteProcessDataToBuffer("AngleAxisMust0", &ArmModule[iArmModuleId].fAngleAxisMust[0]);
				ProcessMonitor.WriteProcessDataToBuffer("AngleWheelMust1", &ArmModule[iArmModuleId].fAngleWheelMust[0]);
				ProcessMonitor.WriteProcessDataToBuffer("AngleWheelMust0", &ArmModule[iArmModuleId].fAngleWheelMust[1]);
				ProcessMonitor.WriteProcessDataToBuffer("AngleAxisIs1", &ArmModule[iArmModuleId].fAngleAxisIs[1]);
				ProcessMonitor.WriteProcessDataToBuffer("AngleAxisIs0", &ArmModule[iArmModuleId].fAngleAxisIs[0]);
				ProcessMonitor.WriteProcessDataToBuffer("AngleWheelIs1", &ArmModule[iArmModuleId].fAngleWheelIs[1]);
				ProcessMonitor.WriteProcessDataToBuffer("AngleWheelIs0", &ArmModule[iArmModuleId].fAngleWheelIs[0]);
				ProcessMonitor.WriteProcessDataToBuffer("PositionError1", &fPositionError[iArmModuleId][1][0]);
				ProcessMonitor.WriteProcessDataToBuffer("PositionError0", &fPositionError[iArmModuleId][0][0]);
				
				// '-> Currents (Must and Is)
				sCurrentMust[iArmModuleId][0] = ArmModule[iArmModuleId].MotorController[0].Motor.sGetCurrentMust(hKeyHandle);
				ProcessMonitor.WriteProcessDataToBuffer("CurrentMust0", &sCurrentMust[iArmModuleId][0]);
				sCurrentIs[iArmModuleId][0] = sGetCurrentIs(iArmModuleId,0);
				ProcessMonitor.WriteProcessDataToBuffer("CurrentIs0", &sCurrentIs[iArmModuleId][0]);
				sCurrentMust[iArmModuleId][1] = ArmModule[iArmModuleId].MotorController[1].Motor.sGetCurrentMust(hKeyHandle);
				ProcessMonitor.WriteProcessDataToBuffer("CurrentMust1", &sCurrentMust[iArmModuleId][1]);
				sCurrentIs[iArmModuleId][1] = sGetCurrentIs(iArmModuleId,1);
				ProcessMonitor.WriteProcessDataToBuffer("CurrentIs1", &sCurrentIs[iArmModuleId][1]);
				ProcessMonitor.WriteBufferEndl();
			}
		}
		catch (unsigned int dwErrorCode)
		{
			std::cout << "Error Handler: CISELLA3PositionControlApp::OnTimer..." << endl;
			ErrorHandler(dwErrorCode);
		}
	}
}

// Enter MANUAL mode
void CISELLA3PositionControlApp::EnterManualMode() // TODO: Still needs adjusting: Currently ManualMode is entered twice
{
	try
	{
		std::cout << "Manual Mode:\n\n Following, Manual Mode will be executed for one Arm Module after another, starting with Arm Module 0." << endl;
		for (int iArmModuleId = 0; iArmModuleId < this->iNbOfArmModules; iArmModuleId++)
		{
			this->EnterManualMode(iArmModuleId);
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::EnterManualMode..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

void CISELLA3PositionControlApp::EnterManualMode(int iArmModuleId)
{
	try
	{
		std::cout << "Manual Mode:\n\n Secure Arm Module. Brakes will be opened on OK." << endl;
		for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
		{
			this->ArmModule[iArmModuleId].MotorController[iMotorControllerId].Brake.Open(this->hKeyHandle);
		}
		std::cout << "Manual Mode:\n\n Brakes will be closed on OK." << endl;
		for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
		{
			this->ArmModule[iArmModuleId].MotorController[iMotorControllerId].Brake.Close(this->hKeyHandle);
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::EnterManualMode..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

//Open Brakes --> created by SA
void CISELLA3PositionControlApp::OpenBrakes()
{
	try
	{
		std::cout << "Manual Mode:\n\n Secure Arm Module. Brakes will be opened on OK." << endl;
		for (int iArmModuleId = 0; iArmModuleId < this->iNbOfArmModules; iArmModuleId++)
		{
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				this->ArmModule[iArmModuleId].MotorController[iMotorControllerId].Brake.Open(this->hKeyHandle);
			}
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::OpenBrakes..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

//Close Brakes --> created by SA
void CISELLA3PositionControlApp::CloseBrakes()
{
	try
	{
		std::cout << "Manual Mode:\n\n Secure Arm Module. Brakes will be opened on OK." << endl;
		for (int iArmModuleId = 0; iArmModuleId < this->iNbOfArmModules; iArmModuleId++)
		{
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				this->ArmModule[iArmModuleId].MotorController[iMotorControllerId].Brake.Close(this->hKeyHandle);
			}
		}
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::CloseBrakes..." << endl;
		ErrorHandler(dwErrorCode);
	}
}

// ControlLoop: Get Actual State, get Target State, Exec changes
void CISELLA3PositionControlApp::ControlLoop(int iArmModuleId)
{
	{
		try
		{
			// Update Actual Axis Angles
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				// Log time of measurement
				fTime = (1.e-03 * ((float) this->ulGetTime()));
				// Measurement
				ArmModule[iArmModuleId].fAngleAxisIs[iMotorControllerId] = ArmModule[iArmModuleId].MotorController[iMotorControllerId].AngleSensor.fGetAngleIs(hKeyHandle);
				// Calculate time_elapsed since last measurement
				if (fTimestamp[iArmModuleId][iMotorControllerId]) fTimeElapsed[iArmModuleId][iMotorControllerId][0] = fTime - fTimestamp[iArmModuleId][iMotorControllerId];	//SA -> DELTA_T IN [S]
				else fTimeElapsed[iArmModuleId][iMotorControllerId][0] = fT_s; // for first run after init SA -> SAMPLING TIME IN [S]
				fTimestamp[iArmModuleId][iMotorControllerId] = fTime;
			}
				
			// Calculate and update Actual Wheel Angles (Assumption: No delay between wheel and joint angle)
			ArmModule[iArmModuleId].fAngleWheelIs[0] = ArmModule[iArmModuleId].fAngleAxisIs[1] + ( ArmModule[iArmModuleId].fAngleAxisIs[0] - 180 ) / double(2.09278351); // 1.685;
			ArmModule[iArmModuleId].fAngleWheelIs[1] = ArmModule[iArmModuleId].fAngleAxisIs[1] - ( ArmModule[iArmModuleId].fAngleAxisIs[0] - 180 ) / double(2.09278351);

			// Calculate Difference between Target and Actual
			for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
			{
				// Update fPositionActualBuffer
					// Shift fPositionActualBuffer matrix down by 1 row
					for (int i = 10; i > 0; i--)
					{
						for (int j = 0; j < 6; j++)
						{
							fPositionActualBuffer[iArmModuleId][iMotorControllerId][i][j] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][i-1][j];
						}
					}
					// Copy timestamp to buffer
					fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][0] = fTime;
					// Pick actual values from ArmModule and copy to buffer (for filtering) 
					fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1] = ArmModule[iArmModuleId].fAngleWheelIs[iMotorControllerId];

				//// Filter (PT1)
//				fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][2] = (1 - fTimeElapsed[iArmModuleId][iMotorControllerId][0] / 0.5) * fPositionActualBuffer[iArmModuleId][iMotorControllerId][1][2] + (fTimeElapsed[iArmModuleId][iMotorControllerId][0] / 0.5) * fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];

				//// Position Control

				// Get actual position value
				fPositionActual[iArmModuleId][iMotorControllerId][1] = fPositionActual[iArmModuleId][iMotorControllerId][0];
				fPositionActual[iArmModuleId][iMotorControllerId][0] = fPositionActualBuffer[iArmModuleId][iMotorControllerId][0][1];

				// Get target position value
				fPositionTarget[iArmModuleId][iMotorControllerId][2] = fPositionTarget[iArmModuleId][iMotorControllerId][1];
				fPositionTarget[iArmModuleId][iMotorControllerId][1] = fPositionTarget[iArmModuleId][iMotorControllerId][0];
				
				// Apply filter
				// '-> PT1 (currently with strong overshoot // TODO: Adjust)
//				fPositionTarget[iArmModuleId][iMotorControllerId][0] = (ArmModule[iArmModuleId].fAngleWheelMust[iMotorControllerId] * fTimeElapsed[iArmModuleId][iMotorControllerId][0] + 0.3 * fPositionTarget[iArmModuleId][iMotorControllerId][1]) / (0.3 + fTimeElapsed[iArmModuleId][iMotorControllerId][0]) ;
				fPositionTarget[iArmModuleId][iMotorControllerId][0] = ArmModule[iArmModuleId].fAngleWheelMust[iMotorControllerId];
				
				// Calculate Position Error
				fPositionError[iArmModuleId][iMotorControllerId][1] = fPositionError[iArmModuleId][iMotorControllerId][0];
				fPositionError[iArmModuleId][iMotorControllerId][0] = fPositionTarget[iArmModuleId][iMotorControllerId][0] - fPositionActual[iArmModuleId][iMotorControllerId][0];
				std::cout << "Position Error = " << fPositionError[iArmModuleId][iMotorControllerId][0] << endl;
				
				// Calculate Position Error Sum
				fPositionErrorSum[iArmModuleId][iMotorControllerId] += (fPositionError[iArmModuleId][iMotorControllerId][0] + fPositionError[iArmModuleId][iMotorControllerId][1]) * 0.5 * fTimeElapsed[iArmModuleId][iMotorControllerId][0];
				
				// '-> Limit
				if (abs(fPositionErrorSum[iArmModuleId][iMotorControllerId]) > 1000) fPositionErrorSum[iArmModuleId][iMotorControllerId] *= 1000 / fPositionErrorSum[iArmModuleId][iMotorControllerId];

				// Calculate Differential of Position Error
				fPositionErrorDiff[iArmModuleId][iMotorControllerId] = (fPositionError[iArmModuleId][iMotorControllerId][0] - fPositionError[iArmModuleId][iMotorControllerId][1]) / fTimeElapsed[iArmModuleId][iMotorControllerId][0];
				
				// '-> PID-PosCtrl (controller output: CurrentMust)
				fPositionControlOutput[iArmModuleId][iMotorControllerId] = ( fK_p[iArmModuleId][iMotorControllerId] * fPositionError[iArmModuleId][iMotorControllerId][0] + fK_i[iArmModuleId][iMotorControllerId] * fPositionErrorSum[iArmModuleId][iMotorControllerId] + fK_d[iArmModuleId][iMotorControllerId] * fPositionErrorDiff[iArmModuleId][iMotorControllerId] );
				if (fPositionControlOutput[iArmModuleId][iMotorControllerId] > 3890) fPositionControlOutput[iArmModuleId][iMotorControllerId] = 3890;
				else if (fPositionControlOutput[iArmModuleId][iMotorControllerId] < -3890) fPositionControlOutput[iArmModuleId][iMotorControllerId] = -3890;


/*				fPositionControlOutput[iArmModuleId][iMotorControllerId] = 2660 * ( fK_p[iArmModuleId][iMotorControllerId] * fPositionError[iArmModuleId][iMotorControllerId][0] + fK_i[iArmModuleId][iMotorControllerId] * fPositionErrorSum[iArmModuleId][iMotorControllerId] + fK_d[iArmModuleId][iMotorControllerId] * fPositionErrorDiff[iArmModuleId][iMotorControllerId] );
				if (fPositionControlOutput[iArmModuleId][iMotorControllerId] > 2660) fPositionControlOutput[iArmModuleId][iMotorControllerId] = 2660;
				else if (fPositionControlOutput[iArmModuleId][iMotorControllerId] < -2660) fPositionControlOutput[iArmModuleId][iMotorControllerId] = -2660;
*/
				// Set sCurrentMust as Input for MotorController Current Control
				sCurrentMust[iArmModuleId][iMotorControllerId] = fPositionControlOutput[iArmModuleId][iMotorControllerId];
				
				// '-> As both motors turn in the same direction, and therefore effect opposing movement, MotorController.Motor[0] has to be inverted
				if (iMotorControllerId) sCurrentMust[iArmModuleId][iMotorControllerId] *= -1;
				std::cout << "sCurrentMust = " << sCurrentMust[iArmModuleId][iMotorControllerId] << endl;
//				this->ArmModule[iArmModuleId].SetMove();
			}

			if (this->ArmModule[iArmModuleId].bShouldMove)
			{
				for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
				{
					// Execute Movement
					ArmModule[iArmModuleId].MotorController[iMotorControllerId].Motor.Move(hKeyHandle, sCurrentMust[iArmModuleId][iMotorControllerId]);
				}
			}
		}
		catch (unsigned int dwErrorCode)
		{
			std::cout << "Error Handler: CISELLA3PositionControlApp::ControlLoop..." << endl;
			ErrorHandler(dwErrorCode);
		}
	}
}

// Halt Motors
void CISELLA3PositionControlApp::Halt()
{
	try
	{
		// Halt = Halt all ArmModules => Set all Move flags to 0 and halt all motors by calling SetHalt for every ArmModule
		for (int iArmModuleId = 0; iArmModuleId < 2; iArmModuleId++)
		{
			this->Halt(iArmModuleId);
		}
		
		// TODO: Close process data file
	}
	catch (unsigned int dwErrorCode)
	{
		std::cout << "Error Handler: CISELLA3PositionControlApp::Halt..." << endl;
		std::cout << "CISELLA3PositionControlApp::Halt\n\nCould not Halt! MotorController not in Enabled State." << endl;
		ErrorHandler(dwErrorCode);
	}
}

void CISELLA3PositionControlApp::Halt(int iArmModuleId)
{
	try
	{
		// Call SetHalt for ArmModule to set Move flag to 0 and halt motors
		this->ArmModule[iArmModuleId].SetHalt(hKeyHandle);
	}
	catch (unsigned int dwErrorCode)
	{
		ErrorHandler(dwErrorCode);
	}
}


void CISELLA3PositionControlApp::ErrorHandler(unsigned int dwErrorCode)
{
	try
	{
		if (hKeyHandle)
		{
			// Halt and disable all
			for (int iArmModuleId = 0; iArmModuleId < iNbOfArmModules; iArmModuleId++)
			{
				for (int iMotorControllerId = 0; iMotorControllerId < 2; iMotorControllerId++)
				{
					if (ArmModule[iArmModuleId].MotorController[iMotorControllerId].wGetState() == 0x0001)
					{
						// Halt
						ArmModule[iArmModuleId].MotorController[iMotorControllerId].Motor.Halt(this->hKeyHandle);
						// Disable
						ArmModule[iArmModuleId].MotorController[iMotorControllerId].Disable(this->hKeyHandle);
					}
					else
					{
						std::cout << "CISELLA3PositionControlApp::Halt\n\nCould not Halt! MotorController not in Enabled State." << endl;
					}
				}
			}
		}
	}
	catch (unsigned int dwErrorCode)
	{
		// Error Message
		std::cout << "ErrorHandler: Could not halt Motors and disable Motor Controllers. Please press Emergency Stop!" << endl;
	}

	// Set ErrorFlag
	this->dwErrorCode = dwErrorCode;
	// Error Message
	switch (dwErrorCode)
	{
		// Communication Errors
		case 0x05030000 : std::cout << "Toggle Error: Toggle bit not alternated!" << endl; break;
		case 0x05040000 : std::cout << "SDO Time Out: SDO protocol timed out!" << endl; break;
		case 0x05040001 : std::cout << "Client/server specifier Error: Client/server command specifier not valid or unknown!" << endl; break;
		case 0x05040002 : std::cout << "Invalid block size: Invalid block size (block mode only)!" << endl; break;
		case 0x05040003 : std::cout << "Invalid sequence: Invalid sequence number (block mode only)!" << endl; break;
		case 0x05040004 : std::cout << "CrcError: CRC error (block mode only)!" << endl; break;
		case 0x05040005 : std::cout << "Out of Memory Error: Out of Memory!" << endl; break;
		case 0x06010000 : std::cout << "Access Error: Unsupported access to an object (e.g. write command to a read-only object)!" << endl; break;
		case 0x06010001 : std::cout << "Write Only: Read command to a write only object!" << endl; break;
		case 0x06010002 : std::cout << "Read Only: Write command to a read only object!" << endl; break;
		case 0x06020000 : std::cout << "Object does not exist Error: The last read or write command had a wrong object index or sub-index!" << endl; break;
		case 0x06040041 : std::cout << "PDO mapping Error: The object cannot be mapped to the PDO!" << endl; break;
		case 0x06040042 : std::cout << "PDO length Error: The number and length of the objects to be mapped would exceed PDO length!" << endl; break;
		case 0x06040043 : std::cout << "General parameter Error: General parameter incompatibility!" << endl; break;
		case 0x06040047 : std::cout << "General Intern Incompatibility Error: General internal incompatibility in device!" << endl; break;
		case 0x06060000 : std::cout << "Hardware Error: Access failed due to an hardware error!" << endl; break;
		case 0x06070010 : std::cout << "Service Parameter Error: Data type does not match, length or service parameter does not match!" << endl; break;
		case 0x06070012 : std::cout << "Service Parameter Error: too High Error Data type does not match, length or service parameter too high!" << endl; break;
		case 0x06070013 : std::cout << "Service Parameter Error: too Low Error Data type does not match, length or service parameter too low!" << endl; break;
		case 0x06090011 : std::cout << "Object Sub-index Error: The last read or write command had a wrong Object sub-index!" << endl; break;
		case 0x06090030 : std::cout << "Value Range Error: Value range of parameter exceeded!" << endl; break;
		case 0x06090031 : std::cout << "Value too High Error: Value of parameter written too high!" << endl; break;
		case 0x06090032 : std::cout << "Value too Low Error: Value of parameter written too low!" << endl; break;
		case 0x06090036 : std::cout << "Maximum less Minimum Error: Maximum value is less than minimum value!" << endl; break;
		case 0x08000000 : std::cout << "General Error: General error!" << endl; break;
		case 0x08000020 : std::cout << "Transfer or store Error: Data cannot be transferred or stored!" << endl; break;
		case 0x08000021 : std::cout << "Local control Error: Data cannot be transferred or stored to application because of local control!" << endl; break;
		case 0x08000022 : std::cout << "Wrong Device State: Data cannot be transferred or stored to application because of the present device state!" << endl; break;
		case 0x0F00FFB9 : std::cout << "Error CAN id: Wrong CAN id!" << endl; break;
		case 0x0F00FFBC : std::cout << "Error Service Mode: The device is not in service mode!" << endl; break;
		case 0x0F00FFBE : std::cout << "Password Error: The password is wrong!" << endl; break;
		case 0x0F00FFBF : std::cout << "Illegal Command Error: The RS232 command is illegal (does not exist)!" << endl; break;
		case 0x0F00FFC0 : std::cout << "Wrong NMT State Error: The device is in wrong NMT state!" << endl; break;

		// General Errors
		case 0x10000001 : std::cout << "Internal Error: Internal Error!" << endl; break;
		case 0x10000002 : std::cout << "Null Pointer: Null Pointer passed to function!" << endl; break;
		case 0x10000003 : std::cout << "Handle not Valid: Handle passed to function is not valid!" << endl; break;
		case 0x10000004 : std::cout << "Bad Virtual Device Name: Virtual Device name is not valid!" << endl; break;
		case 0x10000005 : std::cout << "Bad Device Name: Device name is not valid!" << endl; break;
		case 0x10000006 : std::cout << "Bad ProtocolStack Name: ProtocolStack name is not valid!" << endl; break;
		case 0x10000007 : std::cout << "Bad Interface Name: Interface name is not valid!" << endl; break;
		case 0x10000008 : std::cout << "Bad Port Name: Port is not valid!" << endl; break;
		case 0x10000009 : std::cout << "Library not Loaded: Could not load external library!" << endl; break;
		case 0x1000000A : std::cout << "Executing Command: Command failed!" << endl; break;
		case 0x1000000B : std::cout << "Timeout: Timeout occurred during execution!" << endl; break;
		case 0x1000000C : std::cout << "Bad Parameter: Bad Parameter passed to function!" << endl; break;
		case 0x1000000D : std::cout << "Command Aborted By User: Command aborted by user!" << endl; break;

		// Interface Layer Errors
		case 0x20000001 : std::cout << "Opening Interface: Error opening interface!" << endl; break;
		case 0x20000002 : std::cout << "Closing Interface: Error closing interface!" << endl; break;
		case 0x20000003 : std::cout << "Interface not Open: Interface is not open!" << endl; break;
		case 0x20000004 : std::cout << "Opening Port: Error opening port!" << endl; break;
		case 0x20000005 : std::cout << "Closing Port: Error closing port!" << endl; break;
		case 0x20000006 : std::cout << "Port not Open: Port is not open!" << endl; break;
		case 0x20000007 : std::cout << "Reset Port: Error resetting port!" << endl; break;
		case 0x20000008 : std::cout << "Set Port Settings: Error configuring port settings!" << endl; break;
		case 0x20000009 : std::cout << "Set Port Mode: Error configuring port mode!" << endl; break;

		// Interface Layer 'RS232' Errors
		case 0x21000001 : std::cout << "Write Data: Error writing data!" << endl; break;
		case 0x21000002 : std::cout << "Read Data: Error reading data!" << endl; break;

		// Interface Layer 'CAN' Errors
		case 0x22000001 : std::cout << "Receive CAN Frame: Error receiving CAN frame!" << endl; break;
		case 0x22000002 : std::cout << "Transmit CAN Frame: Error transmitting CAN frame!" << endl; break;

		// Interface Layer 'USB' Errors
		case 0x23000001 : std::cout << "Write Data: Error writing data!" << endl; break;
		case 0x23000002 : std::cout << "Read Data: Error reading data!" << endl; break;

		// Protocol Layer 'MaxonRS232' Errors
		case 0x31000001 : std::cout << "NegAckReceived: Negative acknowledge received!" << endl; break;
		case 0x31000002 : std::cout << "BadCrcReceived: Bad checksum received!" << endl; break;
		case 0x31000003 : std::cout << "BadDataSizeReceived: Bad data size received!" << endl; break;

		// Protocol Layer 'CANopen' Errors
		case 0x32000001 : std::cout << "SdoReceiveFrameNotReceived: CAN frame of SDO protocol not received!" << endl; break;
		case 0x32000002 : std::cout << "RequestedCanFrameNotReceived: Requested CAN frame not received!" << endl; break;
		case 0x32000003 : std::cout << "CanFrameNotReceived: Can frame not received!" << endl; break;

		// Protocol Layer 'USB' Errors
		case 0x33000001 : std::cout << "Stuffing: Failed Stuffing Data!" << endl; break;
		case 0x33000002 : std::cout << "Destuffing: Failed Destuffing Data!" << endl; break;
		case 0x33000003 : std::cout << "BadCrcReceived: Bad CRC received!" << endl; break;
		case 0x33000004 : std::cout << "BadDataSizeReceived: Bad Data received!" << endl; break;

		// Specific I3PosCtrl Errors
		case 0xA0000000 : std::cout << "Error 0xA0000000: Unknown Error!" << endl; break;
		case 0xA0000001 : std::cout << "Error 0xA0000001: LimitSwitch badly defined!" << endl; break;
		case 0xA0000002 : std::cout << "Error 0xA0000002: Negative Limit Switch active!" << endl; break;
		case 0xA0000003 : std::cout << "Error 0xA0000003: Positive Limit Switch active!" << endl; break;
		case 0xA0000004 : std::cout << "Error 0xA0000004: Unknown Error checking Brake Status!" << endl; break;
		case 0xA0000005 : std::cout << "Error 0xA0000005: Invalid AxisId requested!" << endl; break;
		case 0xA0000006 : std::cout << "Error 0xA0000006: Angle limit violation!" << endl; break;
		default : std::cout << "Unknown Error!" << endl; break;
	}
}
