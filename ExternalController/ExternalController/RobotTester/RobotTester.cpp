// RobotTester.cpp: Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include <windows.h>
#include <tchar.h>
#include <strsafe.h>

#include <stdio.h>
#include <conio.h>

#include "..\CustomRobot\CSERobot.h"

#include "VisualRobot.h"

#include "..\CustomRobot\Conversions.h"



#define SDL

DWORD WINAPI debugMain(LPVOID lpParam);
DWORD WINAPI timerMain(LPVOID lpParam);

//bool close = false;

typedef struct {
	CSERobot *robot;
	VisualRobot *visualizer;
} SimData;

int mainProgram() {

	printf("Get RobotDescription: \n");
	CSERobot robot("urdf/test.urdf");

	printf("\nCreating Window\n");
	VisualRobot win = VisualRobot();

	//printf("Calculating:\n");

	//double DH[] = {25, 0, 0, 0,
	//			   25, 0, 0, 0};

	//CustomRobot a(0, DH, 2, 2.0, 5.0, 10.0);

	SimData simDataInstance;
	simDataInstance.robot = &robot;
	simDataInstance.visualizer = &win;

	
	HANDLE  inputThread = CreateThread(
		NULL,                   // default security attributes
		0,                      // use default stack size  
		debugMain,       // thread function name
		&simDataInstance,          // argument to thread function 
		0,                      // use default creation flags 
		NULL);   // returns the thread identifier 
	
	HANDLE  timerThread = CreateThread(
		NULL,                   // default security attributes
		0,                      // use default stack size  
		timerMain,       // thread function name
		&simDataInstance,          // argument to thread function 
		0,                      // use default creation flags 
		NULL);   // returns the thread identifier 
	
	do {
		

	} while (win.update());

	//close = true;
	//WaitForSingleObject(timerThread, INFINITE);
	CloseHandle(timerThread);
	CloseHandle(inputThread);
	
	return 0;
}

DWORD WINAPI timerMain(LPVOID lpParam) {

	SimData *simData = (SimData *)lpParam;

	HANDLE hTimer = CreateWaitableTimer(NULL, TRUE, NULL);

	LARGE_INTEGER liDueTime;
	liDueTime.QuadPart = -1000000LL;

	double pos[2];
	double vel[2];
	double acc[2];
	int ret;
	int lastRet = 0;

	KDL::Frame kdlFrame;
	double frame[12];
	double rpy[6];

	do{
		SetWaitableTimer(hTimer, &liDueTime, 0, NULL, NULL, 0);
		if ( (ret = simData->robot->nextCycle(pos, vel, acc) & 4) && !(lastRet & 4)) {
			printf("Task finished\n", ret);
			//simData->robot->FK(pos, frame);
			//Marshal::doubleToFrame(kdlFrame, frame);
			//frameToRPY(rpy, &kdlFrame);
			//for (int i = 0; i < 6; i++) {
			//	printf("%f ", rpy[i]);
			//} 
			//printf("\nx: %f\t y: %f\t Result phi2: %f\n", rpy[0], rpy[1], r2d(acos((rpy[0]*rpy[0] + rpy[1]*rpy[1]) / 0.125  - 1)));
		}
		lastRet = ret;
		simData->visualizer->setJoints(r2d(pos[0]), r2d(pos[1]));
	} while (WaitForSingleObject(hTimer, INFINITE) == WAIT_OBJECT_0);
	
	return 0;
}

DWORD WINAPI debugMain(LPVOID lpParam)
{
	
	SimData *simData = (SimData *)lpParam;


	double jointInput[2];
	double frame[12];
	
	char input[256];

	char *nextArgument;

	bool ptp = true;
	Task *task = NULL;

	while (true) {
		
		printf("Enter new Task:\n");
		fgets(input, 256, stdin);
		if (input[0] == 'p' && input[1] == ' ') {
			ptp = true;
		}
		else if (input[0] == 'l' && input[1] == ' ') {
			ptp = false;
		}
		else {
			printf("Could not interpret Task\n");
			continue;
		}

		nextArgument = strchr(input+2, ' ');
		*nextArgument = '\0';
		nextArgument++;

		if (ptp) {
			jointInput[0] = d2r(atof(input + 2));
			jointInput[1] = d2r(atof(nextArgument));

			task = new Task;
			task->type = TaskType::PTP;
			task->target = new Position(jointInput, 2); // memory LEAK !
			
		}
		else {
			jointInput[0] = d2r(atof(input + 2));
			jointInput[1] = d2r(atof(nextArgument));

			task = new Task;
			task->type = TaskType::LIN;
			task->target = new Position(jointInput, 2); // memory LEAK;

		}

		simData->robot->addTask(task);
		printf("New Task added!\n");
	}
	return 0;
}

extern"C" {
	int SDL_main(int argc, char* argv[])
	{
		printf("Hallo SDL\n");
		return mainProgram();
	}
}

int main()
{
	printf("Hallo normal C\n");
	return mainProgram();
}

