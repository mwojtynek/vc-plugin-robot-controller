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
		}
		lastRet = ret;
		simData->visualizer->setJoints(r2d(pos[0]), r2d(pos[1]));
	} while (WaitForSingleObject(hTimer, INFINITE) == WAIT_OBJECT_0);
	
	return 0;
}

DWORD WINAPI debugMain(LPVOID lpParam)
{
	
	SimData *simData = (SimData *)lpParam;

	char input[256];

	char *nextArgument;

	bool ptp = true;
	int useFrame = 0;
	Task *task = NULL;

	while (true) {
		
		printf("Enter new Task:\n");
		fgets(input, 256, stdin);

		if (strlen(input) < 4) {
			printf("Could not interpret command\n");
			continue;
		}

		if (input[0] == 'p') {
			ptp = true;
			printf("PTP ");
		}
		else if (input[0] == 'l') {
			ptp = false;
			printf("LIN ");
		}
		else {
			printf("Could not interpret command\n");
			continue;
		}

		if (input[1] == ' ') {
			useFrame = 0;
			printf("JOINTS\n");
		}
		else if( input[1] == 'f' && (input[2] == ' ' || (input[2] == '+' && input[3] == ' ') ) ) {
			useFrame = 1;
			printf("POSITIV FRAME\n");
		}
		else if (input[1] == 'f' && input[2] == '-' && input[3] == ' ') {
			useFrame = 2;
			printf("NEGATIV FRAME\n");
		}
		else {
			printf("Could not interpret command\n");
		}

		std::vector<char *> arguments;
		nextArgument = strchr(input, ' ');
		while (nextArgument != NULL) {
			*nextArgument = '\0';
			nextArgument++;
			arguments.push_back(nextArgument);
			nextArgument = strchr(nextArgument, ' ');
		}

		printf("Found %d arguments\n", arguments.capacity());

		if (arguments.capacity() != 2) {
			printf("Wrong number of arguments\n");
			continue;
		}

		task = new Task; // first delete target!
		if (ptp) {
			task->type = TaskType::PTP;
		}
		else {
			task->type = TaskType::LIN;
		}
		
		if (useFrame == 0) {
			KDL::JntArray arr(2);
			for (int i = 0; i < 2; i++) {
				arr(i) = d2r(atof(arguments[i]));
			}

			task->target = new Position(arr); 
			
		}
		else { // ADAPT TODO; THIS IS JUST FOR THE ROBOTTESTER!
			// input
			double r = atof(arguments[0]);
			double phi = d2r(atof(arguments[1]));

			double x = cos(phi) * r;
			double y = sin(phi) * r;

			double xs = x * 4; // meaning / 0.25 the length of arm
			double ys = y * 4;
			double c = (xs * xs + ys * ys) / 2;
			double cc = c * c;

			double p = - xs * ys / (ys * ys - cc);
			double q = (xs * xs - cc) / (ys * ys - cc);
			double radix = 0;
			if(p*p > q) radix = sqrt(p * p - q);

			double yaw = 0.0;
			if (isinf(radix)) {
				if (y<0) {
					yaw = -M_PI / 2;
				}
				else {
					yaw = M_PI / 2;
				}
			}
			else {
				if (useFrame == 1) {
					yaw = atan(p + radix);
				}
				else {
					yaw = atan(p - radix);
				}
			}

			if (x < 0) yaw += M_PI;

			double rpy[6];
			KDL::Frame newFrame(KDL::Rotation::EulerZYX(yaw, 0.0, 0.0), KDL::Vector(x, y, 0.0));
			frameToRPY(rpy, &newFrame);
			for (int i = 0; i < 6; i++) {
				if (i > 2) {
					printf("%f ", r2d(rpy[i]));
				}
				else {
					printf("%f ", rpy[i]);
				}
			}
			printf("\n");
			task->target = new Position(newFrame);
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

