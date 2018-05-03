#include "VisualRobot.h"

#include <sdl\SDL.h>
#include <stdio.h>

#include <conio.h>

#include <math.h>

#define SCALE 2

VisualRobot::VisualRobot()
{
	//Screen dimension constants 
	const int SCREEN_WIDTH = 640; 
	const int SCREEN_HEIGHT = 480;

	//The surface contained by the window 
	SDL_Surface* screenSurface = NULL; 
	//Initialize SDL
	if( SDL_Init( SDL_INIT_VIDEO ) < 0 ) { 
		printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
	}
	else { 
		//Create window 
		window = SDL_CreateWindow( "Robot Visualizer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN ); 
		if( window == NULL ) { 
			printf( "Window could not be created! SDL_Error: %s\n", SDL_GetError() ); 
		}
		else { 
			//Get window surface 
			screenSurface = SDL_GetWindowSurface( window ); 
			gRenderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

			midX = SCREEN_WIDTH / 2;
			midY = SCREEN_HEIGHT / 2;

			setJoints(0, 0);


		} 
	}


}


VisualRobot::~VisualRobot()
{
	
	//Destroy window 
	SDL_DestroyRenderer(gRenderer);
	SDL_DestroyWindow(window);
	//Quit SDL subsystems 
	SDL_Quit();
	
}

void VisualRobot::setJoints(double j0, double j1)
{
	//Clear screen 
	SDL_SetRenderDrawColor(gRenderer, 0xFF, 0xFF, 0xFF, 0xFF);
	SDL_RenderClear(gRenderer);

	j0 = j0 * M_PI / 180;
	j1 = j1 * M_PI / 180;

	int j1X = midX + SCALE * 25 * cos(j0);
	int j1Y = midY - SCALE * 25 * sin(j0);

	int j2X = j1X + SCALE * 25 * cos(j0 + j1);
	int j2Y = j1Y - SCALE * 25 * sin(j0 + j1);

	SDL_SetRenderDrawColor(gRenderer, 0x00, 0x00, 0xFF, 0xFF);
	SDL_RenderDrawLine(gRenderer, midX, midY, j1X, j1Y);

	SDL_SetRenderDrawColor(gRenderer, 0x00, 0xFF, 0x00, 0xFF);
	SDL_RenderDrawLine(gRenderer, j1X, j1Y, j2X, j2Y);

	//Update the surface 
	SDL_RenderPresent(gRenderer);
}

bool VisualRobot::update()
{
	bool continueSim = true;
	SDL_Event e;
	while (SDL_PollEvent(&e) != 0) { 
		//User requests quit 
		if( e.type == SDL_QUIT ) { 
			continueSim = false;
		} 
	}
	return continueSim;
}

