

#include <sdl\SDL.h>

class VisualRobot
{

private:
	int midX, midY;
public:

	SDL_Window * window;
	SDL_Renderer * gRenderer;

	VisualRobot();
	~VisualRobot();

	void setJoints(double j0, double j1);

	bool update();
};

