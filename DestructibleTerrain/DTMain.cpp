#include "StdAfx.h"

#include "DTScreen.h"

using namespace std;

using namespace CibraryEngine;
using namespace DestructibleTerrain;

int main(int argc, char** argv)
{
	InitEndianness();

	ProgramWindow* win = ProgramWindow::CreateProgramWindow("Destructible Terrain Test Program", 0, 0, 0, false);
	if(win == NULL)
		return 1;

	ScreenshotGrabber grabber(win);
	win->input_state->KeyStateChanged += &grabber;

	ProgramScreen* first_screen = new DTScreen(win);

	int result = win->Run(first_screen);

	win->input_state->KeyStateChanged -= &grabber;

	delete first_screen;
	delete win;

	return result;
}

#ifdef WIN32
int WINAPI WinMain(	HINSTANCE	hInstance,		// Instance
					HINSTANCE	hPrevInstance,	// Previous Instance
					LPSTR		lpCmdLine,		// Command Line Parameters
					int			nCmdShow)		// Window Show State
{
	return main(nCmdShow, &lpCmdLine);
}
#endif
