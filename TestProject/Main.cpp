#include "StdAfx.h"

#include "MainMenu.h"
#include "LoadingScreen.h"

using namespace std;

using namespace CibraryEngine;
using namespace Test;

int main(int argc, char** argv)
{
	ScriptSystem::Init();

	InitEndianness();

	// Network::DoTestProgram();

	ProgramWindow* win = ProgramWindow::CreateProgramWindow("C++ Game Engine", 0, 0, 0, false);
	if(win == NULL)
		return 1;

	ScreenshotGrabber grabber(win);
	win->input_state->KeyStateChanged += &grabber;

	ProgramScreen* first_screen = new MainMenu(win, NULL);

	int result = win->Run(first_screen);

	win->input_state->KeyStateChanged -= &grabber;

	ScriptSystem::Shutdown();

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
