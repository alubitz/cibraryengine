#pragma once

#include "StdAfx.h"
#include "InputState.h"
#include "CmdArgs.h"
#include "Content.h"

#include "SoundSystem.h"
#include "Events.h"

namespace CibraryEngine
{
	class ProgramScreen;
	class ScreenshotGrabber;

	/** Class representing the runtime, and the main window displayed */
	class ProgramWindow
	{
	public:

		struct Imp;
		Imp* imp;

		ContentMan* content;
		InputState* input_state;
		SoundSystem* sound_system;

		/** The total amount of time that the program has been running */

		ProgramWindow(Imp* imp);
		~ProgramWindow();

		/** Runs the game, starting by displaying the specified ProgramScreen */
		int Run(ProgramScreen* initial_screen);

		/** Sets the caption of the window */
		void SetTitle(const string& text);

		int GetWidth();
		int GetHeight();

		bool IsFinished();
		bool IsActive();

		void SetActive(bool active);
		void SetFinished();

		bool IsFullscreen();

		EventDispatcher OnWindowClosing;

		static ProgramWindow* CreateProgramWindow(const string& title, int w, int h, int bpp, bool fullscreen);
	};

	struct WindowClosingEvent : public Event
	{
		ProgramWindow* window;
		bool cancel;

		WindowClosingEvent(ProgramWindow* window) : window(window), cancel(false) { }
	};
}
