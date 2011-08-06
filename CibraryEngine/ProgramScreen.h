#pragma once

#include "TimingInfo.h"

namespace CibraryEngine
{
	class ProgramWindow;
	class InputState;
	struct ContentMan;
	class SoundSystem;

	/** Class representing a screen within the program, e.g. the title screen, credits, ingame, etc. */
	class ProgramScreen
	{
		private:

		public:

			/** The ProgramWindow to which this screen belongs */
			ProgramWindow* window;

			InputState* input_state;
			ContentMan* content;
			SoundSystem* sound_system;

			/** Initializes a ProgramScreen belonging to the specified ProgramWindow */
			ProgramScreen(ProgramWindow* window);
			virtual ~ProgramScreen();

			/** Draws this screen, given the dimensions of the client area of the window */
			virtual void Draw(int width, int height);
			/** Allows the screen to update, and returns the next screen to be displayed. A screen should return itself to remain displayed, or NULL to cause the program to exit */
			virtual ProgramScreen* Update(TimingInfo time);

			/** Called when a screen becomes the active screen (i.e. some other screen returns it in ProgramScreen::Update) */
			virtual void Activate();								// do loading here, rather than in the constructor?
			/** Called when a screen is no longer the active screen (i.e. when it returns something other than itself in ProgramScreen::Update) */
			virtual void Deactivate();
	};
}
