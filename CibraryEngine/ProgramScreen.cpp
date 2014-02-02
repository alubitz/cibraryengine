#include "StdAfx.h"
#include "ProgramScreen.h"
#include "ProgramWindow.h"

namespace CibraryEngine
{
	ProgramScreen::ProgramScreen(ProgramWindow* window) : window(window), input_state(window->input_state), content(window->content), sound_system(window->sound_system) { }
	ProgramScreen::~ProgramScreen() { }

	void ProgramScreen::Draw(int width, int height)
	{
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}

	ProgramScreen* ProgramScreen::Update(const TimingInfo& time) { return this; }

	void ProgramScreen::Activate() { }
	void ProgramScreen::Deactivate() { }
}
