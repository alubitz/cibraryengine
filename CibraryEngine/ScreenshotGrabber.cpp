#include "StdAfx.h"
#include "ScreenshotGrabber.h"

#include "InputState.h"
#include "ProgramWindow.h"

#include "DebugLog.h"

#include "../SOIL/SOIL.h"

namespace CibraryEngine
{
	/*
	 * ScreenshotGrabber methods
	 */
	ScreenshotGrabber::ScreenshotGrabber(ProgramWindow* win) : win(win) { }

	void ScreenshotGrabber::HandleEvent(Event* evt)
	{
		KeyStateEvent* kse = (KeyStateEvent*)evt;
		if(kse->key == VK_F12 && kse->state)
		{
			time_t raw_time;
			time(&raw_time);
			tm now = *localtime(&raw_time);

			string filename = ((stringstream&)(stringstream() << "Screenshots/screenshot-" << now.tm_year + 1900 << "-" << now.tm_mon + 1 << "-" << now.tm_mday << "-" << now.tm_hour << "-" << now.tm_min << "-" << now.tm_sec << ".tga")).str();

			int result = SOIL_save_screenshot(filename.c_str(), SOIL_SAVE_TYPE_TGA, 0, 0, win->GetWidth(), win->GetHeight());
			if(!result)
				Debug("Failed to save screenshot\n");
			else
				Debug(((stringstream&)(stringstream() << "Saved screenshot as " << filename << endl)).str());
		}
	}
}
