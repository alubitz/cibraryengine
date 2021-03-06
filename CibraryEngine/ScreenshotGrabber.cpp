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

			stringstream filename_ss;
			filename_ss << "Screenshots/screenshot-" << now.tm_year + 1900 << "-" << now.tm_mon << "-" << now.tm_mday << "-" << now.tm_hour << "-" << now.tm_min << "-" << now.tm_sec << ".tga";			
			string filename = filename_ss.str();

			int result = SOIL_save_screenshot(filename.c_str(), SOIL_SAVE_TYPE_TGA, 0, 0, win->GetWidth(), win->GetHeight());
			if(!result)
				Debug("Failed to save screenshot");
			else
			{
				stringstream message;
				message << "Saved screenshot as " << filename << endl;
				Debug(message.str());
			}
		}
	}
}
