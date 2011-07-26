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
		if(kse->key == VK_F2 && kse->state)
		{
			time_t raw_time;
			time(&raw_time);
			tm* now = localtime(&raw_time);

			char filename[100];
			sprintf(filename, "Screenshots/screenshot-%i-%i-%i-%i-%i-%i.tga", now->tm_year + 1900, now->tm_mon, now->tm_mday, now->tm_hour, now->tm_min, now->tm_sec);

			int result = SOIL_save_screenshot(filename, SOIL_SAVE_TYPE_TGA, 0, 0, win->GetWidth(), win->GetHeight());
			if(!result)
				Debug("Failed to save screenshot");
			else
			{
				char message[200];
				sprintf(message, "Saved screenshot as %s\n", filename);
				Debug(message);
			}
		}
	}
}
