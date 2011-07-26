#pragma once

#include "StdAfx.h"

#include "Events.h"

namespace CibraryEngine
{
	class ProgramWindow;

	struct ScreenshotGrabber : public EventHandler
	{
		ProgramWindow* win;

		ScreenshotGrabber(ProgramWindow* win);

		void HandleEvent(Event* evt);
	};
};
