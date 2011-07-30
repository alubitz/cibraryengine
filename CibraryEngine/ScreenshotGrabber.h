#pragma once

#include "StdAfx.h"

#include "Events.h"

namespace CibraryEngine
{
	class ProgramWindow;

	class ScreenshotGrabber : public EventHandler
	{
		public:
			ProgramWindow* win;

			ScreenshotGrabber(ProgramWindow* win);

			void HandleEvent(Event* evt);
	};
};
