#pragma once

#include "StdAfx.h"

using namespace CibraryEngine;

namespace ConverterUtil
{
	class ConsoleScreen : public ProgramScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			ConsoleScreen(ProgramWindow* win);
			~ConsoleScreen();

			void Activate();
			void Deactivate();

			void Draw(int width, int height);
			ProgramScreen* Update(TimingInfo time);	
	};
}
