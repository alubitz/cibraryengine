#pragma once

#include "StdAfx.h"

using namespace CibraryEngine;

namespace ConverterUtil
{
	class ConverterScreen : public ProgramScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			ConverterScreen(ProgramWindow* win);
			~ConverterScreen();

			void Activate();
			void Deactivate();

			void Draw(int width, int height);
			ProgramScreen* Update(TimingInfo time);	
	};
}
