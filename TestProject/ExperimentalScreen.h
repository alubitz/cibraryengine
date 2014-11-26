#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class ExperimentalScreen : public MenuScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			ExperimentalScreen(ProgramWindow* win, ProgramScreen* previous);

			void Activate();
			void Deactivate();

			void Draw(int width, int height);

			ProgramScreen* Update(const TimingInfo& time);
	};
}
