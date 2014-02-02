#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class LoadingScreen : public ProgramScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			LoadingScreen(ProgramWindow* window, ProgramScreen* previous, NetworkRole role);
			~LoadingScreen();

			void Draw(int width, int height);
			ProgramScreen* Update(const TimingInfo& time);

			void Activate();
			void Deactivate();
	};
}
