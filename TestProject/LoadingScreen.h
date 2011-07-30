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

			LoadingScreen(ProgramWindow* window, ProgramScreen* previous);
			~LoadingScreen();

			void Draw(int width, int height);
			ProgramScreen* Update(TimingInfo time);

			void Activate();
			void Deactivate();
	};
}
