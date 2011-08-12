#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class InstructionsScreen : public MenuScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			InstructionsScreen(ProgramWindow* win, ProgramScreen* previous);

			void Activate();
			void Deactivate();
	};
}
