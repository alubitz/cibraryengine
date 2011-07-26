#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace CibraryEngine;

	class MainMenu : public MenuScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			MainMenu(ProgramWindow* win, ProgramScreen* previous);

			void Activate();
			void Deactivate();
	};
}
