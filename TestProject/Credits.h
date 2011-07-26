#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace CibraryEngine;

	class Credits : public MenuScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			Credits(ProgramWindow* win, ProgramScreen* previous);

			void Activate();
			void Deactivate();
	};
}
