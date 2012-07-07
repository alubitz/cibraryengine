#pragma once

#include "StdAfx.h"

namespace InverseKinematics
{
	using namespace CibraryEngine;

	class IKScreen : public ProgramScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			IKScreen(ProgramWindow* win);
			~IKScreen();

			ProgramScreen* Update(TimingInfo time);

			void Draw(int width, int height);
	};
}
