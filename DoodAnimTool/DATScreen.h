#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class DATScreen : public ProgramScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			DATScreen(ProgramWindow* win);
			~DATScreen();

			ProgramScreen* Update(const TimingInfo& time);

			void Draw(int width, int height);
	};
}
