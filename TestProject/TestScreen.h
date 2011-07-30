#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class TestGame;

	class TestScreen : public ProgramScreen
	{
		private:

			int width, height;

			bool finished;

		public:

			TestGame* test_game;

			TestScreen(ProgramWindow* window);
			TestScreen(ProgramWindow* window, TestGame* test_game);

			void Draw(int width_, int height_);
			ProgramScreen* Update(TimingInfo time);

			void Activate();
			void Deactivate();
	};
}
