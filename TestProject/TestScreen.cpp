#include "StdAfx.h"

#include <ctime>

#include "TestScreen.h"
#include "TestGame.h"

#include "MainMenu.h"

namespace Test
{
	using namespace CibraryEngine;

	TestScreen::TestScreen(ProgramWindow* window) : ProgramScreen(window), finished(false), test_game(NULL) { }

	TestScreen::TestScreen(ProgramWindow* window, TestGame* test_game) : ProgramScreen(window), finished(false), test_game(test_game) { }

	void TestScreen::Activate()
	{
		if(test_game == NULL)
			test_game = new TestGame(this, window->sound_system);
	}

	ProgramScreen* TestScreen::Update(TimingInfo time)
	{
		test_game->Update(time);

		if(input_state->keys[VK_ESCAPE])
			finished = true;

		if(finished)
			return new MainMenu(window, NULL);
		else
			return this;
	}

	void TestScreen::Draw(int width_, int height_)
	{
		width = width_;
		height = height_;

		test_game->Draw(width, height);
	}

	void TestScreen::Deactivate()
	{
		if(test_game != NULL)
		{
			test_game->Dispose();
			delete test_game;
			test_game = NULL;
		}
	}
}
