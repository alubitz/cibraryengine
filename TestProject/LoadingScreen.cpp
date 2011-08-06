#include "StdAfx.h"
#include "LoadingScreen.h"

#include "TestGame.h"
#include "TestScreen.h"

#include "MainMenu.h"

#define BOOST_THREAD_USE_LIB
#include <boost/thread.hpp>

namespace Test
{
	using namespace CibraryEngine;

	using namespace std;

	/*
	 * LoadingScreen private implementation struct
	 */
	struct LoadingScreen::Imp
	{
		ProgramWindow* window;
		LoadingScreen* scr;

		TestScreen* test_screen;
		TestGame* test_game;

		BitmapFont* font;
		Cursor* cursor;

		ProgramScreen* next;
		ProgramScreen* previous;

		boost::thread* my_thread;

		InputState* input_state;

		Imp(LoadingScreen* scr, ProgramScreen* previous) :
			window(scr->window),
			scr(scr),
			test_screen(new TestScreen(window)),
			test_game(new TestGame(test_screen, window->sound_system)),
			font(NULL),
			cursor(NULL),
			next(scr),
			previous(previous),
			my_thread(NULL),
			input_state(scr->input_state),
			key_handler(this)
		{
			test_screen->test_game = test_game;
		}

		~Imp() { }

		void Draw(int w, int h)
		{
			if(font == NULL)
				font = window->content->GetCache<BitmapFont>()->Load("../Font");
			if(cursor == NULL)
				cursor = window->content->GetCache<Cursor>()->Load("Cursor");

			glViewport(0, 0, w, h);

			glDepthMask(true);
			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			ScreenSpaceOrtho(w, h);
			ModelviewIdentity();

			glDisable(GL_DEPTH_TEST);
			glDepthMask(false);

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			font->Print("Loading...", 100, 100);
			font->Print(test_game->load_status.task, 100, 100 + font->font_height * 2);

			// draw cursor
			if(window->IsActive())
			{
				glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
				cursor->Draw((float)input_state->mx, (float)input_state->my);
			}
		}

		void ScreenSpaceOrtho(int w, int h)
		{
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0, w, h, 0, -1, 1);
		}

		void ModelviewIdentity()
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
		}

		ProgramScreen* Update(TimingInfo time) 
		{
			if(test_game->load_status.stopped)
			{
				if(test_game->load_status.abort)
				{
					delete test_game;
					delete test_screen;

					next = previous;
				}
				else
					next = test_screen;
			}

			return next;
		}

		void Activate()
		{
			if(my_thread == NULL)
				my_thread = new boost::thread(test_game->load_status);

			window->input_state->KeyStateChanged += &key_handler;
		}

		void Deactivate()
		{
			window->input_state->KeyStateChanged -= &key_handler;
		}

		struct KeyHandler : public EventHandler
		{
			Imp* imp;

			KeyHandler(Imp* imp) : imp(imp) { }

			void HandleEvent(Event* evt)
			{
				KeyStateEvent* kse = (KeyStateEvent*)evt;
				if(kse->state && kse->key == VK_ESCAPE)
					imp->test_game->load_status.abort = true;
			}
		} key_handler;
	};




	/* LoadingScreen methods */
	LoadingScreen::LoadingScreen(ProgramWindow* window, ProgramScreen* previous) :
		ProgramScreen(window),
		imp(new Imp(this, previous))
	{
	}

	LoadingScreen::~LoadingScreen() { delete imp; }

	void LoadingScreen::Draw(int width, int height) { imp->Draw(width, height); }

	ProgramScreen* LoadingScreen::Update(TimingInfo time) { return imp->Update(time); }

	void LoadingScreen::Activate()
	{
		ProgramScreen::Activate();
		imp->Activate();
	}

	void LoadingScreen::Deactivate()
	{
		ProgramScreen::Deactivate();
		imp->Deactivate();
	}
}
