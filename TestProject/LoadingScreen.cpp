#include "StdAfx.h"
#include "LoadingScreen.h"

#include "TestGame.h"
#include "TestScreen.h"

#include "MainMenu.h"

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

		thread* my_thread;

		InputState* input_state;

		Imp(LoadingScreen* scr, ProgramScreen* previous, NetworkRole role) :
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
			test_game->network_role = role;
		}

		~Imp() { }

		void Draw(int w, int h)
		{
			if(font == NULL)
				font = window->content->GetCache<BitmapFont>()->Load("../Font");
			if(cursor == NULL)
				cursor = window->content->GetCache<Cursor>()->Load("Cursor");

			string task;
			test_game->load_status.GetTask(task);

			glViewport(0, 0, w, h);

			glDepthMask(true);
			glColorMask(true, true, true, true);
			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

			ScreenSpaceOrtho(w, h);
			ModelviewIdentity();

			glDisable(GL_DEPTH_TEST);
			glDepthMask(false);

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			font->Print("Loading...", 100, 100);
			font->Print(task, 100, 100 + font->font_height * 2);

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

		ProgramScreen* Update(const TimingInfo& time)
		{
			if(test_game->load_status.HasStopped())
			{
				if(test_game->load_status.HasAborted())
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
				my_thread = new thread(test_game->load_status);

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
					imp->test_game->load_status.Abort();
			}
		} key_handler;
	};




	/* LoadingScreen methods */
	LoadingScreen::LoadingScreen(ProgramWindow* window, ProgramScreen* previous, NetworkRole role) :
		ProgramScreen(window),
		imp(new Imp(this, previous, role))
	{
	}

	LoadingScreen::~LoadingScreen() { delete imp; }

	void LoadingScreen::Draw(int width, int height) { imp->Draw(width, height); }

	ProgramScreen* LoadingScreen::Update(const TimingInfo& time) { return imp->Update(time); }

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
