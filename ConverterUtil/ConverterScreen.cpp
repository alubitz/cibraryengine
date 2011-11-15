#include "StdAfx.h"
#include "ConverterScreen.h"

namespace ConverterUtil
{
	using namespace CibraryEngine;

	/*
	 * ConverterScreen private implementation struct
	 */
	struct ConverterScreen::Imp
	{
		ConverterScreen* screen;

		bool exit;

		BitmapFont* font;
		UberModel* working_model;

		Imp(ConverterScreen* screen) :
			screen(screen),
			exit(false),
			font(screen->content->GetCache<BitmapFont>()->Load("../Font")),
			working_model(NULL),
			key_handler(this)
		{
			NewModel();
		}

		void Draw(int width, int height)
		{
			glViewport(0, 0, width, height);

			glDepthMask(true);
			glColorMask(true, true, true, false);

			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0, width, height, 0, -1, 1);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			glDisable(GL_DEPTH_TEST);
			glDisable(GL_CULL_FACE);
		}

		struct KeyHandler : public EventHandler
		{
			ConverterScreen::Imp* imp;
			KeyHandler(ConverterScreen::Imp* imp) : imp(imp) { }

			void HandleEvent(Event* evt)
			{
				KeyStateEvent* kse = (KeyStateEvent*)evt;

				if(kse->state)
				{
					switch(kse->key)
					{
						case VK_ESCAPE:

							imp->exit = true;
							break;
					}
				}
			}
		} key_handler;

		void NewModel()
		{
			if(working_model != NULL)
			{
				// TODO: ask if they want to save changes
				Debug("TODO: ask if they want to save changes\n");

				working_model->Dispose();
				delete working_model;
			}

			working_model = new UberModel();
			Debug("Created new model\n");
		}
	};




	/*
	 * ConverterScreen methods
	 */
	ConverterScreen::ConverterScreen(ProgramWindow* win) : ProgramScreen(win) { imp = new Imp(this); }

	ConverterScreen::~ConverterScreen()
	{ 
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	void ConverterScreen::Activate() { input_state->KeyStateChanged += &imp->key_handler; }
	void ConverterScreen::Deactivate() { input_state->KeyStateChanged -= &imp->key_handler; }

	void ConverterScreen::Draw(int width, int height) { imp->Draw(width, height); }

	ProgramScreen* ConverterScreen::Update(TimingInfo time)
	{
		if(imp->exit)
			return NULL;
		else
			return this;
	}
}
