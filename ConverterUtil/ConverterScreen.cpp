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

		Imp(ConverterScreen* screen) :
			screen(screen),
			exit(false),
			key_handler(this)
		{
		}

		void Draw(int width, int height)
		{
			glViewport(0, 0, width, height);

			glDepthMask(true);
			glColorMask(true, true, true, false);

			glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		}

		struct KeyHandler : public EventHandler
		{
			ConverterScreen::Imp* imp;
			KeyHandler(ConverterScreen::Imp* imp) : imp(imp) { }

			void HandleEvent(Event* evt)
			{
				KeyStateEvent* kse = (KeyStateEvent*)evt;
				if(kse->state && kse->key == VK_ESCAPE)
					imp->exit = true;
			}
		} key_handler;
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
