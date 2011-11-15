#include "StdAfx.h"
#include "ConsoleScreen.h"

namespace ConverterUtil
{
	using namespace CibraryEngine;

	/*
	 * ConsoleScreen private implementation struct
	 */
	struct ConsoleScreen::Imp
	{
		ConsoleScreen* screen;

		bool exit;

		BitmapFont* font;

		vector<string> scrollback;
		string current_line;

		Imp(ConsoleScreen* screen) :
			screen(screen),
			exit(false),
			font(screen->content->GetCache<BitmapFont>()->Load("../Font")),
			key_handler(this),
			scrollback(),
			current_line()
		{
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

			float x_size = font->font_spacing, y_size = font->font_height;

			glColor4f(1.0, 1.0, 1.0, 1.0);
			unsigned int count = scrollback.size();
			for(unsigned int i = 0; i < count; i++)
				font->Print(scrollback[i], -4, height + (float(i) - count - 1) * y_size);

			font->Print(current_line, -4, height - y_size);

			float cursor_x = current_line.length() * x_size;
			float cursor_y = height - y_size;

			glBlendFunc(GL_ONE_MINUS_DST_COLOR, GL_ZERO);

			glDisable(GL_TEXTURE_2D);
			glBegin(GL_QUADS);
			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
			glVertex2f(cursor_x,			cursor_y			);
			glVertex2f(cursor_x,			cursor_y + y_size	);
			glVertex2f(cursor_x + x_size,	cursor_y + y_size	);
			glVertex2f(cursor_x + x_size,	cursor_y			);
			glEnd();
		}

		void ProcessCurrentLine()
		{
			scrollback.push_back(current_line);

			if(!current_line.empty())
			{
				string lowercase;
				for(unsigned int i = 0; i < current_line.size(); i++)
					lowercase += tolower(current_line[i]);
				
				if(lowercase.compare("exit") == 0)
					exit = true;
				else if(lowercase.compare("clear") == 0)
					scrollback.clear();
				else
					scrollback.push_back("> unrecognized command");

				current_line = "";
			}
		}

		struct KeyHandler : public EventHandler
		{
			ConsoleScreen::Imp* imp;
			KeyHandler(ConsoleScreen::Imp* imp) : imp(imp) { }

			void HandleEvent(Event* evt)
			{
				KeyStateEvent* kse = (KeyStateEvent*)evt;

				if(kse->state)
				{
					string& current_line = imp->current_line;
					switch(kse->key)
					{
						case VK_RETURN:

							imp->ProcessCurrentLine();

							break;

						case VK_BACK:

							if(!current_line.empty())
								current_line = current_line.substr(0, current_line.length() - 1);

						default:

							if(kse->key >= 32 && kse->key <= 127)
							{
								int key = kse->key;
								char c = (char)key;

								if(key >= 65 && key <= 95 && !imp->screen->input_state->keys[VK_SHIFT])
									c = (char)(key + 32);

								current_line += c;
							}
							break;
					}
				}
			}
		} key_handler;
	};




	/*
	 * ConsoleScreen methods
	 */
	ConsoleScreen::ConsoleScreen(ProgramWindow* win) : ProgramScreen(win) { imp = new Imp(this); }

	ConsoleScreen::~ConsoleScreen()
	{ 
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	void ConsoleScreen::Activate() { input_state->KeyStateChanged += &imp->key_handler; }
	void ConsoleScreen::Deactivate() { input_state->KeyStateChanged -= &imp->key_handler; }

	void ConsoleScreen::Draw(int width, int height) { imp->Draw(width, height); }

	ProgramScreen* ConsoleScreen::Update(TimingInfo time)
	{
		if(imp->exit)
			return NULL;
		else
			return this;
	}
}
