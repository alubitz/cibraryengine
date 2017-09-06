#include "StdAfx.h"
#include "MainMenu.h"

#include "LoadingScreen.h"
#include "InstructionsScreen.h"
#include "Credits.h"

#include "ExperimentalScreen.h"

namespace Test
{
	/*
	 * MainMenu implementation
	 */
	struct MainMenu::Imp
	{
		struct CreateGameButton : public AutoMenuItem
		{
			//CreateGameButton(ContentMan* content, int row) : AutoMenuItem(content, "Create Game...", row, true) { }
			CreateGameButton(ContentMan* content, int row) : AutoMenuItem(content, "New Game", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(new LoadingScreen(mse->menu->window, mse->menu, NR_Server)); }
		};

		struct JoinGameButton : public AutoMenuItem
		{
			JoinGameButton(ContentMan* content, int row) : AutoMenuItem(content, "Join Game...", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(new LoadingScreen(mse->menu->window, mse->menu, NR_Client)); }
		};

		struct ExperimentalButton : public AutoMenuItem
		{
			ExperimentalButton(ContentMan* content, int row) : AutoMenuItem(content, "Experimental Screen", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(new ExperimentalScreen(mse->menu->window, mse->menu)); }
		};

		struct InstructionsButton : public AutoMenuItem
		{
			InstructionsButton(ContentMan* content, int row) : AutoMenuItem(content, "How to Play", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(new InstructionsScreen(mse->menu->window, mse->menu)); }
		};

		struct CreditsButton : public AutoMenuItem
		{
			CreditsButton(ContentMan* content, int row) : AutoMenuItem(content, "Credits", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(new Credits(mse->menu->window, mse->menu)); }
		};

		struct ExitButton : public AutoMenuItem
		{
			ExitButton(ContentMan* content, int row) : AutoMenuItem(content, "Exit", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(NULL); }
		};

		vector<AutoMenuItem*> auto_menu_items;

		Imp(ProgramWindow* win, MainMenu* menu) : auto_menu_items()
		{
			ContentMan* content = win->content;

			unsigned int row = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "FPS", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", row++, false));
			auto_menu_items.push_back(new CreateGameButton(content, row++));
			//auto_menu_items.push_back(new JoinGameButton(content, row++));
			//auto_menu_items.push_back(new AutoMenuItem(content, "Options...", row++, true));		// default implementation does nothing when selected
			auto_menu_items.push_back(new ExperimentalButton(content, row++));
			auto_menu_items.push_back(new InstructionsButton(content, row++));
			auto_menu_items.push_back(new CreditsButton(content, row++));
			auto_menu_items.push_back(new ExitButton(content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				menu->AddItem(auto_menu_items[i]);
		}

		void Destroy()
		{
			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
			{
				auto_menu_items[i]->Dispose();
				delete auto_menu_items[i];
			}

			auto_menu_items.clear();
		}
	};




	/*
	 * MainMenu methods
	 */
	MainMenu::MainMenu(ProgramWindow* win, ProgramScreen* previous) : MenuScreen(win, previous), imp(NULL) { }

	void MainMenu::Activate()
	{
		MenuScreen::Activate();

		if(imp == NULL)
			imp = new Imp(window, this);
	}

	void MainMenu::Deactivate()
	{
		MenuScreen::Deactivate();
		if(imp)
		{
			imp->Destroy();

			delete imp;
			imp = NULL;
		}
	}

	void MainMenu::Draw(int width, int height)
	{
		MenuScreen::Draw(width, height);

		//glViewport(0, 0, 200, 200);

		//glMatrixMode(GL_MODELVIEW);
		//glLoadIdentity();

		//glMatrixMode(GL_PROJECTION);
		//glLoadIdentity();
		//glOrtho(0, 200, 0, 200, -1, 1);

		//glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

		//glDisable(GL_TEXTURE_2D);
		//glDisable(GL_LINE_SMOOTH);
		//glDisable(GL_BLEND);
		//glLineWidth(1.0f);

		//glBegin(GL_QUADS);
		//glColor3f(0.25f, 0.25f, 0.25f);
		//glVertex2i(-100, -100);
		//glVertex2i(-100,  300);
		//glVertex2i( 300,  300);
		//glVertex2i( 300, -100);
		//glEnd();

		//glBegin(GL_LINES);
		//glColor3f(1, 0, 0);
		//glVertex2i(0, 0);
		//glVertex2i(200, 200);

		//glColor3f(0, 1, 0);
		//glVertex2i(200, 0);
		//glVertex2i(0, 200);
		//
		//glEnd();
	}
}
