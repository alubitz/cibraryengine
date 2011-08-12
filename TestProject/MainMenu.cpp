#include "StdAfx.h"
#include "MainMenu.h"

#include "LoadingScreen.h"
#include "InstructionsScreen.h"
#include "Credits.h"
//#include "TestScreen.h"

namespace Test
{
	/*
	 * MainMenu implementation
	 */
	struct MainMenu::Imp
	{
		struct NewGameButton : public AutoMenuItem
		{
			NewGameButton(ContentMan* content, int row) : AutoMenuItem(content, "New Game...", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(new LoadingScreen(mse->menu->window, mse->menu)); }
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

			int row = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "FPS", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", row++, false));
			auto_menu_items.push_back(new NewGameButton(content, row++));
			//auto_menu_items.push_back(new AutoMenuItem(content, "Options...", row++, true));		// default implementation does nothing when selected
			auto_menu_items.push_back(new InstructionsButton(content, row++));
			auto_menu_items.push_back(new CreditsButton(content, row++));
			auto_menu_items.push_back(new ExitButton(content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); i++)
				menu->AddItem(auto_menu_items[i]);
		}

		void Destroy()
		{
			for(unsigned int i = 0; i < auto_menu_items.size(); i++)
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
		if(imp != NULL)
		{
			imp->Destroy();

			delete imp;
			imp = NULL;
		}
	}
}
