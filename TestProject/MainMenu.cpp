#include "MainMenu.h"

#include "Credits.h"
#include "LoadingScreen.h"
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

			auto_menu_items.push_back(new AutoMenuItem(content, "FPS", 0, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", 1, false));
			auto_menu_items.push_back(new NewGameButton(content, 2));
			auto_menu_items.push_back(new AutoMenuItem(content, "Options...", 3, true));		// default implementation does nothing when selected
			auto_menu_items.push_back(new CreditsButton(content, 4));
			auto_menu_items.push_back(new ExitButton(content, 5));

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
