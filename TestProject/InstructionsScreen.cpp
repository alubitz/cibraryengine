#include "StdAfx.h"
#include "InstructionsScreen.h"

namespace Test
{
	struct InstructionsScreen::Imp
	{
		struct BackButton : public AutoMenuItem
		{
			BackButton (ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		vector<AutoMenuItem*> auto_menu_items;

		Imp(InstructionsScreen* menu) : auto_menu_items()
		{
			ContentMan* content = menu->content;

			int row = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Instructions", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "Movement", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    W/S - Walk forward/backward", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    A/D - Sidestep", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Space - Jump", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "Jetpack", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Hold Space to use your jetpack", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    While holding Space you can move laterally with WASD", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Your jetpack runs out of energy while you're using it", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Crashing into terrain at high speed could seriously injure or even kill you", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "Aiming", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Aim with the mouse", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    The direction you aim is the same as the direction you look and walk", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "Weapon", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Left mouse button - Fire", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    R - Reload", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "General", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Esc - Return to the main menu", row++, false));
			row++;
			auto_menu_items.push_back(new BackButton(content, row++));

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
	 * InstructionsScreen methods
	 */
	InstructionsScreen::InstructionsScreen(ProgramWindow* win, ProgramScreen* previous) : MenuScreen(win, previous), imp(NULL) { }

	void InstructionsScreen::Activate()
	{
		MenuScreen::Activate();

		if(imp == NULL)
			imp = new Imp(this);
	}

	void InstructionsScreen::Deactivate()
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
