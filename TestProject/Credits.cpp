#include "StdAfx.h"
#include "Credits.h"

namespace Test
{
	/*
	 * Credits implementation
	 */
	struct Credits::Imp
	{
		struct BackButton : public AutoMenuItem
		{
			BackButton (ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		vector<AutoMenuItem*> auto_menu_items;

		Imp(Credits* menu) : auto_menu_items()
		{
			ContentMan* content = menu->content;

			auto_menu_items.push_back(new AutoMenuItem(content, "Credits", 0, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", 1, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "Programming", 2, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Andrew Lubitz", 3, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "Artwork", 4, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Andrew Lubitz", 5, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Ritch Rivera", 6, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Trevor Finney", 7, false));

			auto_menu_items.push_back(new BackButton(content, 9));

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
	 * Credits methods
	 */
	Credits::Credits(ProgramWindow* win, ProgramScreen* previous) : MenuScreen(win, previous), imp(NULL) { }

	void Credits::Activate()
	{
		MenuScreen::Activate();

		if(imp == NULL)
			imp = new Imp(this);
	}

	void Credits::Deactivate()
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
