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

			int index = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Credits", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "Programming", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Andrew Lubitz", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "Artwork", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Andrew Lubitz", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    Trevor Finney", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "Instructor", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    M. Q. Azhar", index++, false));
			index++;
			auto_menu_items.push_back(new AutoMenuItem(content, "Made at the Art Institute of Washington", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    VGP350 Skeletal Animation", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    VGP215 Programming for Shading and Dynamics", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    VGP Independent Study", index++, false));
			index++;
			auto_menu_items.push_back(new AutoMenuItem(content, "Contact Info", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    aml316@stu.aii.edu", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "    andrew.lubitz@gmail.com", index++, false));
			index++;
			auto_menu_items.push_back(new BackButton(content, index));

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
