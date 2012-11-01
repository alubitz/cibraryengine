#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "../CibraryEngine/HardwareAcceleratedComputation.h"

namespace Test
{
	/*
	 * ExperimentalScreen implementation
	 */
	struct ExperimentalScreen::Imp
	{
		HardwareAcceleratedComputation* comp;

		struct BackButton : public AutoMenuItem
		{
			BackButton (ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		vector<AutoMenuItem*> auto_menu_items;

		Imp(ExperimentalScreen* menu) : comp(NULL), auto_menu_items()
		{
			ContentMan* content = menu->content;

			int index = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Experimental Screen", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------",	index++, false));
			auto_menu_items.push_back(new BackButton(content, index));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				menu->AddItem(auto_menu_items[i]);

			Shader* shader = content->GetCache<Shader>()->Load("hardware_comp-v");

			vector<const GLchar*> varying_names;
			varying_names.push_back("gl_Position");
			varying_names.push_back("derp");

			comp = new HardwareAcceleratedComputation(shader, varying_names);
		}

		void Destroy()
		{
			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
			{
				auto_menu_items[i]->Dispose();
				delete auto_menu_items[i];
			}
			auto_menu_items.clear();

			if(comp) { delete comp; comp = NULL; }
		}
	};




	/*
	 * ExperimentalScreen methods
	 */
	ExperimentalScreen::ExperimentalScreen(ProgramWindow* win, ProgramScreen* previous) : MenuScreen(win, previous), imp(NULL) { }

	void ExperimentalScreen::Activate()
	{
		MenuScreen::Activate();

		if(!imp)
			imp = new Imp(this);
	}

	void ExperimentalScreen::Deactivate()
	{
		MenuScreen::Deactivate();

		if(imp) { imp->Destroy(); delete imp; imp = NULL; }
	}

	void ExperimentalScreen::Draw(int width, int height)
	{
		MenuScreen::Draw(width, height);

		imp->comp->Process();
	}
}
