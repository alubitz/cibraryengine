#include "AutoMenuItem.h"

namespace CibraryEngine
{
	/*
	 * AutoMenuItem implementation; private methods and properties
	 */
	struct AutoMenuItem::Imp
	{
		struct ActionHandler : public EventHandler
		{
			void HandleEvent(Event* evt)
			{
				MenuSelectionEvent* mse = (MenuSelectionEvent*)evt;
				((AutoMenuItem*)mse->item)->DoAction(mse);
			}
		} handler;
	};




	/*
	 * AutoMenuItem methods
	 */
	AutoMenuItem::AutoMenuItem(ContentMan* content, string text, int row, bool selectable) : MenuItem(content)
	{
		int x1 = 100;
		int y1 = 100 + row * 30;
		int x2 = x1 + 200;
		int y2 = y1 + 16;

		SetRect(&x1, &y1, &x2, &y2);
		SetText(text);
		SetSelectable(selectable);

		imp = new Imp();

		Selected += &imp->handler;
	}

	void AutoMenuItem::InnerDispose()
	{
		MenuItem::InnerDispose();

		Selected -= &imp->handler;

		delete imp;
		imp = NULL;
	}

	void AutoMenuItem::DoAction(MenuSelectionEvent* mse) { }
}
