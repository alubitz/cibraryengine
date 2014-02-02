#pragma once

#include "StdAfx.h"
#include "MenuItem.h"

namespace CibraryEngine
{
	using namespace std;

	struct ContentMan;
	class MenuSelectionEvent;

	/** Type of MenuItem which automatically initializes position, text, and selectability during initialization, and which has a member function that is called when it is activated */
	class AutoMenuItem : public MenuItem
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			virtual void InnerDispose();

		public:

			/** Initializes an AutoMenuItem with the specified content manager, and with the specified caption text, on the specified row, which may or may not be selectable */
			AutoMenuItem(ContentMan* content, const string& text, int row, bool selectable);

			/** Virtual function to perform whatever action this AutoMenuItem is supposed to perform; override this! */
			virtual void DoAction(MenuSelectionEvent* mse);
	};
}
