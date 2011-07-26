#pragma once

#include "StdAfx.h"

#include "ProgramScreen.h"

namespace CibraryEngine
{
	class MenuItem;

	/** Class for creating a menu */
	class MenuScreen : public ProgramScreen
	{
		private:

			struct Imp;
			Imp* imp;

		public:

			/** Initializes a MenuScreen belonging to the specified ProgramWindow, with the specified previous screen (see GetPreviousScreen) */
			MenuScreen(ProgramWindow* window, ProgramScreen* previous);
			virtual ~MenuScreen();

			/** Get the screen to which the user will return if they cancel (e.g. pressing escape) */
			virtual ProgramScreen* GetPreviousScreen();					// the screen to return to if the user cancels (e.g. pressing escape)
			/** Set the screen to which the user will return if they cancel (e.g. pressing escape) */
			virtual void SetPreviousScreen(ProgramScreen* screen);

			/** Draws the menu */
			virtual void Draw(int width, int height);

			virtual ProgramScreen* Update(TimingInfo time);				// returns the next screen to be displayed

			/** Gets the next screen to be displayed (i.e. the screen to be returned by MenuScreen::Update) */
			virtual ProgramScreen* GetNextScreen();						// the screen to be on when the next frame comes along (return "this" to not change)
			/** Sets the next screen to be displayed (i.e. the screen to be returned by MenuScreen::Update) */
			virtual void SetNextScreen(ProgramScreen* screen);

			virtual void Activate();									// do loading here, rather than in the constructor
			virtual void Deactivate();

			/** Gets the number of menu items on this menu */
			unsigned int GetItemCount();
			/** Gets the MenuItem with the specified index */
			MenuItem* GetItem(unsigned int index);
			/** Removes the last MenuItem in this menu */
			MenuItem* RemoveLast();
			/** Adds the specified MenuItem */
			void AddItem(MenuItem* item);
	};
}
