#pragma once

#include "StdAfx.h"

#include "Disposable.h"
#include "Events.h"

namespace CibraryEngine
{
	using namespace std;

	struct TimingInfo;
	struct ContentMan;

	class MenuItem;
	class MenuScreen;

	/** Event for when the user selects an item from a menu */
	class MenuSelectionEvent : public Event
	{
		public:

			/** The menu from which the user selected an item */
			MenuScreen* menu;
			/** The MenuItem which they selected */
			MenuItem* item;

			/** The horizontal position of the cursor */
			int mx;
			/** The vertical position of the cursor */
			int my;

			/** Initializes all the fields of a MenuSelectionEvent */
			MenuSelectionEvent(MenuScreen* menu, MenuItem* item, int mx, int my);
	};

	/** Class representing an item on a MenuScreen */
	class MenuItem : public Disposable
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			virtual void InnerDispose();

		public:

			/** Initializes a MenuItem with the given content manager */
			MenuItem(ContentMan* content);

			/** Draws this MenuItem, given the dimensions of the menu */
			virtual void Draw(int screen_w, int screen_h);
			/** Updates the MenuItem */
			virtual void Update(const TimingInfo& time);

			/** Queries whether the specified point is within the clickable rectangular region for this MenuItem */
			bool RectContains(int x, int y);

			/** Gets the caption for this MenuItem */
			string GetText();
			/** Gets the coordinates of the top-left and bottom-right corners of the MenuItem; pass NULL to not use one or more of the outputs */
			void GetRect(int* x1, int* y1, int* x2, int* y2);
			/** Gets the dimensions of the MenuItem; pass NULL to not use one or more of the outputs */
			void GetSize(int* w, int* h);
			/** Returns whether this MenuItem may be selected */
			bool IsSelectable();
			/** Returns whether this MenuItem is currently selected (i.e. mouse is over it) */
			bool GetHover();
			/** Returns a pointer chosen by the user */
			void* GetUserPointer();

			/** Sets the caption for this MenuItem */
			void SetText(const string& text);
			/** Sets the coordinates of the top-left and bottom-right corners of the MenuItem; pass NULL to not change one or more of the values */
			void SetRect(int* x1, int* y1, int* x2, int* y2);
			/** Sets the dimensions of the MenuItem; pass NULL to not change one or more of the values */
			void SetSize(int* w, int* h);
			/** Sets whether this MenuItem may be selected */
			void SetSelectable(bool s);
			/** Sets whether this MenuItem is currently selected */
			void SetHover(bool hover);
			/** Set a pointer chosen by the user */
			void SetUserPointer(void* ptr);

			/** Event which occurs when this MenuItem is activated, i.e. when it's actually clicked on */
			EventDispatcher Selected;
	};
}
