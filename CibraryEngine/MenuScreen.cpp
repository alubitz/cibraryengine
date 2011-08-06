#include "StdAfx.h"
#include "MenuScreen.h"

#include "ProgramWindow.h"
#include "Events.h"
#include "DebugLog.h"

#include "Content.h"
#include "BitmapFont.h"
#include "Cursor.h"

#include "MenuItem.h"

namespace CibraryEngine
{
	/*
	 * MenuScreen implementation; private methods and properties
	 */
	struct MenuScreen::Imp
	{
		MenuScreen* handle;

		bool no_delete;
		ProgramWindow* window;
		ProgramScreen* previous;
		ProgramScreen* next;

		BitmapFont* font;
		Cursor* cursor;

		MenuItem* selected_item;
		vector<MenuItem*> menu_items;

		ContentMan* content;
		InputState* input_state;

		Imp(MenuScreen* handle, ProgramWindow* window, ProgramScreen* previous) : handle(handle), no_delete(false), window(window), previous(previous), next(handle), font(NULL), cursor(NULL), selected_item(NULL), menu_items(), content(handle->content), input_state(handle->input_state), click_handler(this), key_handler(this) { }

		void Draw(int w, int h)
		{
			GLDEBUG();

			// By overriding MenuScreen::Draw you can prevent this content from being loaded
			if(font == NULL)
				font = content->GetCache<BitmapFont>()->Load("../Font");
			if(cursor == NULL)
				cursor = content->GetCache<Cursor>()->Load("Cursor");

			glViewport(0, 0, w, h);

			glDepthMask(true);
			glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
			GLDEBUG();
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			ScreenSpaceOrtho(w, h);
			ModelviewIdentity();

			glDisable(GL_DEPTH_TEST);
			glDepthMask(false);

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			// draw menu items
			for(unsigned int i = 0; i < menu_items.size(); i++)
			{
				MenuItem* item = menu_items[i];
				item->Draw(w, h);
			}

			// draw cursor only when window has "mouse focus"
			if(window->IsActive())
			{
				glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
				cursor->Draw((float)input_state->mx, (float)input_state->my);
			}

			GLDEBUG();
		}

		void ScreenSpaceOrtho(int w, int h)
		{
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			glOrtho(0, w, h, 0, -1, 1);
		}

		void ModelviewIdentity()
		{
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
		}

		void Activate()
		{
			window->input_state->MouseButtonStateChanged += &click_handler;
			window->input_state->KeyStateChanged += &key_handler;
		}

		void Deactivate()
		{
			window->input_state->MouseButtonStateChanged -= &click_handler;
			window->input_state->KeyStateChanged -= &key_handler;
		}

		void Destroy() { }

		// listen for when the user clicks on stuff
		struct ClickHandler : public EventHandler
		{
			Imp* imp;

			ClickHandler(Imp* imp) : imp(imp) { }

			void HandleEvent(Event* evt)
			{
				MouseButtonStateEvent* mbse = (MouseButtonStateEvent*)evt;
				if(mbse->button == 0 && mbse->state)
				{
					if(imp->selected_item != NULL)
					{
						MenuSelectionEvent mse = MenuSelectionEvent(imp->handle, imp->selected_item, imp->window->input_state->mx, imp->window->input_state->my);
						imp->selected_item->Selected(&mse);
					}
				}
			}

		} click_handler;

		struct KeyHandler : public EventHandler
		{
			Imp* imp;

			KeyHandler(Imp* imp) : imp(imp) { }

			void HandleEvent(Event* evt)
			{
				KeyStateEvent* kse = (KeyStateEvent*)evt;
				if(kse->state && kse->key == VK_ESCAPE)
					imp->next = imp->previous;							// exit back to whatever previous program screen was displayed
			}
		} key_handler;

		void UpdateSelection()
		{
			int mx = window->input_state->mx;
			int my = window->input_state->my;

			// find out what, if anything, the cursor is over
			MenuItem* selection = NULL;
			for(unsigned int i = 0; i < menu_items.size(); i++)
			{
				MenuItem* item = menu_items[i];
				if(selection == NULL && item->IsSelectable() && item->RectContains(mx, my))
				{
					selection = item;
					selection->SetHover(true);
				}
				else
					item->SetHover(false);
			}

			selected_item = selection;
		}

		ProgramScreen* Update(TimingInfo time)
		{
			UpdateSelection();

			// update menu items
			for(unsigned int i = 0; i < menu_items.size(); i++)
				menu_items[i]->Update(time);

			return next;
		}
	};




	/*
	 * MenuScreen methods
	 */
	MenuScreen::MenuScreen(ProgramWindow* window, ProgramScreen* previous) : ProgramScreen(window), imp(new Imp(this, window, previous)) { }
	MenuScreen::~MenuScreen() { }

	ProgramScreen* MenuScreen::GetPreviousScreen() { return imp->previous; }
	void MenuScreen::SetPreviousScreen(ProgramScreen* screen) { imp->previous = screen; }

	void MenuScreen::Draw(int width, int height) { imp->Draw(width, height); }

	ProgramScreen* MenuScreen::Update(TimingInfo time) { return imp->Update(time); }

	ProgramScreen* MenuScreen::GetNextScreen() { return imp->next; }
	void MenuScreen::SetNextScreen(ProgramScreen* screen) { imp->next = screen; }

	void MenuScreen::Activate()
	{
		if(imp == NULL)
			imp = new Imp(this, window, NULL);			// uh... not sure this is the best solution

		imp->Activate();
	}

	void MenuScreen::Deactivate()
	{
		if(imp != NULL)
		{
			imp->Deactivate();
			if(!imp->no_delete)
			{
				imp->Destroy();

				delete imp;
				imp = NULL;
			}
		}
	}

	unsigned int MenuScreen::GetItemCount()
	{
		return imp->menu_items.size();
	}

	MenuItem* MenuScreen::GetItem(unsigned int index)
	{
		if(index > imp->menu_items.size())
			return NULL;

		return imp->menu_items[index];
	}

	MenuItem* MenuScreen::RemoveLast()
	{
		if(imp->menu_items.size() == 0)
			return NULL;

		MenuItem* item = imp->menu_items[imp->menu_items.size() - 1];
		imp->menu_items.pop_back();

		return item;
	}

	void MenuScreen::AddItem(MenuItem* item)
	{
		imp->menu_items.push_back(item);
	}
}
