#include "StdAfx.h"
#include "MenuItem.h"

#include "TimingInfo.h"
#include "BitmapFont.h"

#include "Content.h"

namespace CibraryEngine
{
	/*
	 * MenuSelectionEvent methods
	 */
	MenuSelectionEvent::MenuSelectionEvent(MenuScreen* menu, MenuItem* item, int mx, int my) : menu(menu), item(item), mx(mx), my(my) { }




	/*
	 * MenuItem implementation; private methods and properties
	 */
	struct MenuItem::Imp
	{
		ContentMan* content;
		BitmapFont* font;

		int x1, y1, x2, y2;
		string text;

		bool selectable;
		bool hover;
		float hover_phase;

		void* user_pointer;

		Imp(ContentMan* content) : content(content), font(NULL), x1(-1), y1(-1), x2(-2), y2(-2), text(), selectable(true), hover(false), hover_phase(0), user_pointer(NULL) { }			// initial dimensions are impossible to click (on purpose)
	};




	/*
	 * MenuItem methods
	 */
	MenuItem::MenuItem(ContentMan* content) : Disposable(), imp(new Imp(content)), Selected() { }

	void MenuItem::InnerDispose()
	{
		if(imp != NULL)
		{
			delete imp;
			imp = NULL;
		}
	}

	void MenuItem::Draw(int screen_w, int screen_h)
	{
		if(imp->font == NULL)											// By overriding this function you can prevent this font from being loaded
			imp->font = imp->content->GetCache<BitmapFont>()->Load("../Font");

		float br = imp->selectable ? imp->hover_phase : 1.0f;
		glColor3f(0.5f + 0.5f * br, 0.5f + 0.5f * br, 1.0f);

		imp->font->Print(imp->text, (float)imp->x1, (float)imp->y1);
	}

	void MenuItem::Update(TimingInfo time)
	{
		float change_rate = time.elapsed * 3.0f;
		if(imp->selectable && imp->hover)
			imp->hover_phase = min(1.0f, imp->hover_phase + change_rate);
		else
			imp->hover_phase = max(0.0f, imp->hover_phase - change_rate);
	}

	bool MenuItem::RectContains(int x, int y) { return (x >= imp->x1 && y >= imp->y1 && x <= imp->x2 && y <= imp->y2); }

	string MenuItem::GetText() { return imp->text; }

	void MenuItem::GetRect(int* x1, int* y1, int* x2, int* y2)
	{
		if(x1 != NULL)
			*x1 = imp->x1;
		if(y1 != NULL)
			*y1 = imp->y1;
		if(x2 != NULL)
			*x2 = imp->x2;
		if(y2 != NULL)
			*y2 = imp->y2;
	}

	void MenuItem::GetSize(int* w, int* h)
	{
		if(w != NULL)
			*w = imp->x2 - imp->x1;			// not +1?
		if(h != NULL)
			*h = imp->y2 - imp->y1;			// not +1?
	}

	bool MenuItem::IsSelectable() { return imp->selectable; }

	bool MenuItem::GetHover() { return imp->hover; }

	void* MenuItem::GetUserPointer() { return imp->user_pointer; }

	void MenuItem::SetText(string text) { imp->text = text; }

	void MenuItem::SetRect(int* x1, int* y1, int* x2, int* y2)
	{
		if(x1 != NULL)
			imp->x1 = *x1;
		if(y1 != NULL)
			imp->y1 = *y1;
		if(x2 != NULL)
			imp->x2 = *x2;
		if(y2 != NULL)
			imp->y2 = *y2;
	}

	void MenuItem::SetSize(int* w, int* h)
	{
		if(w != NULL)
			imp->x2 = imp->x1 + *w;			// not -1?
		if(h != NULL)
			imp->y2 = imp->y1 + *h;			// not -1?
	}

	void MenuItem::SetSelectable(bool s) { imp->selectable = s; }

	void MenuItem::SetHover(bool hover) { imp->hover = hover; }

	void MenuItem::SetUserPointer(void* ptr) { imp->user_pointer = ptr; }
}