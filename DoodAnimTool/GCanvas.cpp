#include "StdAfx.h"
#include "GCanvas.h"

namespace DoodAnimTool
{
	/*
	 * GCanvas::HAlign and ::VAlign methods
	 */
	GCanvas::HAlign::HAlign(GUIComponent* left, int padding) : mode(Left), other_a(left), padding(padding) { }
	GCanvas::HAlign::HAlign(int padding, GUIComponent* right) : mode(Right), other_a(right), padding(padding) { }
	GCanvas::HAlign::HAlign(GUIComponent* left, GUIComponent* right) : mode(Center), other_a(left), other_b(right) { }

	GCanvas::VAlign::VAlign(GUIComponent* top, int padding) : mode(Top), other_a(top), padding(padding) { }
	GCanvas::VAlign::VAlign(int padding, GUIComponent* bottom) : mode(Bottom), other_a(bottom), padding(padding) { }
	GCanvas::VAlign::VAlign(GUIComponent* top, GUIComponent* bottom) : mode(Center), other_a(top), other_b(bottom) { }




	/*
	 * GCanvas methods
	 */
	GCanvas::GCanvas() : children(), mouseover_item(NULL) { }

	void GCanvas::AddChild(GUIComponent* child, const HAlign& halign, const VAlign& valign) { children.push_back(AlignedChild(child, halign, valign)); }

	void GCanvas::Layout(int w, int h)
	{
		set<unsigned int> unknown_x, unknown_y, known_x, known_y;
		map<GUIComponent*, unsigned int> indices;
		for(unsigned int i = 0; i < children.size(); ++i)
		{
			GUIComponent* child = children[i].child;

			child->LayoutChildren();

			indices[child] = i;
			unknown_x.insert(i);
			unknown_y.insert(i);
		}

		// resolve horizontal positioning
		bool did_anything;
		do
		{
			did_anything = false;

			for(set<unsigned int>::iterator iter = unknown_x.begin(); iter != unknown_x.end();)
			{
				AlignedChild& child = children[*iter];
				const HAlign& align = child.halign;
				GUIComponent* comp = child.child;

				bool resolved_me = false;

				switch(align.mode)
				{
					case HAlign::Left:

						if(align.other_a == NULL)
						{
							comp->x1 = align.padding;
							resolved_me = true;
						}
						else if(known_x.find(indices[align.other_a]) != known_x.end())
						{
							comp->x1 = align.other_a->x2 + align.padding;
							resolved_me = true;
						}

						break;

					case HAlign::Center:

						if((align.other_a == NULL || known_x.find(indices[align.other_a]) != known_x.end()) && (align.other_b == NULL || known_x.find(indices[align.other_b]) != known_x.end()))
						{
							if(align.other_a == NULL)
								comp->x1 = 0;
							else
								comp->x1 = align.other_a->x2;

							if(align.other_b == NULL)
								comp->x2 = w;
							else
								comp->x2 = align.other_b->x1;

							comp->x1 = (comp->x1 + comp->x2) / 2 - comp->w / 2;
							resolved_me = true;
						}

						break;

					case HAlign::Right:

						if(align.other_a == NULL)
						{
							comp->x1 = w - align.padding - comp->w;
							resolved_me = true;
						}
						else if(known_x.find(indices[align.other_a]) != known_x.end())
						{
							comp->x1 = align.other_a->x1 - align.padding - comp->w;
							resolved_me = true;
						}

						break;

				}

				if(resolved_me)
				{
					comp->x2 = comp->x1 + comp->w;

					known_x.insert(*iter);
					iter = unknown_x.erase(iter);
				}
				else
					++iter;
			}
		} while(did_anything);

		// resolve vertical positioning
		do
		{
			did_anything = false;

			for(set<unsigned int>::iterator iter = unknown_y.begin(); iter != unknown_y.end();)
			{
				AlignedChild& child = children[*iter];
				const VAlign& align = child.valign;
				GUIComponent* comp = child.child;

				bool resolved_me = false;

				switch(align.mode)
				{
					case VAlign::Top:

						if(align.other_a == NULL)
						{
							comp->y1 = align.padding;
							resolved_me = true;
						}
						else if(known_y.find(indices[align.other_a]) != known_y.end())
						{
							comp->y1 = align.other_a->y2 + align.padding;
							resolved_me = true;
						}

						break;

					case VAlign::Center:

						if((align.other_a == NULL || known_y.find(indices[align.other_a]) != known_y.end()) && (align.other_b == NULL || known_y.find(indices[align.other_b]) != known_y.end()))
						{
							if(align.other_a == NULL)
								comp->y1 = 0;
							else
								comp->y1 = align.other_a->y2;

							if(align.other_b == NULL)
								comp->y2 = w;
							else
								comp->y2 = align.other_b->y1;

							comp->y1 = (comp->y1 + comp->y2) / 2 - comp->h / 2;
							resolved_me = true;
						}

						break;

					case VAlign::Bottom:

						if(align.other_a == NULL)
						{
							comp->y1 = h - align.padding - comp->h;
							resolved_me = true;
						}
						else if(known_y.find(indices[align.other_a]) != known_y.end())
						{
							comp->y1 = align.other_a->y1 - align.padding - comp->h;
							resolved_me = true;
						}

						break;

				}

				if(resolved_me)
				{
					comp->y2 = comp->y1 + comp->h;

					known_y.insert(*iter);
					iter = unknown_y.erase(iter);
				}
				else
					++iter;
			}
		} while(did_anything);
	}

	void GCanvas::Draw(int w, int h)
	{
		for(vector<AlignedChild>::iterator iter = children.begin(); iter != children.end(); ++iter)
		{
			GUIComponent* child = iter->child;
			child->Draw(0, 0, w, h);
		}
	}

	GUIComponent* GCanvas::GetComponentAtPos(int x, int y)
	{
		for(vector<AlignedChild>::iterator iter = children.begin(); iter != children.end(); ++iter)
		{
			GUIComponent* child = iter->child;
			if(x >= child->x1 && y >= child->y1 && x <= child->x2 && y <= child->y2)
				return child->GetComponentAtPos(x - child->x1, y - child->y1);
		}

		return this;
	}

	void GCanvas::OnMouseMoved(int x, int y)
	{
		GUIComponent* nu_mouseover = GetComponentAtPos(x, y);

		if(nu_mouseover != mouseover_item)
		{
			if(mouseover_item != NULL)
				mouseover_item->has_mouse = false;
			if(nu_mouseover != NULL)
				nu_mouseover->has_mouse = true;

			mouseover_item = nu_mouseover;
		}
	}

	void GCanvas::OnClick(int x, int y)
	{
		GUIComponent* clicky = GetComponentAtPos(x, y);
		if(clicky != this && clicky != NULL)
			clicky->OnClick(x - clicky->x1, y - clicky->y1);
	}
}
