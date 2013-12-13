#include "StdAfx.h"
#include "GCheckbox.h"

namespace DoodAnimTool
{
	/*
	 * GCheckbox methods and constants
	 */
	static const string checkbox_unselected_text = "[ ]";
	static const string checkbox_selected_text   = "[x]";

	GCheckbox::GCheckbox(BitmapFont* font) :
		GLabel(font, checkbox_unselected_text, true),
		selected(false)
	{
		w = int(font->font_spacing * 3);
		h = int(font->font_height);
	}

	void GCheckbox::Draw(int cx1, int cy1, int cx2, int cy2)
	{
		text = selected ? checkbox_selected_text : checkbox_unselected_text;

		GLabel::Draw(cx1, cy1, cx2, cy2);
	}
}
