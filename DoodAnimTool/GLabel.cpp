#include "StdAfx.h"
#include "GLabel.h"

namespace DoodAnimTool
{
	/*
	 * GLabel methods
	 */
	GLabel::GLabel(BitmapFont* font, const string& text, bool hover_glows) : font(font), text(text), hover_glows(hover_glows) { Measure(); }

	void GLabel::Draw(int cx1, int cy1, int cx2, int cy2)
	{
		if(hover_glows && has_mouse)
			glColor4f(1.0f,	1.0f, 0.5f, 1.0f);
		else
			glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

		font->Print(text, float(cx1 + x1), float(cy1 + y1));

		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	}

	void GLabel::Measure()
	{
		w = int(font->font_spacing * text.size());
		h = int(font->font_height);
	}

	void GLabel::SetText(const string& text_) { text = text_; Measure(); }
}
