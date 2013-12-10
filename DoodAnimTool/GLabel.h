#pragma once

#include "StdAfx.h"

#include "GUIComponent.h"

namespace DoodAnimTool
{
	class GLabel : public GUIComponent
	{
		public:

			BitmapFont* font;
			string text;

			bool hover_glows;

			GLabel() { }
			GLabel(BitmapFont* font, const string& text, bool hover_glows = false);

			void Draw(int cx1, int cy1, int cx2, int cy2);
	};
}
