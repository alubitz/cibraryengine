#pragma once

#include "StdAfx.h"

#include "GUIComponent.h"

namespace DoodAnimTool
{
	/**
	 * @brief Describes a label widget. Generally static text, but often (ab)used for dynamic text in readouts
	 */
	class GLabel : public GUIComponent
	{
		public:
			BitmapFont* font;	//!< What font to use
			string text;		//!< Text value to display

			bool hover_glows;

			GLabel() { }
			GLabel(BitmapFont* font, const string& text, bool hover_glows = false);

			void Draw(int cx1, int cy1, int cx2, int cy2);

			void Measure();
			void SetText(const string& text);			// sets text and remeasures width and height
	};
}
