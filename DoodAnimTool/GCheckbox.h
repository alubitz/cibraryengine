#pragma once

#include "StdAfx.h"

#include "GLabel.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class GCheckbox : public GLabel
	{
		public:

			string label_text;
			bool selected;

			GCheckbox(BitmapFont* font, const string& label_text);

			void Draw(int cx1, int cy1, int cx2, int cy2);

			virtual void OnClick(int x, int y) { selected = !selected; }			// checkboxes should subclass and override this to do something useful
	};
}
