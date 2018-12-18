#pragma once

#include "StdAfx.h"

#include "GLabel.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	/**
	 * @brief Describes a checkbox widget.
	 */
	class GCheckbox : public GLabel
	{
		public:
			bool selected;

			GCheckbox(BitmapFont* font);

			void Draw(int cx1, int cy1, int cx2, int cy2);

			virtual bool OnClick(int x, int y) { selected = !selected; return true; }			// checkboxes should subclass and override this to do something useful
	};
}
