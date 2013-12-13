#pragma once

#include "StdAfx.h"

namespace DoodAnimTool
{
	using namespace CibraryEngine;

	class GUIComponent
	{
		public:

			int w, h;
			int x1, y1, x2, y2;
			bool has_mouse;

			GUIComponent() : w(-1), h(-1), x1(-1), y1(-1), x2(-2), y2(-2), has_mouse(false) { }

			virtual void LayoutChildren() { }			// recursively calls this on its children to determine their x1,y1,x2,y2, to determine our w,h
			virtual void Draw(int cx1, int cy1, int cx2, int cy2) { }

			virtual GUIComponent* GetComponentAtPos(int x, int y) { return this; }

			virtual bool OnClick(int x, int y) { return false; }
	};
}
