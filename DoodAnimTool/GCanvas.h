#pragma once

#include "StdAfx.h"

#include "GUIComponent.h"

namespace DoodAnimTool
{
	/**
	 * @brief Describes a panel widget, on which all other widgets are drawn on. Supports some alignment.
	 */
	class GCanvas : public GUIComponent
	{
		public:

			struct HAlign
			{
				enum Mode { Left, Center, Right  } mode;
					
				GUIComponent* other_a;
				union { int padding; GUIComponent* other_b; };

				HAlign(GUIComponent* left, int padding);
				HAlign(int padding, GUIComponent* right);
				HAlign(GUIComponent* left, GUIComponent* right);
			};

			struct VAlign
			{
				enum Mode { Top, Center, Bottom } mode;

				GUIComponent* other_a;
				union { int padding; GUIComponent* other_b; };

				VAlign(GUIComponent* top, int padding);
				VAlign(int padding, GUIComponent* bottom);
				VAlign(GUIComponent* top, GUIComponent* bottom);
			};

			struct AlignedChild
			{
				GUIComponent* child;

				HAlign halign;
				VAlign valign;

				AlignedChild(GUIComponent* child, const HAlign& halign, const VAlign& valign) : child(child), halign(halign), valign(valign) { }
			};
			vector<AlignedChild> children;

			GUIComponent* mouseover_item;

			GCanvas();

			void AddChild(GUIComponent* child, const HAlign& halign, const VAlign& valign);

			void Layout(int w, int h);
			void Draw(int w, int h);

			GUIComponent* GetComponentAtPos(int x, int y);

			void OnMouseMoved(int x, int y);
			bool OnClick(int x, int y);
	};
}
