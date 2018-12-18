#pragma once

#include "StdAfx.h"

#include "GUIComponent.h"

namespace DoodAnimTool
{
	/**
	 * @brief Describes a box sizer (specifically a column sizer). Attempts to align and lay out child widgets.
	 */
	class GColumnList : public GUIComponent
	{
		public:

			enum Align
			{
				Left,
				Center,
				Right
			};

			int row_padding;

			unsigned int columns;

			vector<int> col_padding;				// padding between each column; count = columns - 1
			vector<Align> col_align;				// alignment of each column;    count = columns

			vector<vector<GUIComponent*>> rows;

			GColumnList() { }
			GColumnList(int row_padding, Align first_col_align);
			
			void AddColumn(int left_padding, Align align);

			void BeginRow();
			void AddToLastRow(GUIComponent* item);

			void LayoutChildren();
			void Draw(int cx1, int cy1, int cx2, int cy2);

			GUIComponent* GetComponentAtPos(int x, int y);
	};
}
