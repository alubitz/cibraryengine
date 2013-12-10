#include "StdAfx.h"
#include "GColumnList.h"

namespace DoodAnimTool
{
	/*
	 * GColumnList methods
	 */
	GColumnList::GColumnList(int row_padding, Align first_col_align) : row_padding(row_padding), columns(1), col_padding(), col_align(), rows()
	{
		col_align.push_back(first_col_align);
	}

	void GColumnList::AddColumn(int left_padding, Align align)
	{
		++columns;
		col_padding.push_back(left_padding);
		col_align.push_back(align);
	}

	void GColumnList::BeginRow() { rows.push_back(vector<GUIComponent*>()); }
	void GColumnList::AddToLastRow(GUIComponent* item) { rows.rbegin()->push_back(item); }

	void GColumnList::LayoutChildren()
	{
		vector<int> col_widths(columns);
		vector<int> row_heights(rows.size());

		for(unsigned int i = 0; i < columns; ++i)
			col_widths[i] = 0;

		for(unsigned int i = 0; i < rows.size(); ++i)
		{
			int row_height = 0;
			
			vector<GUIComponent*>& row = rows[i];
			for(unsigned int j = 0, row_end = min(row.size(), columns); j < row_end; ++j)
			{
				if(GUIComponent* item = row[j])
				{
					item->LayoutChildren();

					row_height    = max(item->h, row_height);
					col_widths[j] = max(item->w, col_widths[j]);
				}
			}

			row_heights[i] = row_height;
		}

		vector<int> col_lefts(columns), col_centers(columns), col_rights(columns);
		for(unsigned int i = 0; i < columns; ++i)
		{
			if(i == 0)
				col_lefts[i] = 0;
			else
				col_lefts[i] = col_rights[i - 1] + col_padding[i - 1];
			col_rights[i]  = col_lefts[i] + col_widths[i];
			col_centers[i] = (col_lefts[i] + col_rights[i]) / 2;
		}

		int y = 0;
		for(unsigned int i = 0; i < rows.size(); ++i)
		{
			vector<GUIComponent*>& row = rows[i];
			for(unsigned int j = 0, row_end = min(columns, row.size()); j < row_end; ++j)
				if(GUIComponent* item = row[j])
				{
					switch(col_align[j])
					{
						case Left:   { item->x1 = col_lefts  [j];               break; }
						case Center: { item->x1 = col_centers[j] - item->w / 2; break; }
						case Right:  { item->x1 = col_rights [j] - item->w;     break; }							
					}
					item->x2 = item->x1 + item->w;

					item->y1 = y;
					item->y2 = y + item->h;
				}

			y += row_heights[i] + row_padding;
		}
		
		w = col_rights[columns - 1];
		h = y - row_padding;
	}

	void GColumnList::Draw(int cx1, int cy1, int cx2, int cy2)
	{
		for(vector<vector<GUIComponent*>>::iterator iter = rows.begin(); iter != rows.end(); ++iter)
			for(vector<GUIComponent*>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
				if(GUIComponent* item = *jter)
					item->Draw(x1 + cx1, y1 + cy1, x1 + cx2, y1 + cy2);
	}

	GUIComponent* GColumnList::GetComponentAtPos(int x, int y)
	{
		for(vector<vector<GUIComponent*>>::iterator iter = rows.begin(); iter != rows.end(); ++iter)
			for(vector<GUIComponent*>::iterator jter = iter->begin(); jter != iter->end(); ++jter)
				if(GUIComponent* item = *jter)
				{
					if(x >= item->x1 && x <= item->x2 && y >= item->y1 && y <= item->y2)
					{
						GUIComponent* result = item->GetComponentAtPos(x - item->x1, y - item->y1);
						return result == NULL ? item : result;
					}
				}

		return this;
	}
}
