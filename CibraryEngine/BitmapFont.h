#pragma once

#include "StdAfx.h"

#include "Disposable.h"

#include "Content.h"

namespace CibraryEngine
{
	using namespace std;

	class Texture2D;

	/** Class for drawing text using bitmap fonts */
	class BitmapFont : public Disposable
	{
		private:

			unsigned int first_display_list;

		protected:

			void InnerDispose();

		public:

			/** The texture for the font */
			Texture2D* texture;
			/** The display height of each character */
			float font_height;
			/** The horizontal spacing for each character */
			float font_spacing;

			/** Initializes a BitmapFont handle */
			BitmapFont(unsigned int first_display_list, float height, float spacing, Texture2D* texture);

			/** Displays text at the specified location */
			void Print(string text, float x, float y);
	};

	struct BitmapFontLoader : public ContentTypeHandler<BitmapFont>
	{
		BitmapFontLoader(ContentMan* man);

		BitmapFont* Load(ContentMetadata& what);
		void Unload(BitmapFont* content, ContentMetadata& meta);
	};
}
