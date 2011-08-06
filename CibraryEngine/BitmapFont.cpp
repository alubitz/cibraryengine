#include "StdAfx.h"

#include "BitmapFont.h"
#include "Texture2D.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * BitmapFont methods
	 */
	BitmapFont::BitmapFont(unsigned int first_display_list, float height, float spacing, Texture2D* texture) :
		first_display_list(first_display_list),
		texture(texture),
		font_height(height),
		font_spacing(spacing)
	{
	}

	void BitmapFont::Print(string text, float x, float y)
	{
		const int set = 0;			// if this were 1, it would use the bottom of the font texture

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, texture->GetGLName());
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_CULL_FACE);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glTranslatef(x, y + font_height, 0.0f);

		glListBase(first_display_list - 32 + (128 * set));

		int length = text.length();
		unsigned char* vals = new unsigned char[length];

		for (int i = 0; i < length; i++)
			vals[i] = (unsigned char)text[i];

		glCallLists(length, GL_UNSIGNED_BYTE, &vals[0]);
		delete[] vals;

		glPopMatrix();
	}

	void BitmapFont::InnerDispose() { }




	/*
	 * BitmapFontLoader methods
	 */
	BitmapFontLoader::BitmapFontLoader(ContentMan* man) : ContentTypeHandler<BitmapFont>(man), tex_cache(man->GetCache<Texture2D>()) { }

	BitmapFont* BitmapFontLoader::Load(ContentMetadata& what)
	{
		ContentHandle<Texture2D> texture_handle = tex_cache->GetHandle(what.name);
		tex_cache->ForceLoad(texture_handle);

		if(texture_handle.GetObject() == NULL)
			return NULL;

		float font_height = 16.0f;
		float font_spacing = font_height * 0.625f;

		glEnable(GL_TEXTURE_2D);

		float cx;
		float cy;

		unsigned int first_display_list = glGenLists(256);
		for (int i = 0; i < 256; i++)
		{
			cx = (float)(i % 16) / 16.0f;
			cy = (float)(i / 16) / 16.0f;

			glNewList(first_display_list + i, GL_COMPILE);

			glBegin(GL_QUADS);
			glTexCoord2f(cx, cy);
			glVertex2f(0.0f, -font_height);
			glTexCoord2f(cx + 0.0625f, cy);
			glVertex2f(font_height, -font_height);
			glTexCoord2f(cx + 0.0625f, cy + 0.0625f);
			glVertex2f(font_height, 0.0f);
			glTexCoord2f(cx, cy + 0.0625f);
			glVertex2f(0.0f, 0.0f);
			glEnd();

			glTranslatef(font_spacing, 0.0f, 0.0f);

			glEndList();
		}

		return new BitmapFont(first_display_list, font_height, font_spacing, texture_handle.GetObject());
	}

	void BitmapFontLoader::Unload(BitmapFont* content, ContentMetadata& meta)
	{
		content->Dispose();
		delete content;
	}
}
