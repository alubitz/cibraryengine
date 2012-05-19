#include "StdAfx.h"
#include "Cursor.h"

#include "DebugLog.h"

#include "Vector.h"
#include "Texture2D.h"

namespace CibraryEngine
{
	/*
	 * Cursor implementation; private methods and properties
	 */
	struct Cursor::Imp
	{
		Texture2D* texture;
		Vec2 offset;

		Imp(Texture2D* texture, Vec2 offset) :
			texture(texture),
			offset(offset)
		{
		}

		void Destroy() { }				// don't delete a texture which might be used elsewhere!
	};




	/*
	 * Cursor methods
	 */
	Cursor::Cursor(Texture2D* texture, Vec2 offset) :
		imp(new Imp(texture, offset))
	{
	}

	void Cursor::InnerDispose()
	{
		if(imp != NULL)
		{
			imp->Destroy();

			delete imp;
			imp = NULL;
		}
	}

	Texture2D* Cursor::GetImage() { return imp->texture; }
	Vec2 Cursor::GetOffset() { return imp->offset; }

	// assumes we're already in the correct view matrix, proj, etc.
	void Cursor::Draw(float x, float y)
	{
		glDisable(GL_LIGHTING);
		glDisable(GL_CULL_FACE);

		glDisable(GL_DEPTH_TEST);
		glDepthMask(GL_FALSE);

		glEnable(GL_TEXTURE_2D);

		glBindTexture(GL_TEXTURE_2D, imp->texture->GetGLName());

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();

		glTranslatef(x - imp->offset.x, y - imp->offset.y, 0);

		int w = imp->texture->width;
		int h = imp->texture->height;
		float max_x = (float)(w - 1);
		float max_y = (float)(h - 1);
		float max_u = (float)max_x / w;
		float max_v = (float)max_y / h;

		glBegin(GL_QUADS);
		glTexCoord2f	(0.0f,	0.0f);
		glVertex2f		(0.0f,	0.0f);
		glTexCoord2f	(max_u,	0.0f);
		glVertex2f		(max_x,	0.0f);
		glTexCoord2f	(max_u,	max_v);
		glVertex2f		(max_x,	max_y);
		glTexCoord2f	(0.0f,	max_v);
		glVertex2f		(0.0f,	max_y);
		glEnd();

		glPopMatrix();
	}




	/*
	 * CursorLoader methods
	 */
	CursorLoader::CursorLoader(ContentMan* man) : ContentTypeHandler<Cursor>(man) { }

	Cursor* CursorLoader::Load(ContentMetadata& what)
	{
		Cache<Texture2D>* cache = man->GetCache<Texture2D>();
		ContentHandle<Texture2D> texture(cache->GetHandle(what.name));
		cache->ForceLoad(texture);
		if(texture.GetObject() == NULL)
		{
			Debug(((stringstream&)(stringstream() << "Couldn't load texture for cursor \"" << what.name << "\"" << endl)).str());
			return NULL;
		}
		else
			return new Cursor(texture.GetObject(), Vec2());
	}

	void CursorLoader::Unload(Cursor* content, ContentMetadata& what)
	{
		content->Dispose();
		delete content;
	}
};