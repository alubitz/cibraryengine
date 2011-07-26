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
		int max_x = w - 1;
		int max_y = h - 1;
		float max_u = (float)max_x / w;
		float max_v = (float)max_y / h;

		glBegin(GL_QUADS);
		glTexCoord2f	(0,		0);
		glVertex2f		(0,		0);
		glTexCoord2f	(max_u,	0);
		glVertex2f		(max_x,	0);
		glTexCoord2f	(max_u,	max_v);
		glVertex2f		(max_x,	max_y);
		glTexCoord2f	(0,		max_v);
		glVertex2f		(0,		max_y);
		glEnd();

		glPopMatrix();
	}




	/*
	 * CursorLoader methods
	 */
	CursorLoader::CursorLoader(ContentMan* man) : ContentTypeHandler<Cursor>(man) { }

	Cursor* CursorLoader::Load(ContentMetadata& what)
	{
		ContentHandle<Texture2D> texture = man->GetHandle<Texture2D>(what.name);
		man->ForceLoad<Texture2D>(texture);
		if(texture.GetObject() == NULL)
			return NULL;
		else
			return new Cursor(texture.GetObject(), Vec2());
	}

	void CursorLoader::Unload(Cursor* content, ContentMetadata& what)
	{
		content->Dispose();
		delete content;
	}
};