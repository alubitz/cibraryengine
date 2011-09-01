#pragma once

#include "StdAfx.h"

#include "Disposable.h"

#include "Content.h"

namespace CibraryEngine
{
	using namespace std;

	class Texture2D;
	struct Vec2;

	class Cursor : public Disposable
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			string cursor_name;

			Cursor(Texture2D* texture, Vec2 offset);

			Texture2D* GetImage();
			Vec2 GetOffset();

			void Draw(float x, float y);
	};

	struct CursorLoader : public ContentTypeHandler<Cursor>
	{
		CursorLoader(ContentMan* man);

		Cursor* Load(ContentMetadata& what);
		void Unload(Cursor* content, ContentMetadata& what);
	};
}
