#pragma once

#include "StdAfx.h"
#include "Texture.h"

namespace CibraryEngine
{
	/** Class representing a 2-dimensional texture */
	class Texture2D : public Texture
	{
		protected:

			void InnerDispose();

		public:

			/** The horizontal size of this texture */
			int width;
			/** The vertical size of this texture */
			int height;

			/** The byte data for this texture */
			unsigned char* byte_data;

			bool mipmaps;
			bool clamp;

			Texture2D(int width, int height, unsigned char* byte_data, bool mipmaps, bool clamp);

			unsigned int GetGLName();
	};




	struct Texture2DLoader : public ContentTypeHandler<Texture2D>
	{
		bool default_mipmaps;
		bool default_clamp;

		Texture2DLoader(ContentMan* man);

		Texture2D* Load(ContentMetadata& what);
		void Unload(Texture2D* content, ContentMetadata& meta);
	};
}
