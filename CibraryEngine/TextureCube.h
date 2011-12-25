#pragma once

#include "StdAfx.h"
#include "Content.h"
#include "Texture.h"
#include "Texture2D.h"

namespace CibraryEngine
{
	using namespace std;

	/** Class representing a cube map */
	class TextureCube : public Texture
	{
		protected:

			void InnerDispose();

		public:

			/** The size of each face (the faces are square) */
			int size;

			/** The byte data for each of the six faces of the cube */
			unsigned char** byte_data;

			bool mipmaps;

			/** Initializes a TextureCube handle */
			TextureCube(int size, unsigned char** byte_data, bool mipmaps);

			unsigned int GetGLName();
	};

	/** ContentLoader for TextureCube content */
	struct TextureCubeLoader : public ContentTypeHandler<TextureCube>
	{
		bool default_mipmaps;

		TextureCubeLoader(ContentMan* man) : ContentTypeHandler<TextureCube>(man), default_mipmaps(true) { }

		TextureCube* Load(ContentMetadata& what)
		{
			Texture2D* anim = man->GetCache<Texture2D>()->Load("../Cubemaps/" + what.name);
			if(anim == NULL)
				return NULL;

			int size = anim->height / 2;

			unsigned char** byte_data = new unsigned char* [6];

			for (int i = 0; i < 6; ++i)
			{
				byte_data[i] = new unsigned char[size * size * 4];
				int x_initial = (i % 4) * size;
				int y_initial = (i / 4) * size;
				for (int x = 0; x < size; ++x)
					for (int y = 0; y < size; ++y)
					{
						int source = (y * size + x) * 4;
						int dest = ((y_initial + y) * anim->width + (x_initial + x)) * 4;

						for (int j = 0; j < 4; ++j)
							byte_data[i][source + j] = anim->byte_data[dest + j];
					}
			}

			return new TextureCube(size, byte_data, default_mipmaps);
		}
	};
}
