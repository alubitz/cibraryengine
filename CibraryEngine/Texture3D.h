#pragma once

#include "StdAfx.h"
#include "Texture.h"

namespace CibraryEngine
{
	class Texture2D;

	/** Class representing a 3-dimensional texture */
	class Texture3D : public Texture
	{
		protected:

			void InnerDispose();

		public:

			/** The depth-direction size of this texture */
			int depth;
			/** The horizontal size of this texture */
			int width;
			/** The vertical size of this texture */
			int height;
			/** The byte data for this texture */
			unsigned char* byte_data;

			bool mipmaps;
			bool clamp;

			/** Creates a 3D texture with the specified properties */
			Texture3D(int depth, int width, int height, unsigned char* byte_data, bool mipmaps, bool clamp);

			unsigned int GetGLName();

			static Texture3D* FromSpriteSheetAnimation(Texture2D* sheet, int col_size, int row_size, int cols, int rows, int frame_count);
	};
}
