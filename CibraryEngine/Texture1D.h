#pragma once

#include "StdAfx.h"
#include "Texture.h"

namespace CibraryEngine
{
	using namespace std;

	/** Class representing a 1-dimensional texture */
	class Texture1D : public Texture
	{
		protected:

			void InnerDispose();

		public:

			/** The byte data for this texture */
			unsigned char* byte_data;
			/** The size of this texture */
			unsigned int size;

			/** Creates a 1-dimensional texture with the specified byte data, given the size of the texture */
			Texture1D(unsigned int size, unsigned char* byte_data);

			unsigned int GetGLName();
	};
}
