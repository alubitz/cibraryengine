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

			/**
			 * Function to call when the byte_data has been changed, but the other properties (including gl name) should be kept
			 * This will in turn call glTexImage1D, so don't call it from a thread without a gl context */
			void UpdateTextureData();
	};
}
