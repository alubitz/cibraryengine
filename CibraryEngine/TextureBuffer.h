#pragma once

#include "StdAfx.h"
#include "Texture.h"

namespace CibraryEngine
{
	using namespace std;

	struct VertexBuffer;

	class TextureBuffer : public Texture
	{
		protected:

			void InnerDispose();

		public:

			VertexBuffer* buffer;
			GLenum format;

			TextureBuffer(VertexBuffer* buffer, GLenum format);

			unsigned int GetGLName();
	};
}
