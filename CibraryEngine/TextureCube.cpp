#include "StdAfx.h"
#include "TextureCube.h"

#include "Texture2D.h"
#include "DebugLog.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * TextureCube methods
	 */
	TextureCube::TextureCube(int size, unsigned char** byte_data, bool mipmaps) :
		Texture(),
		size(size),
		byte_data(byte_data),
		mipmaps(mipmaps)
	{
	}

	void TextureCube::InnerDispose()
	{
		glDeleteTextures(1, &gl_name);
		gl_name = 0;

		if(byte_data)
		{
			for(int i = 0; i < 6; ++i)
				delete[] byte_data[i];
			delete[] byte_data;
			byte_data = NULL;
		}
	}

	unsigned int TextureCube::GetGLName()
	{
		if(gl_name == 0)
		{
			GLDEBUG();

			glGenTextures(1, &gl_name);
			glEnable(GL_TEXTURE_CUBE_MAP);
			glBindTexture(GL_TEXTURE_CUBE_MAP, gl_name);

			for(int i = 0; i < 6; ++i)
				glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGBA8, size, size, 0, GL_RGBA, GL_UNSIGNED_BYTE, byte_data[i]);

			if(mipmaps)
			{
				glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
				glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S,		(int)GL_CLAMP_TO_EDGE);
				glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T,		(int)GL_CLAMP_TO_EDGE);
				glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R,		(int)GL_CLAMP_TO_EDGE);
				glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER,	(int)GL_LINEAR_MIPMAP_LINEAR);
			}

			glDisable(GL_TEXTURE_CUBE_MAP);

			GLDEBUG();
		}
		return gl_name;
	}
}
