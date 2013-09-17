#include "StdAfx.h"
#include "Texture1D.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * Texture1D methods
	 */
	Texture1D::Texture1D(unsigned int size, unsigned char* byte_data) :
		Texture(),
		byte_data(byte_data),
		size(size)
	{
	}

	void Texture1D::InnerDispose()
	{
		glDeleteTextures(1, &gl_name);
		gl_name = 0;

		if(byte_data)
		{
			delete byte_data;
			byte_data = NULL;
		}
	}

	unsigned int Texture1D::GetGLName()
	{
		if(gl_name == 0)
		{
			GLDEBUG();

			glGenTextures(1, &gl_name);
			glEnable(GL_TEXTURE_1D);
			glBindTexture(GL_TEXTURE_1D, gl_name);

			glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

			glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
			glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

			glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA8, size, 0, GL_RGBA, GL_UNSIGNED_BYTE, byte_data);
			glDisable(GL_TEXTURE_1D);

			GLDEBUG();
		}
		return gl_name;
	}

	void Texture1D::UpdateTextureData()
	{
		glEnable(GL_TEXTURE_1D);
		glBindTexture(GL_TEXTURE_1D, gl_name);

		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		glTexImage1D(GL_TEXTURE_1D, 0, GL_RGBA8, size, 0, GL_RGBA, GL_UNSIGNED_BYTE, byte_data);

		glDisable(GL_TEXTURE_1D);
	}
}
