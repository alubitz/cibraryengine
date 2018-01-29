#include "StdAfx.h"

#include "DebugLog.h"
#include "TextureBuffer.h"

#include "VertexBuffer.h"

namespace CibraryEngine
{
	/*
	 * TextureBuffer methods
	 */
	TextureBuffer::TextureBuffer(VertexBuffer* buffer, GLenum format) : Texture(), buffer(buffer), format(format) { }

	void TextureBuffer::InnerDispose()
	{
		glDeleteTextures(1, &gl_name);
		gl_name = 0;
	}

	unsigned int TextureBuffer::GetGLName()
	{
		if(gl_name == 0)
		{
			GLDEBUG();

			glGenTextures(1, &gl_name);						GLDEBUG();

			glBindTexture(GL_TEXTURE_BUFFER, gl_name);		GLDEBUG();
			glBindTexture(GL_TEXTURE_BUFFER, 0);			GLDEBUG();
		}
		return gl_name;
	}
}
