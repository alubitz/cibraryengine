#include "StdAfx.h"
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
		if(!gl_name)
		{
			GLDEBUG();

			glGenTextures(1, &gl_name);										GLDEBUG();

			SetBuffer(buffer);
		}
		return gl_name;
	}

	void TextureBuffer::SetBuffer(VertexBuffer* buffer_)
	{
		buffer = buffer_;

		glBindTexture(GL_TEXTURE_BUFFER, gl_name);						GLDEBUG();
		glTexBufferEXT(GL_TEXTURE_BUFFER, format, buffer->GetVBO());	GLDEBUG();
		glBindTexture(GL_TEXTURE_BUFFER, 0);							GLDEBUG();
	}
}
