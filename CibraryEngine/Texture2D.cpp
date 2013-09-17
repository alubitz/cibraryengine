#include "StdAfx.h"
#include "Texture2D.h"
#include "ImageIO.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * Texture2D methods
	 */
	Texture2D::Texture2D(int width, int height, unsigned char* byte_data, bool mipmaps, bool clamp) :
		Texture(),
		width(width),
		height(height),
		byte_data(byte_data),
		mipmaps(mipmaps),
		clamp(clamp)
	{
	}

	Texture2D::Texture2D(unsigned int gl_name_) :
		Texture(),
		byte_data(NULL)
	{
		gl_name = gl_name_;
	}

	void Texture2D::InnerDispose()
	{
		glDeleteTextures(1, &gl_name);
		gl_name = 0;

		if(byte_data)
		{
			delete[] byte_data;
			byte_data = NULL;
		}
	}

	unsigned int Texture2D::GetGLName()
	{
		if(gl_name == 0)
		{
			GLDEBUG();

			bool were_textures_enabled = glIsEnabled(GL_TEXTURE_2D) == GL_TRUE;
			glEnable(GL_TEXTURE_2D);

			glGenTextures(1, &gl_name);

			glBindTexture(GL_TEXTURE_2D, gl_name);

			glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, mipmaps ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, clamp ? GL_CLAMP : GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, clamp ? GL_CLAMP : GL_REPEAT);

			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, byte_data);

			if(mipmaps)
				glGenerateMipmap(GL_TEXTURE_2D);

			if(!were_textures_enabled)
				glDisable(GL_TEXTURE_2D);

			GLDEBUG();
		}
		return gl_name;
	}




	/*
	 * Texture2DLoader methods
	 */
	Texture2DLoader::Texture2DLoader(ContentMan* man) : ContentTypeHandler<Texture2D>(man), default_mipmaps(true), default_clamp(false) { }

	Texture2D* Texture2DLoader::Load(ContentMetadata& what)
	{
		string filename = "Files/Textures/" + what.name + ".png";

		int width, height;
		vector<unsigned char> image;

		if(unsigned int result = ImageIO::LoadPNG(filename, image, width, height))
			return NULL;
		else if(width == 0 || height == 0)
		{
			Debug(((stringstream&)(stringstream() << "Image loaded from file \"" << filename << "\" has dimensions " << width << " x " << height << "!" << endl)).str());
			return NULL;
		}

		int dim = image.size();

		unsigned char* byte_data = new unsigned char[dim];
		memcpy(byte_data, &image[0], dim);

		return new Texture2D(width, height, byte_data, default_mipmaps, default_clamp);
	}

	void Texture2DLoader::Unload(Texture2D* content, ContentMetadata& meta)
	{
		content->Dispose();
		delete content;
	}
}
