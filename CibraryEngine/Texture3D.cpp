#include "Texture3D.h"
#include "Texture2D.h"

namespace CibraryEngine
{
	/*
	 * Texture3D methods
	 */
	Texture3D::Texture3D(int depth, int width, int height, unsigned char* byte_data, bool mipmaps, bool clamp) :
		Texture(),
		depth(depth),
		width(width),
		height(height),
		byte_data(byte_data),
		mipmaps(mipmaps),
		clamp(clamp)
	{
	}

	void Texture3D::InnerDispose()
	{
		glDeleteTextures(1, &gl_name);
		gl_name = 0;

		if(byte_data != NULL)
		{
			delete[] byte_data;
			byte_data = NULL;
		}
	}

	unsigned int Texture3D::GetGLName()
	{
		if(gl_name == 0)
		{
			glDisable(GL_TEXTURE_2D);

			bool were_textures_enabled = glIsEnabled(GL_TEXTURE_3D);
			glEnable(GL_TEXTURE_3D);

			glGenTextures(1, &gl_name);

			glBindTexture(GL_TEXTURE_3D, gl_name);

			glPixelStorei(GL_UNPACK_ALIGNMENT, 4);

			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, mipmaps ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_3D, GL_GENERATE_MIPMAP, mipmaps ? 1 : 0);
			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT);
			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT);
			glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, clamp ? GL_CLAMP_TO_EDGE : GL_REPEAT);

			glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, width, height, depth, 0, GL_RGBA, GL_UNSIGNED_BYTE, byte_data);

			if (!were_textures_enabled)
				glDisable(GL_TEXTURE_3D);
		}
		return gl_name;
	}

	Texture3D* Texture3D::FromSpriteSheetAnimation(Texture2D* sheet, int col_size, int row_size, int cols, int rows, int frame_count)
	{
		unsigned char* data = new unsigned char[frame_count * col_size * row_size * 4];		// gets deleted by Texture3D::InnerDispose

		int data_index = 0;
		for (int level = 0; level < frame_count; level++)
		{
			int row = level / cols;
			int col = level % cols;
			int from_x = col * col_size, to_x = from_x + col_size;
			int from_y = row * row_size, to_y = from_y + row_size;

			for (int x = from_x; x < to_x; x++)
				for (int y = from_y; y < to_y; y++)
				{
					//unsigned char* rgba = &sheet->byte_data[(y * sheet->width + x) * 4];
					for (int chan = 0; chan < 4; chan++)
						data[data_index++] = sheet->byte_data[(y * sheet->width + x) * 4 + chan];
				}
		}

		return new Texture3D(frame_count, col_size, row_size, data, false, true);
	}
}
