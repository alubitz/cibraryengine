#include "StdAfx.h"

#include "ImageIO.h"
#include "Serialize.h"

#include "SOIL.h"

namespace CibraryEngine
{
	/*
	 * ImageIO methods
	 */
	unsigned int ImageIO::LoadPNG(string filename, std::vector<unsigned char>& image, int& w, int& h)
	{
		int channels;
		unsigned char* byte_data = SOIL_load_image(filename.c_str(), &w, &h, &channels, SOIL_LOAD_RGBA);

		if(!byte_data)
			return 1;

		unsigned int size = w * h * 4;

		vector<unsigned char> result = vector<unsigned char>();
		result.resize(size);

		memcpy(&result[0], byte_data, size);

		image = result;			// save the actual copy for last
		return 0;
	}
	unsigned int ImageIO::SaveTGA(string filename, std::vector<unsigned char>& image, int w, int h)
	{
		if(SOIL_save_image(filename.c_str(), SOIL_SAVE_TYPE_TGA, w, h, 4, &image[0]))
			return 0;
		else
			return 1;
	}
}
