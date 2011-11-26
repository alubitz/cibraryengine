#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct ImageIO
	{
		/**
		 * Function to load a PNG image.
		 * @return An int error code, or 0 if all clear
		 */
		static unsigned int LoadPNG(string filename, std::vector<unsigned char>& image, int& w, int& h);

		/**
		 * Function to save a TGA image.
		 * @return An int error code, or 0 if all clear
		 */
		static unsigned int SaveTGA(string filename, std::vector<unsigned char>& image, int w, int h);
	};
};
