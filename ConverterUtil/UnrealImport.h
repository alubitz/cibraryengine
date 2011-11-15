#pragma once

#include "StdAfx.h"

namespace ConverterUtil
{
	using namespace std;
	using namespace CibraryEngine;

	int LoadPSK(string filename, SkinnedModel* model, float scale);

	int LoadPSA(string filename, vector<KeyframeAnimation>& animations, float scale);
}
