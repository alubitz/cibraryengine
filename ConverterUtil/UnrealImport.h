#pragma once

#include "StdAfx.h"

namespace ConverterUtil
{
	using namespace std;
	using namespace CibraryEngine;

	int LoadPSK(const string& filename, SkinnedModel* model, float scale);

	int LoadPSA(const string& filename, vector<KeyframeAnimation>& animations, float scale);
}
