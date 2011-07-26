#pragma once

#include "StdAfx.h"

#include "UnrealAnimDataStructs.h"

namespace CibraryEngine
{
	using namespace std;

	class KeyframeAnimation;
	class SkinnedModel;

	int LoadPSK(string filename, SkinnedModel* model, float scale);

	int LoadPSA(string filename, vector<KeyframeAnimation>& animations, float scale);
}
