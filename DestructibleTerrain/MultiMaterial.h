#pragma once
#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	struct MultiMaterial
	{
		unsigned char types[4], weights[4];

		MultiMaterial();

		void Clear();
		unsigned char GetMaterialAmount(unsigned char mat);
		void SetMaterialAmount(unsigned char mat, unsigned char amount);
		
		Vec4 GetColor();
	};
}
