#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct VertexBoneWeightInfo
	{
		int point_index;

		vector<int> bones;
		vector<float> weights;

		VertexBoneWeightInfo() : point_index(-1), bones(), weights() { }

		void AddValue(int bone, float weight);

		void GetByteValues(unsigned char* indices, unsigned char* weights);
	};
}
