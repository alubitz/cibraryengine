#include "UnclampedVertexBoneWeightInfo.h"

namespace CibraryEngine
{
	void VertexBoneWeightInfo::AddValue(int bone, float weight)
	{
		int size = bones.size();

		for(int i = 0; i < size; i++)
			if(bones[i] == bone)
			{
				weights[i] += weight;
				return;
			}

		// insert the new bone
		bones.push_back(bone);
		weights.push_back(weight);
	}

	void VertexBoneWeightInfo::GetByteValues(unsigned char* indices_out, unsigned char* weights_out)
	{
		unsigned int size = bones.size();
		if(size > 4)
			size = 4;

		// get the topmost several bone indices/weights (count = size)
		for(unsigned k = 0; k < size; k++)
		{
			unsigned int best = k;
			for(unsigned int j = k + 1; j < bones.size(); j++)
			{
				if(weights[j] > weights[best])
					best = j;
			}
			if(best != k)
			{
				int temp_bone = bones[k];
				float temp_weight = weights[k];
				bones[k] = bones[best];
				weights[k] = weights[best];
				bones[best] = temp_bone;
				weights[best] = temp_weight;
			}
		}

		float total = 0;
		for(unsigned int k = 0; k < size; k++)
			total += weights[k];

		for(unsigned int k = 0; k < size; k++)
		{
			unsigned char b = (unsigned char)(bones[k]);
			unsigned char w = (unsigned char)(weights[k] * 255.0f / total);

			indices_out[k] = b;
			weights_out[k] = w;
		}
		for(unsigned int k = size; k < 4; k++)
		{
			indices_out[k] = 0;
			weights_out[k] = 0;
		}
		if(size == 0)
			weights_out[0] = 255;
	}
}
