#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace std;
	using namespace CibraryEngine;

	struct MarchingCubes
	{
		static const unsigned short int edge_table[256];
		static const char tri_table[256][16];

		template <class I, class O> static void Polygonize(Vec3 base_xyz, I grid[8], float isolevel, O* target, char* indices);
		template <class O, class I> static O InterpolateVertex(float isolevel, I p1, I p2);
	};

	/*
	 * Due to the nature of template functions, the definitions have to be in the same
	 * compilation unit as the declarations!
     *
	 * At least I didn't have to put the tables in the same file...
	 *
	 */
	template <class I, class O> static void MarchingCubes::Polygonize(Vec3 base_xyz, I grid[8], float isolevel, O* target, char* indices)
	{
		// tables and much of the algorithm is from http://paulbourke.net/geometry/polygonise
		int classification = 0;

		if(grid[0].value < isolevel) { classification |= 1; }
		if(grid[1].value < isolevel) { classification |= 2; }
		if(grid[2].value < isolevel) { classification |= 4; }
		if(grid[3].value < isolevel) { classification |= 8; }
		if(grid[4].value < isolevel) { classification |= 16; }
		if(grid[5].value < isolevel) { classification |= 32; }
		if(grid[6].value < isolevel) { classification |= 64; }
		if(grid[7].value < isolevel) { classification |= 128; }

		unsigned short int edges = edge_table[classification];
		if(edges == 0)
		{
			indices[0] = -1;
			return;
		}

		// find the vertices where the surface intersects the cube
		if(edges & 1)
			target[0] = InterpolateVertex<O>(isolevel, grid[0], grid[1]);
		if(edges & 2)
			target[1] = InterpolateVertex<O>(isolevel, grid[1], grid[2]);
		if(edges & 4)
			target[2] = InterpolateVertex<O>(isolevel, grid[2], grid[3]);
		if(edges & 8)
			target[3] = InterpolateVertex<O>(isolevel, grid[3], grid[0]);
		if(edges & 16)
			target[4] = InterpolateVertex<O>(isolevel, grid[4], grid[5]);
		if(edges & 32)
			target[5] = InterpolateVertex<O>(isolevel, grid[5], grid[6]);
		if(edges & 64)
			target[6] = InterpolateVertex<O>(isolevel, grid[6], grid[7]);
		if(edges & 128)
			target[7] = InterpolateVertex<O>(isolevel, grid[7], grid[4]);
		if(edges & 256)
			target[8] = InterpolateVertex<O>(isolevel, grid[0], grid[4]);
		if(edges & 512)
			target[9] = InterpolateVertex<O>(isolevel, grid[1], grid[5]);
		if(edges & 1024)
			target[10] = InterpolateVertex<O>(isolevel, grid[2], grid[6]);
		if(edges & 2048)
			target[11] = InterpolateVertex<O>(isolevel, grid[3], grid[7]);

		const char* vert_indices = &tri_table[classification][0];
		for(int i = 0; i < 16; ++i)
			if((indices[i] = vert_indices[i]) == -1)
				break;
	}

	template <class O, class I> static O MarchingCubes::InterpolateVertex(float isolevel, I p1, I p2)
	{
		float v1 = p1.value, v2 = p2.value;

		if (fabs(isolevel - v1) < 0.00001f)
			return(O(p1));
		if (fabs(isolevel - v2) < 0.00001f)
			return(O(p2));
		if (fabs(v1 - v2) < 0.00001f)
			return(O(p1));

		float mu = (isolevel - v1) / (v2 - v1);
		return O(p1) * (1.0f - mu) + O(p2) * mu;
	}
}
