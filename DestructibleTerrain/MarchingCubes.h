#pragma once

#include "StdAfx.h"

namespace DestructibleTerrain
{
	using namespace std;
	using namespace CibraryEngine;

	struct MarchingCubes
	{
		static const int edge_table[256];
		static const int tri_table[256][16];

		template <class I, class O> static void Polygonize(Vec3 base_xyz, I grid[8], float isolevel, vector<O>& target);
		template <class O, class I> static O InterpolateVertex(float isolevel, I p1, I p2);
	};

	/*
	 * Due to the nature of template functions, the definitions have to be in the same
	 * compilation unit as the declarations!
     *
	 * At least I didn't have to put the tables in the same file...
	 *
	 */
	template <class I, class O> static void MarchingCubes::Polygonize(Vec3 base_xyz, I grid[8], float isolevel, vector<O>& target)
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

		if (edge_table[classification] == 0)
			return;

		O vert_list[12];

		/* Find the vertices where the surface intersects the cube */
		if (edge_table[classification] & 1)
			vert_list[0] = InterpolateVertex<O>(isolevel, grid[0], grid[1]);
		if (edge_table[classification] & 2)
			vert_list[1] = InterpolateVertex<O>(isolevel, grid[1], grid[2]);
		if (edge_table[classification] & 4)
			vert_list[2] = InterpolateVertex<O>(isolevel, grid[2], grid[3]);
		if (edge_table[classification] & 8)
			vert_list[3] = InterpolateVertex<O>(isolevel, grid[3], grid[0]);
		if (edge_table[classification] & 16)
			vert_list[4] = InterpolateVertex<O>(isolevel, grid[4], grid[5]);
		if (edge_table[classification] & 32)
			vert_list[5] = InterpolateVertex<O>(isolevel, grid[5], grid[6]);
		if (edge_table[classification] & 64)
			vert_list[6] = InterpolateVertex<O>(isolevel, grid[6], grid[7]);
		if (edge_table[classification] & 128)
			vert_list[7] = InterpolateVertex<O>(isolevel, grid[7], grid[4]);
		if (edge_table[classification] & 256)
			vert_list[8] = InterpolateVertex<O>(isolevel, grid[0], grid[4]);
		if (edge_table[classification] & 512)
			vert_list[9] = InterpolateVertex<O>(isolevel, grid[1], grid[5]);
		if (edge_table[classification] & 1024)
			vert_list[10] = InterpolateVertex<O>(isolevel, grid[2], grid[6]);
		if (edge_table[classification] & 2048)
			vert_list[11] = InterpolateVertex<O>(isolevel, grid[3], grid[7]);

		/* Add the triangles to the vertex buffer */
		for(int i = 0; tri_table[classification][i] != -1; i += 3)
		{	
			target.push_back(vert_list[tri_table[classification][i    ]]);
			target.push_back(vert_list[tri_table[classification][i + 1]]);
			target.push_back(vert_list[tri_table[classification][i + 2]]);
		}
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
