#include "StdAfx.h"
#include "CubeTriangles.h"

#include "TerrainNode.h"
#include "TerrainChunk.h"
#include "MarchingCubes.h"

namespace DestructibleTerrain
{
	/*
	 * Struct used for input to MarchingCubes::Polygonize in CubeTriangles::AppendVertexData
	 */
	struct GridStruct
	{
		float value;
		Vec3 position;
		Vec4 color;

		GridStruct(TerrainChunk* t, int x, int y, int z) : position(float(x), float(y), float(z)) 
		{
			TerrainNode* node_ptr = t->GetNodeRelative(x, y, z);
			assert(node_ptr != NULL);

			TerrainNode& node = *node_ptr;

			color = node.IsSolid() ? node.GetColor() : Vec4();
			value = node.GetScalarValue();
			position = Vec3(float(x), float(y), float(z));
		}

		operator TerrainVertex() { return TerrainVertex(position, color); }
	};




	/*
	 * CubeTriangles methods
	 */
	void CubeTriangles::Invalidate() { num_vertices = -1; chunk->InvalidateVBO(); }

	void CubeTriangles::BuildAsNeeded()
	{
		if(num_vertices == -1)
		{
			GridStruct grid[] =
			{
				GridStruct(chunk, x, y, z),	
				GridStruct(chunk, x, y, z + 1),
				GridStruct(chunk, x, y + 1, z + 1),
				GridStruct(chunk, x, y + 1, z),			
				GridStruct(chunk, x + 1, y, z),	
				GridStruct(chunk, x + 1, y, z + 1),
				GridStruct(chunk, x + 1, y + 1, z + 1),
				GridStruct(chunk, x + 1, y + 1, z)
			};

			MarchingCubes::Polygonize(Vec3(float(x), float(y), float(z)), grid, 0.0f, &verts[0], &indices[0]);

			int i;
			for(i = 0; i < 16; i++)
				if(indices[i] == -1)
					break;
			num_vertices = i;
		}
	}
}
