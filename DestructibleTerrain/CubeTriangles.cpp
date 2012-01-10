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
		MultiMaterial material;

		GridStruct(TerrainChunk* t, int x, int y, int z) : position(float(x), float(y), float(z)) 
		{
			TerrainNode* node_ptr = t->GetNodeRelative(x, y, z);
			assert(node_ptr != NULL);

			TerrainNode& node = *node_ptr;

			value = node.GetScalarValue();
			position = Vec3(float(x), float(y), float(z));
			material = node.material;
		}

		static TerrainVertex Convert(GridStruct& a) { return TerrainVertex(a.position, a.material); }
		static TerrainVertex Lerp(GridStruct& a, GridStruct& b, float mu) {  return TerrainVertex(a.position * (1.0f - mu) + b.position * mu, MultiMaterial::Lerp(a.material, b.material, mu)); }
	};




	/*
	 * CubeTriangles methods
	 */
	void CubeTriangles::Invalidate()
	{
		num_vertices = -1;

		if(cache != NULL)
		{
			delete cache;
			cache = NULL;
		}

		chunk->InvalidateVBO();
	}

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

			assert(cache == NULL);

			CacheData temp_cache;

			MarchingCubes::Polygonize(Vec3(float(x), float(y), float(z)), grid, 0.0f, &temp_cache.verts[0], &temp_cache.indices[0]);

			int i;
			for(i = 0; i < 16; ++i)
				if(temp_cache.indices[i] == -1)
					break;
			num_vertices = i;

			if(num_vertices > 0)
				cache = new CacheData(temp_cache);
		}
	}
}
