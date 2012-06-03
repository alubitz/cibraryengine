#include "StdAfx.h"
#include "CubeTriangles.h"

#include "TerrainNode.h"
#include "TerrainChunk.h"
#include "MarchingCubes.h"

namespace CibraryEngine
{
	/*
	 * Struct used for input to MarchingCubes::Polygonize
	 */
	struct GridStruct
	{
		float value;
		Vec3 position;
		MultiMaterial material;
		TerrainChunk* owner;

		GridStruct(TerrainChunk* t, int x, int y, int z) : position(float(x), float(y), float(z)), owner(t) 
		{
			if(TerrainNode* node_ptr = t->GetNodeRelative(x, y, z))
			{
				TerrainNode& node = *node_ptr;

				value = node.GetScalarValue();
				material = node.material;
			}
			else
				value = 127.5;
		}

		static TerrainVertex Convert(GridStruct& a) { return TerrainVertex(a.position, a.material); }
		static TerrainVertex Lerp(GridStruct& a, GridStruct& b, float mu)
		{
			float a_coeff = (1.0f - mu) * max(0.0f, -a.value);
			float b_coeff = mu * max(0.0f, -b.value);
			float mat_mu = b_coeff / (a_coeff + b_coeff);

			return TerrainVertex(a.position * (1.0f - mu) + b.position * mu, MultiMaterial::Lerp(a.material, b.material, mat_mu));
		}
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

	void CubeTriangles::InvalidateNormals()
	{
		if(cache != NULL)
		{
			cache->verts[0].normal_valid = cache->verts[1].normal_valid = cache->verts[2].normal_valid = false;

			chunk->InvalidateVBO();
		}
	}

	void CubeTriangles::BuildAsNeeded()
	{
		if(num_vertices == -1)
		{
			int X = x << lod;
			int Y = y << lod;
			int Z = z << lod;
			int D = 0x1u << lod;

			GridStruct grid[] =
			{
				GridStruct(chunk, X,		Y,		Z		),	
				GridStruct(chunk, X,		Y,		Z + D	),
				GridStruct(chunk, X,		Y + D,	Z + D	),
				GridStruct(chunk, X,		Y + D,	Z		),			
				GridStruct(chunk, X + D,	Y,		Z		),	
				GridStruct(chunk, X + D,	Y,		Z + D	),
				GridStruct(chunk, X + D,	Y + D,	Z + D	),
				GridStruct(chunk, X + D,	Y + D,	Z		)
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
