#include "StdAfx.h"
#include "CubeTriangles.h"

#include "TerrainNode.h"
#include "TerrainChunk.h"
#include "MarchingCubes.h"

namespace DestructibleTerrain
{
	/*
	 * CubeTriangles methods
	 */
	CubeTriangles::CubeTriangles(TerrainChunk* chunk, int x, int y, int z) :
		chunk(chunk),
		x(x),
		y(y),
		z(z),
		valid(false)
	{
	}

	void CubeTriangles::Invalidate()
	{
		valid = false;
		chunk->InvalidateVBO();
	}





	/*
	 * Struct used for input to MarchingCubes::Polygonize in CubeTriangles::AppendVertexData, below
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
			color = Vec4();

			if(node.IsSolid())
			{
				for(int i = 0; i < 4; i++)
				{
					switch(node.types[i])
					{
					case 1:
					
						// stone color
						color += Vec4(0.5f, 0.5f, 0.5f, 1.0f) * node.weights[i];
						break;

					case 2:
					
						// dirt (sand) color
						color += Vec4(0.85f, 0.75f, 0.55f, 1.0f) * node.weights[i];
						break;
					}
				}
			}

			value = node.GetScalarValue();
			position = Vec3(float(x), float(y), float(z));

			color *= color.w == 0 ? 1.0f : 1.0f / color.w;
		}

		operator TerrainVertex() { return TerrainVertex(position, color); }
	};




	/*
	 * More CubeTriangles methods
	 */
	void CubeTriangles::AppendVertexData(vector<TerrainVertex>& target)
	{
		if(!valid)
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

			valid = true;
		}

		for(int i = 0; indices[i] != -1; i += 3)
		{
			target.push_back(verts[indices[i    ]]);
			target.push_back(verts[indices[i + 1]]);
			target.push_back(verts[indices[i + 2]]);
		}
	}
}
