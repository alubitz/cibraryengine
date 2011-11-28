#include "StdAfx.h"

#include "VoxelTerrain.h"
#include "TerrainNode.h"
#include "TerrainChunk.h"

#include "VoxelMaterial.h"
#include "PerlinNoise.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	/*
	 * VoxelTerrain methods
	 */
	VoxelTerrain::VoxelTerrain(VoxelMaterial* material, int dim_x, int dim_y, int dim_z) :
		chunks(),
		scale(Mat4::UniformScale(0.25f)),
		xform(Mat4::Translation(float(-0.5f * dim_x * TerrainChunk::ChunkSize), float(-0.5f * dim_y * TerrainChunk::ChunkSize), float(-0.5f * dim_z * TerrainChunk::ChunkSize)))
	{
		dim[0] = dim_x;
		dim[1] = dim_y;
		dim[2] = dim_z;
		x_span = dim_y * dim_z;
		
		// Generate stone using Perlin noise
		struct PNFunc
		{
			PerlinNoise n1;

			PNFunc() : n1(256) { }
			float operator() (Vec3 vec) { return n1.Sample(vec) + n1.Sample(vec * 2) / 4 + n1.Sample(vec * 4) / 8 + n1.Sample(vec * 8) / 16; }
		} n;

		for(int x = 0; x < dim[0]; x++)
			for(int y = 0; y < dim[1]; y++)
				for(int z = 0; z < dim[2]; z++)
				{
					TerrainChunk* chunk = new TerrainChunk(material, this, x, y, z);

					struct Populator
					{
						PNFunc* n;
						int ox, oy, oz;

						Populator(PNFunc& func, int ox, int oy, int oz) : n(&func), ox(ox), oy(oy), oz(oz) { }

						TerrainNode operator ()(int x, int y, int z)
						{
							Vec3 pos = Vec3(float(ox * TerrainChunk::ChunkSize + x), float(oy * TerrainChunk::ChunkSize + y), float(oz * TerrainChunk::ChunkSize + z)) / 32.0f;

							TerrainNode result;
							result.solidity = (unsigned char)max(0.0f, min(255.0f, 128.0f + 255.0f * ((*n)(pos) - pos.y)));
							result.SetMaterialAmount(1, 255);

							return result;
						}
					} pop(n, x, y - dim[1] / 2, z);

					chunk->PopulateValues(pop);
					chunks.push_back(chunk);
				}

		Solidify();
	}

	VoxelTerrain::~VoxelTerrain() 
	{
		for(vector<TerrainChunk*>::iterator iter = chunks.begin(); iter != chunks.end(); ++iter)
		{
			TerrainChunk* chunk = *iter;
			if(chunk != NULL)
				delete chunk;
		}

		chunks.clear();
	}

	TerrainChunk* VoxelTerrain::Chunk(int x, int y, int z) 
	{
		if(x < 0 || y < 0 || z < 0 || x >= dim[0] || y >= dim[1] || z >= dim[2])
			return NULL;
		else
			return chunks[x * x_span + y * dim[2] + z]; 
	}

	bool VoxelTerrain::PosToNode(Vec3 pos, TerrainChunk*& chunk, int& x, int& y, int& z)
	{
		int gx = (int)floor(pos.x), gy = (int)floor(pos.y), gz = (int)floor(pos.z);

		if(gx < 0 || gy < 0 || gz < 0)
			return false;
		else
		{
			int cx = gx / TerrainChunk::ChunkSize, cy = gy / TerrainChunk::ChunkSize, cz = gz / TerrainChunk::ChunkSize;
			
			chunk = Chunk(cx, cy, cz);
			if(chunk == NULL)
				return false;
			else
			{
				x = gx - TerrainChunk::ChunkSize * cx;
				y = gy - TerrainChunk::ChunkSize * cy;
				z = gz - TerrainChunk::ChunkSize * cz;

				return true;
			}
		}
	}

	void VoxelTerrain::Vis(SceneRenderer* renderer)
	{ 
		for(vector<TerrainChunk*>::iterator iter = chunks.begin(); iter != chunks.end(); ++iter)
		{
			TerrainChunk* chunk = *iter;
			if(chunk != NULL)
				chunk->Vis(renderer, GetTransform());
		}
	}

	void VoxelTerrain::Solidify()
	{
		for(vector<TerrainChunk*>::iterator iter = chunks.begin(); iter != chunks.end(); ++iter)
			(*iter)->Solidify();
	}

	void VoxelTerrain::Explode(Vec3 center, float inner_radius, float outer_radius)
	{
		for(vector<TerrainChunk*>::iterator iter = chunks.begin(); iter != chunks.end(); ++iter)
			(*iter)->Explode(center, inner_radius, outer_radius);
	}
}
