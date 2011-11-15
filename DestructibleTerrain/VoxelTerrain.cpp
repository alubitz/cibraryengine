#include "StdAfx.h"

#include "VoxelTerrain.h"
#include "TerrainLeaf.h"
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
		scale(Mat4::UniformScale(1.0f / 4.0f)),
		xform(Mat4::Translation(float(-dim_x * 4), float(-dim_y * 4), float(-dim_z * 4)))
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

						TerrainLeaf operator ()(int x, int y, int z)
						{
							Vec3 pos = Vec3(float(ox * TerrainChunk::ChunkSize + x), float(oy * TerrainChunk::ChunkSize + y), float(oz * TerrainChunk::ChunkSize + z)) / 16.0f;

							TerrainLeaf result;
							result.solidity = (unsigned char)max(0.0f, min(255.0f, 128.0f + 255.0f * ((*n)(pos) + 1.0f - pos.y)));
							result.SetMaterialAmount(1, 255);

							return result;
						}
					} pop(n, x, y, z);

					chunk->PopulateValues(pop);
					chunks.push_back(chunk);
				}
	}

	VoxelTerrain::~VoxelTerrain() 
	{
		for(vector<TerrainChunk*>::iterator iter = chunks.begin(); iter != chunks.end(); iter++)
		{
			TerrainChunk* chunk = *iter;
			if(chunk != NULL)
				delete chunk;
		}

		chunks.clear();
	}

	void VoxelTerrain::GetDimensions(int& x, int& y, int& z)
	{
		x = dim[0];
		y = dim[1];
		z = dim[2];
	}

	TerrainChunk*& VoxelTerrain::Chunk(int x, int y, int z) { return chunks[x * x_span + y * dim[2] + z]; }

	void VoxelTerrain::Vis(SceneRenderer* renderer)
	{ 
		for(vector<TerrainChunk*>::iterator iter = chunks.begin(); iter != chunks.end(); iter++)
		{
			TerrainChunk* chunk = *iter;
			if(chunk != NULL)
				chunk->Vis(renderer, scale * xform);
		}
	}

	void VoxelTerrain::Explode()
	{
		int x = Random3D::RandInt(dim[0] * TerrainChunk::ChunkSize), z = Random3D::RandInt(dim[2] * TerrainChunk::ChunkSize);
		int y;

		for(y = dim[1] * TerrainChunk::ChunkSize - 1; y >= 0; y--)
			if(Chunk(x / TerrainChunk::ChunkSize, y / TerrainChunk::ChunkSize, z / TerrainChunk::ChunkSize)->Element(x % TerrainChunk::ChunkSize, y % TerrainChunk::ChunkSize, z % TerrainChunk::ChunkSize).IsSolid())
				break;

		Vec3 blast_center = Vec3(float(x), float(y), float(z));
		float blast_force = Random3D::Rand(4, 10) * 2;

		for(vector<TerrainChunk*>::iterator iter = chunks.begin(); iter != chunks.end(); iter++)
			(*iter)->Explode(blast_center, blast_force);
	}
}
