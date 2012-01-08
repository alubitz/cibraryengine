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

		for(int x = 0; x < dim_x; ++x)
			for(int y = 0; y < dim_y; ++y)
				for(int z = 0; z < dim_z; ++z)
					chunks.push_back(new TerrainChunk(material, this, x, y, z));
	}

	VoxelTerrain::~VoxelTerrain() { Dispose(); }

	void VoxelTerrain::InnerDispose()
	{
		for(vector<TerrainChunk*>::iterator iter = chunks.begin(); iter != chunks.end(); ++iter)
		{
			TerrainChunk* chunk = *iter;
			if(chunk != NULL)
				delete chunk;
		}

		chunks.clear();

		Disposable::InnerDispose();
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

	void VoxelTerrain::ModifySphere(Vec3 center, float inner_radius, float outer_radius, TerrainAction& action)
	{
		for(vector<TerrainChunk*>::iterator iter = chunks.begin(); iter != chunks.end(); ++iter)
			(*iter)->ModifySphere(center, inner_radius, outer_radius, action);
	}




	/*
	 * VoxelTerrainLoader methods
	 */
	VoxelTerrainLoader::VoxelTerrainLoader(ContentMan* man, VoxelMaterial* default_material) : ContentTypeHandler<VoxelTerrain>(man), default_material(default_material) { }

	VoxelTerrain* VoxelTerrainLoader::Load(ContentMetadata& what)
	{
		string filename = "Files/Levels/" + what.name + ".vvv";

		VoxelTerrain* model = NULL;
		unsigned int vvv_result = VoxelTerrainLoader::LoadVVV(model, default_material, filename);

		if(vvv_result != 0)
		{
			stringstream vvv_msg;
			vvv_msg << "LoadVVV (" << what.name << ") returned with status " << vvv_result << endl;
			Debug(vvv_msg.str());
		}

		return model;
	}

	void VoxelTerrainLoader::Unload(VoxelTerrain* content, ContentMetadata& meta)
	{
		content->Dispose();
		delete content;
	}

	unsigned int VoxelTerrainLoader::SaveVVV(VoxelTerrain* terrain, string filename)
	{
		if(terrain == NULL)
			return 1;

		terrain->Solidify();			// lets us skip whole chunks if they are empty

		BinaryChunk whole;
		whole.SetName("VX_TRN__");

		stringstream ss;

		WriteUInt32(terrain->dim[0], ss);
		WriteUInt32(terrain->dim[1], ss);
		WriteUInt32(terrain->dim[2], ss);

		for(int x = 0; x < terrain->dim[0]; ++x)
			for(int y = 0; y < terrain->dim[1]; ++y)
				for(int z = 0; z < terrain->dim[2]; ++z)
					if(unsigned int chunk_write_error = terrain->Chunk(x, y, z)->Write(ss))
						return chunk_write_error + 2;

		whole.data = ss.str();

		ofstream file(filename.c_str(), ios::out | ios::binary);
		if(!file)
			return 2;

		whole.Write(file);

		return 0;
	}

	unsigned int VoxelTerrainLoader::LoadVVV(VoxelTerrain*& terrain, VoxelMaterial* material, string filename)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		BinaryChunk whole;
		whole.Read(file);

		if(whole.GetName() != "VX_TRN__")
			return 2;

		istringstream ss(whole.data);

		unsigned int dim_x = ReadUInt32(ss);
		unsigned int dim_y = ReadUInt32(ss);
		unsigned int dim_z = ReadUInt32(ss);

		VoxelTerrain* temp = new VoxelTerrain(material, dim_x, dim_y, dim_z);

		for(unsigned int x = 0; x < dim_x; ++x)
			for(unsigned int y = 0; y < dim_y; ++y)
				for(unsigned int z = 0; z < dim_x; ++z)
					if(unsigned int chunk_read_error = temp->Chunk(x, y, z)->Read(ss))
					{
						delete temp;
						terrain = NULL;

						return chunk_read_error + 2;
					}

		terrain = temp;
		return 0;
	}

	VoxelTerrain* VoxelTerrainLoader::GenerateTerrain(VoxelMaterial* material, int dim_x, int dim_y, int dim_z)
	{
		// Generate stone using Perlin noise
		struct PNFunc
		{
			PerlinNoise n1;

			PNFunc() : n1(256) { }
			float operator() (Vec3 vec) { return n1.Sample(vec) + n1.Sample(vec * 2) / 4 + n1.Sample(vec * 4) / 8 + n1.Sample(vec * 8) / 16; }
		} n;

		VoxelTerrain* result = new VoxelTerrain(material, dim_x, dim_y, dim_z);

		vector<TerrainChunk*>::iterator iter = result->chunks.begin();

		const int center_y = dim_y / 2;				// center point for terrain elevation

		for(int x = 0; x < dim_x; ++x)
			for(int y = 0; y < dim_y; ++y)
				for(int z = 0; z < dim_z; ++z)
				{
					TerrainChunk* chunk = *(iter++);

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
							result.material.SetMaterialAmount(1, 255);

							return result;
						}
					} pop(n, x, y - center_y, z);

					chunk->PopulateValues(pop);
				}

		result->Solidify();

		return result;
	}
}
