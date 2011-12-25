#include "StdAfx.h"
#include "TerrainChunk.h"

#include "TerrainNode.h"
#include "CubeTriangles.h"
#include "TerrainVertex.h"

#include "VoxelMaterial.h"

namespace DestructibleTerrain
{
	/*
	 * TerrainChunk methods
	 */
	TerrainChunk::TerrainChunk(VoxelMaterial* material, VoxelTerrain* owner, int x, int y, int z) :
		node_data(),
		tri_data(),
		chunk_x(x),
		chunk_y(y),
		chunk_z(z),
		material(material),
		model(NULL),
		vbo_valid(false),
		owner(owner)
	{
		xform = Mat4::Translation(float(x * ChunkSize), float(y * ChunkSize), float(z * ChunkSize));

		for(int i = 0; i < ChunkSize * ChunkSize * ChunkSize; ++i)
			node_data.push_back(TerrainNode());

		for(int x = 0; x < ChunkSize; ++x)
			for(int y = 0; y < ChunkSize; ++y)
				for(int z = 0; z < ChunkSize; ++z)
					tri_data.push_back(CubeTriangles(this, x, y, z));
	}

	TerrainChunk::~TerrainChunk() { InvalidateVBO(); }

	TerrainNode* TerrainChunk::GetNode(int x, int y, int z) { return &node_data[x * ChunkSizeSquared + y * ChunkSize + z]; }
	CubeTriangles* TerrainChunk::GetCube(int x, int y, int z) { return &tri_data[x * ChunkSizeSquared + y * ChunkSize + z]; }

	TerrainNode* TerrainChunk::GetNodeRelative(int x, int y, int z)
	{
		TerrainChunk* chunk;
		int dx, dy, dz;

		if(GetRelativePositionInfo(x, y, z, chunk, dx, dy, dz))
		{
			if(chunk != NULL)
				return chunk->GetNode(dx, dy, dz);
			else
				return NULL;
		}
		else
			return GetNode(x, y, z);
	}

	CubeTriangles* TerrainChunk::GetCubeRelative(int x, int y, int z)
	{
		TerrainChunk* chunk;
		int dx, dy, dz;

		if(GetRelativePositionInfo(x, y, z, chunk, dx, dy, dz))
		{
			if(chunk != NULL)
				return chunk->GetCube(dx, dy, dz);
			else
				return NULL;
		}
		else
			return GetCube(x, y, z);
	}

	bool TerrainChunk::GetRelativePositionInfo(int x, int y, int z, TerrainChunk*& chunk, int& dx, int &dy, int& dz)
	{
		if(x >= 0 && y >= 0 && z >= 0 && x < ChunkSize && y < ChunkSize && z < ChunkSize)
			return false;
		else
		{
			int cx = (int)floor((float)x / ChunkSize) + chunk_x;
			int cy = (int)floor((float)y / ChunkSize) + chunk_y;
			int cz = (int)floor((float)z / ChunkSize) + chunk_z;

			dx = x - (cx - chunk_x) * ChunkSize;
			dy = y - (cy - chunk_y) * ChunkSize;
			dz = z - (cz - chunk_z) * ChunkSize;

			chunk = owner->Chunk(cx, cy, cz);

			return true;
		}
	}



	void TerrainChunk::InvalidateVBO()
	{
		vbo_valid = false;
		if(model != NULL)
		{
			model->Dispose(); 
			delete model; 
			model = NULL;
		}
	}

	void TerrainChunk::InvalidateNode(int x, int y, int z)
	{
		InvalidateCubeRelative(	x - 1,	y - 1,	z - 1	);
		InvalidateCubeRelative(	x - 1,	y - 1,	z		);
		InvalidateCubeRelative(	x - 1,	y,		z - 1	);
		InvalidateCubeRelative(	x - 1,	y,		z		);
		InvalidateCubeRelative(	x,		y - 1,	z - 1	);
		InvalidateCubeRelative(	x,		y - 1,	z		);
		InvalidateCubeRelative(	x,		y,		z - 1	);
		InvalidateCubeRelative(	x,		y,		z		);
	}

	void TerrainChunk::InvalidateCubeRelative(int x, int y, int z)
	{
		TerrainChunk* chunk;
		int dx, dy, dz;

		if(GetRelativePositionInfo(x, y, z, chunk, dx, dy, dz))
		{
			if(chunk != NULL)
				chunk->GetCube(dx, dy, dz)->Invalidate();
		}
		else
			GetCube(x, y, z)->Invalidate();
	}



	// Nodes with same solidity as all neighbors get 0 or 255 solidity (whichever is appropriate)
	void TerrainChunk::Solidify()
	{
		for(int x = 0; x < ChunkSize; ++x)
			for(int y = 0; y < ChunkSize; ++y)
				for(int z = 0; z < ChunkSize; ++z)
				{
					int tot = GetNode(x, y, z)->solidity;
					if(tot == 0 || tot == 255)
						continue;

					bool solid = GetNode(x, y, z)->IsSolid();
					bool pass = true;

					for(int xx = x - 1; xx <= x + 1 && pass; ++xx)
						for(int yy = y - 1; yy <= y + 1 && pass; ++yy)
							for(int zz = z - 1; zz <= z + 1 && pass; ++zz)
							{
								TerrainNode* neighbor = GetNodeRelative(xx, yy, zz);
								if(neighbor != NULL && neighbor->IsSolid() != solid)
									pass = false;
							}

					if(pass)
						GetNode(x, y, z)->solidity = solid ? 255 : 0;
				}
	}

	void TerrainChunk::ModifySphere(Vec3 center, float inner_radius, float outer_radius, TerrainAction& action)
	{
		const float inv_range = 1.0f / (outer_radius - inner_radius);
		const float outer_radius_sq = outer_radius * outer_radius;

		int owner_dim_x = owner->GetXDim(), owner_dim_y = owner->GetYDim(), owner_dim_z = owner->GetZDim();

		int max_radius = (int)ceil(outer_radius);
		center -= Vec3(float(chunk_x * ChunkSize), float(chunk_y * ChunkSize), float(chunk_z * ChunkSize));

		int min_x = max(0, (int)floor(center.x - max_radius)), max_x = min(ChunkSize - 1, (int)ceil(center.x + max_radius));
		int min_y = max(0, (int)floor(center.y - max_radius)), max_y = min(ChunkSize - 1, (int)ceil(center.y + max_radius));
		int min_z = max(0, (int)floor(center.z - max_radius)), max_z = min(ChunkSize - 1, (int)ceil(center.z + max_radius));

		for(int xx = min_x; xx <= max_x; ++xx)
		{
			for(int yy = min_y; yy <= max_y; ++yy)
			{
				for(int zz = min_z; zz <= max_z; ++zz)
				{
					Vec3 point = Vec3(float(xx), float(yy), float(zz));
					Vec3 radius_vec = point - center;

					float dist_sq = radius_vec.ComputeMagnitudeSquared();
					if(dist_sq < outer_radius_sq)
					{
						TerrainNode& node = *GetNode(xx, yy, zz);

						float dist = sqrtf(dist_sq);
						
						action.AffectNode(this, node, xx, yy, zz, max(0, min(255, (int)(255.0f * ((outer_radius - dist) * inv_range)))));
					}
				}
			}
		}
	}




	void TerrainChunk::Vis(SceneRenderer *renderer, Mat4 main_xform)
	{
		if(!vbo_valid)
		{
			assert(model == NULL);

			model = CreateVBO();
			vbo_valid = true;
		}

		if(model != NULL)
			renderer->objects.push_back(RenderNode(material, new VoxelMaterialNodeData(model, Vec3(float(chunk_x * ChunkSize), float(chunk_y * ChunkSize), float(chunk_z * ChunkSize)), main_xform * xform), 0));
	}




	/* 
	 * CreateVBO function is quite long...
	 */
	VertexBuffer* TerrainChunk::CreateVBO()
	{
		int num_verts = 0;

		int max_x = owner->GetXDim() == chunk_x + 1 ? ChunkSize - 1 : ChunkSize + 1;
		int max_y = owner->GetYDim() == chunk_y + 1 ? ChunkSize - 1 : ChunkSize + 1;
		int max_z = owner->GetZDim() == chunk_z + 1 ? ChunkSize - 1 : ChunkSize + 1;

		int vbo_x_span = max_y * max_z;

		vector<TerrainVertex> unique_vertices;
		vector<unsigned int>* vertex_indices = new vector<unsigned int>[vbo_x_span * max_x];

		for(int x = 0; x < max_x; ++x)
		{
			for(int y = 0; y < max_y; ++y)
			{
				for(int z = 0; z < max_z; ++z)
				{
					// find the verts for this one cube
					CubeTriangles* cube = GetCubeRelative(x, y, z);

					if(cube != NULL)
					{						
						cube->BuildAsNeeded();

						char cube_vert_count = cube->num_vertices;
						if(cube_vert_count == 0)
							continue;

						num_verts += cube_vert_count;

						char* cube_indices = &cube->cache->indices[0];
						TerrainVertex* cube_verts = &cube->cache->verts[0];
						int cube_global_indices[12];							// key = index in "cube_verts", value = index in "unique_vertices"

						unsigned short int known_mask = 0;

						vector<unsigned int> cube_vertex_indices;
						for(int i = 0; i < cube_vert_count; ++i)
						{
							// find out if this vert is a duplicate of one which has already been assigned an index
							char index = cube_indices[i];
							if((known_mask & (1 << index)) == 0)
							{
								TerrainVertex& vert = cube_verts[index];
								Vec3& pos = vert.pos;

								bool found = false;
								unsigned int use_index = unique_vertices.size();		// if we don't find a duplicate vert, use the next available vert

								for(int xx = x - 1; xx >= 0 && xx <= x && !found; ++xx)						
									for(int yy = y - 1; yy >= 0 && yy <= y && !found; ++yy)
										for(int zz = z - 1; zz >= 0 && zz <= z && !found; ++zz)
										{
											vector<unsigned int>& indices = vertex_indices[xx * vbo_x_span + yy * max_z + zz];
											for(vector<unsigned int>::iterator jter = indices.begin(); jter != indices.end(); ++jter)
											{
												Vec3 vertex_pos = unique_vertices[*jter].pos;
											
												if(	(vertex_pos.x == pos.x && vertex_pos.y == pos.y && fabs(vertex_pos.z - pos.z) < 0.000001f) ||
													(vertex_pos.x == pos.x && vertex_pos.z == pos.z && fabs(vertex_pos.y - pos.y) < 0.000001f) ||
													(vertex_pos.y == pos.y && vertex_pos.z == pos.z && fabs(vertex_pos.x - pos.x) < 0.000001f))
												{
													use_index = *jter;
													found = true;
													break;
												}
											}
										}

								// vert doesn't already exist; create it
								if(!found)
									unique_vertices.push_back(vert);

								cube_global_indices[index] = use_index;

								known_mask |= (1 << index);
							}

							cube_vertex_indices.push_back(cube_global_indices[index]);
						}

						vertex_indices[x * vbo_x_span + y * max_z + z] = cube_vertex_indices;
					}
				}
			}
		}

		Vec3* normal_vectors = new Vec3[unique_vertices.size()];
		for(unsigned int i = 0; i < unique_vertices.size(); ++i)
			normal_vectors[i] = Vec3();

		// go back through them and find the normal vectors (faster than it used to be!)
		for(int x = 0; x < max_x; ++x)
		{
			for(int y = 0; y < max_y; ++y)
			{
				for(int z = 0; z < max_z; ++z)
				{
					vector<unsigned int>& cube_verts = vertex_indices[x * vbo_x_span + y * max_z + z];

					if(cube_verts.empty())
						continue;

					// iterate through all of the verts
					for(vector<unsigned int>::iterator iter = cube_verts.begin(); iter != cube_verts.end(); )
					{
						unsigned int a = *(iter++), b = *(iter++), c = *(iter++);
						Vec3 va = unique_vertices[a].pos, vb = unique_vertices[b].pos, vc = unique_vertices[c].pos;

						Vec3 tri_normal = Vec3::Cross(vb - va, vc - va);

						float len_sq = tri_normal.ComputeMagnitudeSquared();
						if(len_sq > 0.0f)
						{
							//tri_normal /= sqrtf(len_sq);

							normal_vectors[a] += tri_normal;
							normal_vectors[b] += tri_normal;
							normal_vectors[c] += tri_normal;
						}
					}
				}
			}
		}

		if(num_verts == 0)
			return NULL;
		else
		{
			VertexBuffer* model = new VertexBuffer(Triangles);

			model->AddAttribute("gl_Vertex",			Float, 3);
			model->AddAttribute("gl_Normal",			Float, 3);
			model->AddAttribute("material_weights",		Float, 3);

			// TODO: fix it so verts from other cubes (included for normal vector calculation) don't get included in this
			model->SetNumVerts(num_verts);

			float* vertex_ptr = model->GetFloatPointer("gl_Vertex");
			float* normal_ptr = model->GetFloatPointer("gl_Normal");
			float* mat_ptr = model->GetFloatPointer("material_weights");

			// we extended to ChunkSize + 2 to make the normal vectors compute correctly, but now we only want the ones within this chunk
			int cmax_x = min(max_x, ChunkSize);
			int cmax_y = min(max_y, ChunkSize);
			int cmax_z = min(max_z, ChunkSize);

			// now build the actual vbo with the values we computed
			for(int x = 0; x < cmax_x; ++x)
			{
				for(int y = 0; y < cmax_y; ++y)
				{
					for(int z = 0; z < cmax_z; ++z)
					{
						vector<unsigned int>& cube_verts = vertex_indices[x * vbo_x_span + y * max_z + z];

						if(cube_verts.empty())
							continue;

						// iterate through all of the verts in this cube
						for(vector<unsigned int>::iterator iter = cube_verts.begin(); iter != cube_verts.end(); ++iter)
						{
							TerrainVertex& vert = unique_vertices[*iter];

							Vec3 pos = vert.pos;

							Vec3 normal = Vec3::Normalize(normal_vectors[*iter]);

							// put the data for this vertex into the VBO
							*(vertex_ptr++) = pos.x;
							*(vertex_ptr++) = pos.y;
							*(vertex_ptr++) = pos.z;

							*(normal_ptr++) = normal.x;
							*(normal_ptr++) = normal.y;
							*(normal_ptr++) = normal.z;

							// TODO: deal with materials!
							*(mat_ptr++) = 1.0f;
							*(mat_ptr++) = 0.0f;
							*(mat_ptr++) = 0.0f;
						}
					}
				}
			}

			delete[] normal_vectors;
			delete[] vertex_indices;

			model->BuildVBO();
			return model;
		}
	}




	/*
	 * TerrainChunk I/O functions
	 */
	unsigned int TerrainChunk::Write(ostream& stream)
	{
		for(vector<TerrainNode>::iterator iter = node_data.begin(); iter != node_data.end(); ++iter)
			if(unsigned int node_write_error = iter->Write(stream))
				return node_write_error;
		return 0;
	}

	unsigned int TerrainChunk::Read(istream& stream)
	{
		for(vector<TerrainNode>::iterator iter = node_data.begin(); iter != node_data.end(); ++iter)
			if(unsigned int node_read_error = iter->Read(stream))
				return node_read_error;
		return 0;
	}
}
