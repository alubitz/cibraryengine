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
		owner(owner)
	{
		xform = Mat4::Translation(float(x * ChunkSize), float(y * ChunkSize), float(z * ChunkSize));

		for(int i = 0; i < ChunkSize * ChunkSize * ChunkSize; i++)
			node_data.push_back(TerrainNode());

		for(int x = 0; x < ChunkCubes; x++)
			for(int y = 0; y < ChunkCubes; y++)
				for(int z = 0; z < ChunkCubes; z++)
					tri_data.push_back(CubeTriangles(this, x, y, z));
	}

	TerrainChunk::~TerrainChunk() { InvalidateVBO(); }

	TerrainNode* TerrainChunk::GetNode(int x, int y, int z) { return &node_data[x * ChunkSizeSquared + y * ChunkSize + z]; }

	TerrainNode* TerrainChunk::GetNodeRelative(int x, int y, int z)
	{
		if(x >= 0 && y >= 0 && z >= 0 && x < ChunkSize && y < ChunkSize && z < ChunkSize)
			return &node_data[x * ChunkSizeSquared + y * ChunkSize + z];
		else
		{
			int cx = (int)floor((float)x / ChunkSize) + chunk_x;
			int cy = (int)floor((float)y / ChunkSize) + chunk_y;
			int cz = (int)floor((float)z / ChunkSize) + chunk_z;

			int dx = x - (cx - chunk_x) * ChunkSize;
			int dy = y - (cy - chunk_y) * ChunkSize; 
			int dz = z - (cz - chunk_z) * ChunkSize;

			TerrainChunk* neighbor_chunk = owner->Chunk(cx, cy, cz);
			if(neighbor_chunk == NULL)
				return NULL;
			else
				return neighbor_chunk->GetNode(dx, dy, dz);
		}
	}

	CubeTriangles* TerrainChunk::GetCube(int x, int y, int z) { return &tri_data[x * ChunkCubesSquared + y * ChunkCubes + z]; }

	CubeTriangles* TerrainChunk::GetCubeRelative(int x, int y, int z)
	{
		if(x >= 0 && y >= 0 && z >= 0 && x < ChunkCubes && y < ChunkCubes && z < ChunkCubes)
			return &tri_data[x * ChunkCubesSquared + y * ChunkCubes + z];
		else
		{
			int cx = (int)floor((float)x / ChunkCubes) + chunk_x;
			int cy = (int)floor((float)y / ChunkCubes) + chunk_y;
			int cz = (int)floor((float)z / ChunkCubes) + chunk_z;

			int dx = x - (cx - chunk_x) * ChunkCubes;
			int dy = y - (cy - chunk_y) * ChunkCubes; 
			int dz = z - (cz - chunk_z) * ChunkCubes;

			TerrainChunk* neighbor_chunk = owner->Chunk(cx, cy, cz);
			if(neighbor_chunk == NULL)
				return NULL;
			else
				return neighbor_chunk->GetCube(dx, dy, dz);
		}
	}

	void TerrainChunk::InvalidateVBO()
	{
		if(model != NULL)
		{
			model->Dispose(); 
			delete model; 
			model = NULL;
		}
	}




	/* 
	 * CreateVBO function is quite long...
	 */
	VertexBuffer* TerrainChunk::CreateVBO()
	{
		VertexBuffer* model = new VertexBuffer(Triangles);

		model->AddAttribute("gl_Vertex",	Float, 3);
		model->AddAttribute("gl_Normal",	Float, 3);
		model->AddAttribute("gl_Color",		Float, 3);

		int num_verts = 0;

		int max_x, max_y, max_z;
		owner->GetDimensions(max_x, max_y, max_z);
	
		/*
		max_x = max_x == chunk_x + 1 ? ChunkSize : ChunkSize + 2;
		max_y = max_y == chunk_y + 1 ? ChunkSize : ChunkSize + 2;
		max_z = max_z == chunk_z + 1 ? ChunkSize : ChunkSize + 2;
		*/
		max_x = max_y = max_z = ChunkCubes + 1;
		
		int vbo_x_span = max_y * max_z;

		vector<TerrainVertex> unique_vertices;
		vector<unsigned int>* vertex_indices = new vector<unsigned int>[vbo_x_span * max_x];

		for(int x = 0; x < max_x; x++)
		{
			for(int y = 0; y < max_y; y++)
			{
				for(int z = 0; z < max_z; z++)
				{
					// find the verts for this one cube
					CubeTriangles* cube = GetCubeRelative(x, y, z);

					if(cube != NULL)
					{
						vector<TerrainVertex> cube_vertex_data;
						cube->AppendVertexData(cube_vertex_data);

						num_verts += cube_vertex_data.size();

						vector<unsigned int> cube_vertex_indices = vector<unsigned int>();
						for(vector<TerrainVertex>::iterator iter = cube_vertex_data.begin(); iter != cube_vertex_data.end(); iter++)
						{
							// find out if this vert is a duplicate of one which has already been assigned an index
							Vec3& pos = iter->pos;

							bool found = false;
							unsigned int use_index = unique_vertices.size();		// if we don't find a duplicate vert, use the next available vert

							for(int xx = x - 1; xx >= 0 && xx <= x && !found; xx++)						
								for(int yy = y - 1; yy >= 0 && yy <= y && !found; yy++)
									for(int zz = z - 1; zz >= 0 && zz <= z && !found; zz++)
									{
										vector<unsigned int>& indices = vertex_indices[xx * vbo_x_span + yy * max_z + zz];
										for(vector<unsigned int>::iterator jter = indices.begin(); jter != indices.end(); jter++)
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
								unique_vertices.push_back(*iter);

							cube_vertex_indices.push_back(use_index);
						}
						vertex_indices[x * vbo_x_span + y * max_z + z] = cube_vertex_indices;
					}
				}
			}
		}

		Vec3* normal_vectors = new Vec3[unique_vertices.size()];
		for(unsigned int i = 0; i < unique_vertices.size(); i++)
			normal_vectors[i] = Vec3();

		// go back through them and find the normal vectors (faster than it used to be!)
		for(int x = 0; x < max_x; x++)
		{
			for(int y = 0; y < max_y; y++)
			{
				for(int z = 0; z < max_z; z++)
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

		model->SetNumVerts(num_verts);

		float* vertex_ptr = model->GetFloatPointer("gl_Vertex");
		float* normal_ptr = model->GetFloatPointer("gl_Normal");
		float* color_ptr = model->GetFloatPointer("gl_Color");

		// we extended to ChunkSize + 2 to make the normal vectors compute correctly, but now we only want the ones within this chunk
		int cmax_x = min(max_x, ChunkSize + 1);
		int cmax_y = min(max_y, ChunkSize + 1);
		int cmax_z = min(max_z, ChunkSize + 1);

		// now build the actual vbo with the values we computed
		for(int x = 1; x < cmax_x; x++)
		{
			for(int y = 1; y < cmax_y; y++)
			{
				for(int z = 1; z < cmax_z; z++)
				{
					vector<unsigned int>& cube_verts = vertex_indices[x * vbo_x_span + y * max_z + z];

					if(cube_verts.empty())
						continue;

					// iterate through all of the verts in this cube
					for(vector<unsigned int>::iterator iter = cube_verts.begin(); iter != cube_verts.end(); iter++)
					{
						TerrainVertex& vert = unique_vertices[*iter];

						Vec3 pos = vert.pos;

						Vec4 color = vert.color;
						if(color.w > 0)
						{
							float inv = 1.0f / color.w;
							color.x *= inv;
							color.y *= inv;
							color.z *= inv;
						}

						Vec3 normal = Vec3::Normalize(normal_vectors[*iter]);

						// put the data for this vertex into the VBO
						*(vertex_ptr++) = pos.x;
						*(vertex_ptr++) = pos.y;
						*(vertex_ptr++) = pos.z;

						*(normal_ptr++) = normal.x;
						*(normal_ptr++) = normal.y;
						*(normal_ptr++) = normal.z;

						*(color_ptr++) = color.x;
						*(color_ptr++) = color.y;
						*(color_ptr++) = color.z;
					}
				}
			}
		}

		delete[] normal_vectors;
		delete[] vertex_indices;

		model->BuildVBO();
		return model;
	}

	// Nodes with same solidity as all neighbors get 0 or 255 solidity (whichever is appropriate)
	void TerrainChunk::Solidify()
	{
		for(int x = 0; x < ChunkSize; x++)
			for(int y = 0; y < ChunkSize; y++)
				for(int z = 0; z < ChunkSize; z++)
				{
					int tot = GetNode(x, y, z)->solidity;
					if(tot == 0 || tot == 255)
						continue;

					bool solid = GetNode(x, y, z)->IsSolid();
					bool pass = true;

					for(int xx = x - 1; xx <= x + 1 && pass; xx++)
						for(int yy = y - 1; yy <= y + 1 && pass; yy++)
							for(int zz = z - 1; zz <= z + 1 && pass; zz++)
							{
								TerrainNode* neighbor = GetNodeRelative(xx, yy, zz);
								if(neighbor != NULL && neighbor->IsSolid() != solid)
									pass = false;
							}

					if(pass)
						GetNode(x, y, z)->solidity = solid ? 255 : 0;
				}
	}

	void TerrainChunk::Explode(Vec3 blast_center, float blast_force, set<TerrainChunk*>& affected_chunks)
	{
		int owner_dim_x, owner_dim_y, owner_dim_z;
		owner->GetDimensions(owner_dim_x, owner_dim_y, owner_dim_z);

		const float blast_force_multiplier = 100.0f;
		const float damage_threshold = 50.0f;					// value gets converted to integer!

		int blast_radius = (int)ceil(sqrtf(blast_force_multiplier * blast_force - 1.0f));
		blast_center -= Vec3(float(chunk_x * ChunkSize), float(chunk_y * ChunkSize), float(chunk_z * ChunkSize));

		int min_x = max(0, (int)floor(blast_center.x - blast_radius)), max_x = min(ChunkSize - 1, (int)ceil(blast_center.x + blast_radius));
		int min_y = max(0, (int)floor(blast_center.y - blast_radius)), max_y = min(ChunkSize - 1, (int)ceil(blast_center.y + blast_radius));
		int min_z = max(0, (int)floor(blast_center.z - blast_radius)), max_z = min(ChunkSize - 1, (int)ceil(blast_center.z + blast_radius));

		bool has_nx_neighbor = min_x == 0 && chunk_x > 0, has_ny_neighbor = min_y == 0 && chunk_y > 0, has_nz_neighbor = min_z == 0 && chunk_z > 0;
		bool has_px_neighbor = max_x + 1 == ChunkSize && chunk_x + 1 < owner_dim_x, has_py_neighbor = max_y + 1 == ChunkSize && chunk_y + 1 < owner_dim_y, has_pz_neighbor = max_z + 1 == ChunkSize && chunk_z + 1 < owner_dim_z;

		for(int xx = min_x; xx <= max_x; xx++)
		{
			bool is_nx_neighbor = has_nx_neighbor && xx == 0;
			bool is_px_neighbor = has_px_neighbor && xx + 1 == ChunkSize;
			for(int yy = min_y; yy <= max_y; yy++)
			{
				bool is_ny_neighbor = has_ny_neighbor && yy == 0;
				bool is_py_neighbor = has_py_neighbor && yy + 1 == ChunkSize;
				for(int zz = min_z; zz <= max_z; zz++)
				{
					bool is_nz_neighbor = has_nz_neighbor && zz == 0;
					bool is_pz_neighbor = has_pz_neighbor && zz + 1 == ChunkSize;

					Vec3 point = Vec3(float(xx), float(yy), float(zz));
					Vec3 radius_vec = point - blast_center;

					float dist_sq = radius_vec.ComputeMagnitudeSquared();
					float damage = blast_force * blast_force_multiplier / (dist_sq + 1.0f);

					if(damage >= damage_threshold)
					{
						TerrainNode& node = *GetNode(xx, yy, zz);
						int nu_value = (unsigned char)max(0, (int)node.solidity - damage);
						if(nu_value != node.solidity)
						{
							node.solidity = nu_value;
							affected_chunks.insert(this);

							if(is_nx_neighbor)
							{
								affected_chunks.insert(owner->Chunk(chunk_x - 1, chunk_y, chunk_z));
								if(is_ny_neighbor)
								{
									affected_chunks.insert(owner->Chunk(chunk_x - 1, chunk_y - 1, chunk_z));

									if(is_nz_neighbor)
										affected_chunks.insert(owner->Chunk(chunk_x - 1, chunk_y - 1, chunk_z - 1));
									else if(is_pz_neighbor)
										affected_chunks.insert(owner->Chunk(chunk_x - 1, chunk_y - 1, chunk_z + 1));
								}
								else if(is_py_neighbor)
								{
									affected_chunks.insert(owner->Chunk(chunk_x - 1, chunk_y + 1, chunk_z));

									if(is_nz_neighbor)
										affected_chunks.insert(owner->Chunk(chunk_x - 1, chunk_y - 1, chunk_z - 1));
									else if(is_pz_neighbor)
										affected_chunks.insert(owner->Chunk(chunk_x - 1, chunk_y - 1, chunk_z + 1));
								}
							}
							else if(is_px_neighbor)
							{
								affected_chunks.insert(owner->Chunk(chunk_x + 1, chunk_y, chunk_z));
								if(is_ny_neighbor)
								{
									affected_chunks.insert(owner->Chunk(chunk_x + 1, chunk_y - 1, chunk_z));

									if(is_nz_neighbor)
										affected_chunks.insert(owner->Chunk(chunk_x + 1, chunk_y - 1, chunk_z - 1));
									else if(is_pz_neighbor)
										affected_chunks.insert(owner->Chunk(chunk_x + 1, chunk_y - 1, chunk_z + 1));
								}
								else if(is_py_neighbor)
								{
									affected_chunks.insert(owner->Chunk(chunk_x + 1, chunk_y + 1, chunk_z));

									if(is_nz_neighbor)
										affected_chunks.insert(owner->Chunk(chunk_x + 1, chunk_y - 1, chunk_z - 1));
									else if(is_pz_neighbor)
										affected_chunks.insert(owner->Chunk(chunk_x + 1, chunk_y - 1, chunk_z + 1));
								}
							}

							if(is_ny_neighbor)
							{
								affected_chunks.insert(owner->Chunk(chunk_x, chunk_y - 1, chunk_z));

								if(is_nz_neighbor)
									affected_chunks.insert(owner->Chunk(chunk_x, chunk_y - 1, chunk_z - 1));
								else if(is_pz_neighbor)
									affected_chunks.insert(owner->Chunk(chunk_x, chunk_y - 1, chunk_z + 1));
							}
							else if(is_py_neighbor)
							{
								affected_chunks.insert(owner->Chunk(chunk_x, chunk_y + 1, chunk_z));

								if(is_nz_neighbor)
									affected_chunks.insert(owner->Chunk(chunk_x, chunk_y + 1, chunk_z - 1));
								else if(is_pz_neighbor)
									affected_chunks.insert(owner->Chunk(chunk_x, chunk_y + 1, chunk_z + 1));
							}

							if(is_nz_neighbor)
								affected_chunks.insert(owner->Chunk(chunk_x, chunk_y, chunk_z - 1));
							else if(is_pz_neighbor)
								affected_chunks.insert(owner->Chunk(chunk_x, chunk_y, chunk_z + 1));
						}
					}
				}
			}
		}
	}

	void TerrainChunk::Vis(SceneRenderer *renderer, Mat4 main_xform)
	{
		if(model == NULL)
			model = CreateVBO();

		if(model->GetNumVerts() > 0)
			renderer->objects.push_back(RenderNode(material, new VoxelMaterialNodeData(model, main_xform * xform), 0));
	}
}