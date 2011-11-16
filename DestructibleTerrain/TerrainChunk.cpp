#include "StdAfx.h"
#include "TerrainChunk.h"
#include "TerrainLeaf.h"
#include "VoxelMaterial.h"

#include "MarchingCubes.h"

namespace DestructibleTerrain
{
	/*
	 * TerrainChunk methods
	 */
	TerrainChunk::TerrainChunk(VoxelMaterial* material, VoxelTerrain* owner, int x, int y, int z) :
		data(),
		chunk_x(x),
		chunk_y(y),
		chunk_z(z),
		material(material),
		model(NULL),
		owner(owner)
	{

		xform = Mat4::Translation(float(x * ChunkSize), float(y * ChunkSize), float(z * ChunkSize));

		for(int i = 0; i < ChunkSize * ChunkSize * ChunkSize; i++)
			data.push_back(TerrainLeaf());
	}

	TerrainChunk::~TerrainChunk() { InvalidateVBO(); }

	TerrainLeaf& TerrainChunk::Element(int x, int y, int z) { return data[x * ChunkSizeSquared + y * ChunkSize + z]; }

	TerrainLeaf* TerrainChunk::GetElementRelative(int x, int y, int z)
	{
		if(x >= 0 && y >= 0 && z >= 0 && x < ChunkSize && y < ChunkSize && z < ChunkSize)
			return &data[x * ChunkSizeSquared + y * ChunkSize + z];
		else
		{
			int cx = x / ChunkSize + chunk_x, cy = y / ChunkSize + chunk_y, cz = z / ChunkSize + chunk_z;
			int dx = x - (cx - chunk_x) * ChunkSize, dy = y - (cy - chunk_y) * ChunkSize, dz = z - (cz - chunk_z) * ChunkSize;

			TerrainChunk* neighbor_chunk = owner->Chunk(cx, cy, cz);
			if(neighbor_chunk == NULL)
				return NULL;
			else
				return &neighbor_chunk->Element(dx, dy, dz);
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

		struct GridStruct
		{
			float value;
			Vec3 position;
			Vec4 color;

			GridStruct(TerrainChunk* t, int x, int y, int z) : position(float(x), float(y), float(z)) 
			{
				TerrainLeaf* leaf_ptr = t->GetElementRelative(x, y, z);
				assert(leaf_ptr != NULL);

				TerrainLeaf& leaf = *leaf_ptr;
				color = Vec4();

				if(leaf.IsSolid())
				{
					for(int i = 0; i < 4; i++)
					{
						switch(leaf.types[i])
						{
						case 1:
						
							// stone color
							color += Vec4(0.5f, 0.5f, 0.5f, 1.0f) * leaf.weights[i];
							break;

						case 2:
						
							// dirt (sand) color
							color += Vec4(0.85f, 0.75f, 0.55f, 1.0f) * leaf.weights[i];
							break;
						}
					}
				}

				value = leaf.GetScalarValue();
				position = Vec3(float(x), float(y), float(z));

				color *= color.w == 0 ? 1.0f : 1.0f / color.w;
			}
		};

		struct VertStruct
		{
			Vec3 pos;
			Vec4 color;

			VertStruct() : pos(), color(1.0f, 1.0f, 1.0f, 0.0f) { }
			VertStruct(Vec3 pos, Vec4 color) : pos(pos), color(color) { }
			VertStruct(GridStruct grid) : pos(grid.position), color(grid.color) { }
			
			VertStruct operator *(float amount) { return VertStruct(pos * amount, color * amount); }
			VertStruct operator +(VertStruct a) { return VertStruct(pos + a.pos, color + a.color); }
		};

		int num_verts = 0;

		int max_x, max_y, max_z;
		owner->GetDimensions(max_x, max_y, max_z);

		max_x = max_x == chunk_x + 1 ? ChunkSize : ChunkSize + 1;
		max_y = max_y == chunk_y + 1 ? ChunkSize : ChunkSize + 1;
		max_z = max_z == chunk_z + 1 ? ChunkSize : ChunkSize + 1;
		
		int vbo_x_span = max_y * max_z;

		vector<VertStruct> unique_vertices;
		vector<unsigned int>* vertex_indices = new vector<unsigned int>[vbo_x_span * max_x];

		for(int x = 1; x < max_x; x++)
		{
			for(int y = 1; y < max_y; y++)
			{
				for(int z = 1; z < max_z; z++)
				{
					GridStruct grid[] =
					{
						GridStruct(this, x - 1, y - 1, z - 1),	
						GridStruct(this, x - 1, y - 1, z),
						GridStruct(this, x - 1, y, z),
						GridStruct(this, x - 1, y, z - 1),			
						GridStruct(this, x, y - 1, z - 1),	
						GridStruct(this, x, y - 1, z),
						GridStruct(this, x, y, z),
						GridStruct(this, x, y, z - 1)
					};

					// find the verts for this one cube
					vector<VertStruct> cube_vertex_data;
					MarchingCubes::Polygonize(Vec3(float(x), float(y), float(z)), grid, 0.0f, cube_vertex_data);

					num_verts += cube_vertex_data.size();

					vector<unsigned int> cube_vertex_indices = vector<unsigned int>();
					for(vector<VertStruct>::iterator iter = cube_vertex_data.begin(); iter != cube_vertex_data.end(); iter++)
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
										VertStruct& value = unique_vertices[*jter];
										if((value.pos - pos).ComputeMagnitudeSquared() < 0.000001f)
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

		Vec3* normal_vectors = new Vec3[unique_vertices.size()];
		for(unsigned int i = 0; i < unique_vertices.size(); i++)
			normal_vectors[i] = Vec3();

		// go back through them and find the normal vectors (faster than it used to be!)
		for(int x = 1; x < max_x; x++)
		{
			for(int y = 1; y < max_y; y++)
			{
				for(int z = 1; z < max_z; z++)
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
						float len = tri_normal.ComputeMagnitudeSquared();
						if(len > 0.0f)
						{
							tri_normal /= sqrtf(len);
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

		// now build the actual vbo with the values we computed
		for(int x = 1; x < max_x; x++)
		{
			for(int y = 1; y < max_y; y++)
			{
				for(int z = 1; z < max_z; z++)
				{
					vector<unsigned int>& cube_verts = vertex_indices[x * vbo_x_span + y * max_z + z];

					if(cube_verts.empty())
						continue;

					// iterate through all of the verts in this cube
					for(vector<unsigned int>::iterator iter = cube_verts.begin(); iter != cube_verts.end(); iter++)
					{
						VertStruct& vert = unique_vertices[*iter];

						Vec3 pos = vert.pos;

						Vec4 color = vert.color;
						if(color.w > 0)
							color /= color.w;

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




	/*
	 * Solidify function is also kind of long
	 */
	void TerrainChunk::Solidify()
	{
		int owner_dim_x, owner_dim_y, owner_dim_z;
		owner->GetDimensions(owner_dim_x, owner_dim_y, owner_dim_z);

		// Nodes with same solidity as all neighbors get 0 or 255 solidity (whichever is appropriate)
		for(int x = 0; x < ChunkSize; x++)
			for(int y = 0; y < ChunkSize; y++)
				for(int z = 0; z < ChunkSize; z++)
				{
					int tot = Element(x, y, z).solidity;
					if(tot == 0 || tot == 255)
						continue;

					bool solid = Element(x, y, z).IsSolid();
					bool pass = true;

					for(int xx = x - 1; xx <= x + 1 && pass; xx++)
					{
						int use_chunk_x = chunk_x, use_x = xx;

						if(xx == -1)
						{
							if(chunk_x > 0)
							{
								use_chunk_x = chunk_x - 1;
								use_x = ChunkSize - 1;
							}
							else
								continue;
						}
						else if(xx == ChunkSize)
						{
							if(chunk_x + 1 < owner_dim_x)
							{
								use_chunk_x = chunk_x + 1;
								use_x = 0;
							}
							else
								continue;
						}

						for(int yy = y - 1; yy <= y + 1 && pass; yy++)
						{
							int use_chunk_y = chunk_y, use_y = yy;

							if(yy == -1)
							{
								if(chunk_y > 0)
								{
									use_chunk_y = chunk_y - 1;
									use_y = ChunkSize - 1;
								}
								else
									continue;
							}
							else if(yy == ChunkSize)
							{
								if(chunk_y + 1 < owner_dim_y)
								{
									use_chunk_y = chunk_y + 1;
									use_y = 0;
								}
								else
									continue;
							}

							for(int zz = z - 1; zz <= z + 1 && pass; zz++)
							{
								int use_chunk_z = chunk_z, use_z = zz;

								if(zz == -1)
								{
									if(chunk_z > 0)
									{
										use_chunk_z = chunk_z - 1;
										use_z = ChunkSize - 1;
									}
									else
										continue;
								}
								else if(zz == ChunkSize)
								{
									if(chunk_z + 1 < owner_dim_z)
									{
										use_chunk_z = chunk_z + 1;
										use_z = 0;
									}
									else
										continue;
								}

								if(owner->Chunk(use_chunk_x, use_chunk_y, use_chunk_z)->Element(use_x, use_y, use_z).IsSolid() != solid)
									pass = false;
							}
						}
					}
					if(pass)
						if(solid)
							Element(x, y, z).solidity = 255;
						else
							Element(x, y, z).solidity = 0;
				}
	}




	void TerrainChunk::Explode(Vec3 blast_center, float blast_force)
	{
		const float blast_force_multiplier = 100.0f;
		const float damage_threshold = 10.0f;					// should be at least 1 (gets converted to integer)

		int blast_radius = (int)ceil(sqrtf(blast_force_multiplier * blast_force - 1.0f));
		blast_center -= Vec3(float(chunk_x * 8), float(chunk_y * 8), float(chunk_z * 8));

		int min_x = max(0, (int)floor(blast_center.x - blast_radius)), max_x = min(ChunkSize - 1, (int)ceil(blast_center.x + blast_radius));
		int min_y = max(0, (int)floor(blast_center.y - blast_radius)), max_y = min(ChunkSize - 1, (int)ceil(blast_center.y + blast_radius));
		int min_z = max(0, (int)floor(blast_center.z - blast_radius)), max_z = min(ChunkSize - 1, (int)ceil(blast_center.z + blast_radius));

		bool any = false;

		for(int xx = min_x; xx <= max_x; xx++)
		{
			for(int yy = min_y; yy <= max_y; yy++)
			{
				for(int zz = min_z; zz <= max_z; zz++)
				{
					Vec3 point = Vec3(float(xx), float(yy), float(zz));
					Vec3 radius_vec = point - blast_center;

					float dist_sq = radius_vec.ComputeMagnitudeSquared();
					float damage = blast_force * blast_force_multiplier / (dist_sq + 1.0f);

					if(damage >= damage_threshold)
					{
						TerrainLeaf& leaf = Element(xx, yy, zz);
						int nu_value = (unsigned char)max(0, (int)leaf.solidity - damage);
						if(nu_value != leaf.solidity)
						{
							leaf.solidity = nu_value;
							any = true;
						}
					}
				}
			}
		}

		if(any)
		{
			Solidify();
			InvalidateVBO();
		}
	}

	void TerrainChunk::Vis(SceneRenderer *renderer, Mat4 main_xform)
	{
		if(model == NULL)
			model = CreateVBO();

		renderer->objects.push_back(RenderNode(material, new VoxelMaterialNodeData(model, main_xform * xform), 0));
	}
}
