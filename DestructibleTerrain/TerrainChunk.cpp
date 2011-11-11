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
		dim[0] = 8;
		dim[1] = 8;
		dim[2] = 8;

		x_span = dim[1] * dim[2];

		xform = Mat4::Translation(float(x * 8), float(y * 8), float(z * 8));

		for(int i = 0; i < 512; i++)
			data.push_back(TerrainLeaf());
	}

	TerrainChunk::~TerrainChunk() { InvalidateVBO(); }

	TerrainLeaf& TerrainChunk::Element(int x, int y, int z) { return data[x * x_span + y * dim[2] + z]; }
	
	void TerrainChunk::GetDimensions(int& x, int& y, int& z)
	{
		x = dim[0];
		y = dim[1];
		z = dim[2];
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
				TerrainLeaf* leaf_ptr;
				if(x < t->dim[0] && y < t->dim[1] && z < t->dim[2])
					leaf_ptr = &t->Element(x, y, z);
				else
				{
					int world_dim_x, world_dim_y, world_dim_z;
					t->owner->GetDimensions(world_dim_x, world_dim_y, world_dim_z);

					int chunk_x = t->chunk_x, chunk_y = t->chunk_y, chunk_z = t->chunk_z;
					int use_x = x, use_y = y, use_z = z;

					if(x == t->dim[0])
					{
						if(chunk_x + 1 < world_dim_x)
						{
							chunk_x++;
							use_x = 0;
						}
						else
							assert(false);
					}
							
					if(y == t->dim[1])
					{
						if(chunk_y + 1 < world_dim_y)
						{
							chunk_y++;
							use_y = 0;
						}
						else
							assert(false);
					}

					if(z == t->dim[2])
					{
						if(chunk_z + 1 < world_dim_z)
						{
							chunk_z++;
							use_z = 0;
						}
						else
							assert(false);
					}

					leaf_ptr = &t->owner->Chunk(chunk_x, chunk_y, chunk_z)->Element(use_x, use_y, use_z);
				}

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

		max_x = max_x == chunk_x + 1 ? dim[0] : dim[0] + 1;
		max_y = max_y == chunk_y + 1 ? dim[1] : dim[1] + 1;
		max_z = max_z == chunk_z + 1 ? dim[2] : dim[2] + 1;
		
		int vbo_x_span = max_y * max_z;

		vector<VertStruct>* vertex_data = new vector<VertStruct>[vbo_x_span * max_x];

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
						GridStruct(this, x - 1, y, z),					// order of verts 6&7 swapped to conform with the tables' vertex ordering
						GridStruct(this, x - 1, y, z - 1),			
						GridStruct(this, x, y - 1, z - 1),	
						GridStruct(this, x, y - 1, z),
						GridStruct(this, x, y, z),						// order of verts 6&7 swapped to conform with the tables' vertex ordering
						GridStruct(this, x, y, z - 1)
					};

					// find the verts for this one cube
					vector<VertStruct> cube_vertex_data;
					MarchingCubes::Polygonize(Vec3(float(x), float(y), float(z)), grid, 0.0f, cube_vertex_data);

					num_verts += cube_vertex_data.size();

					vertex_data[x * vbo_x_span + y * max_z + z] = cube_vertex_data;
				}
			}
		}

		model->SetNumVerts(num_verts);

		float* vertex_ptr = model->GetFloatPointer("gl_Vertex");
		float* normal_ptr = model->GetFloatPointer("gl_Normal");
		float* color_ptr = model->GetFloatPointer("gl_Color");

		// now go back through them and find the normal vectors...
		for(int x = 1; x < max_x; x++)
		{
			for(int y = 1; y < max_y; y++)
			{
				for(int z = 1; z < max_z; z++)
				{
					vector<VertStruct>& cube_verts = vertex_data[x * vbo_x_span + y * max_z + z];

					if(cube_verts.empty())
						continue;

					// iterate through all of the verts
					for(vector<VertStruct>::iterator iter = cube_verts.begin(); iter != cube_verts.end(); iter++)
					{
						Vec3 pos = iter->pos;
						
						Vec4 color = iter->color;
						float inv = color.w == 0 ? 1.0f : 1.0f / color.w;
						color.x *= inv;
						color.y *= inv;
						color.z *= inv;

						Vec3 normal;
	
						for(int i = x - 1; i <= x + 1 && i < max_x; i++)
						{
							for(int j = y - 1; j <= y + 1 && j < max_y; j++)
							{
								for(int k = z - 1; k <= z + 1 && k < max_z; k++)
								{
									vector<VertStruct>& other_verts = vertex_data[i * vbo_x_span + j * max_z + k];
									if(other_verts.empty())
										continue;
									
									// find triangles which contain this vertex
									for(vector<VertStruct>::iterator jter = other_verts.begin(); jter != other_verts.end(); )
									{
										Vec3 a = (jter++)->pos;
										Vec3 b = (jter++)->pos;
										Vec3 c = (jter++)->pos;

										//if(pos == a || pos == b || pos == c)
										if((pos - a).ComputeMagnitudeSquared() < 0.00001f || (pos - b).ComputeMagnitudeSquared() < 0.00001f || (pos - c).ComputeMagnitudeSquared() < 0.00001f)
										{
											// add this triangle's contribution to the vert's normal vector
											Vec3 tri_normal = Vec3::Cross(b - a, c - a);
											float len = tri_normal.ComputeMagnitudeSquared();
											if(len > 0.0f)
												normal += tri_normal / sqrtf(len);
										}
									}
								}
							}
						}


						// make sure the normal vector we computed has unit magnitude
						normal = Vec3::Normalize(normal);

						// now put this data in the VBO
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

		delete[] vertex_data;

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
		for(int x = 0; x < dim[0]; x++)
			for(int y = 0; y < dim[1]; y++)
				for(int z = 0; z < dim[2]; z++)
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
								use_x = dim[0] - 1;
							}
							else
								continue;
						}
						else if(xx == dim[0])
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
									use_y = dim[1] - 1;
								}
								else
									continue;
							}
							else if(yy == dim[1])
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
										use_z = dim[0] - 1;
									}
									else
										continue;
								}
								else if(zz == dim[0])
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

		int min_x = max(0, (int)floor(blast_center.x - blast_radius)), max_x = min(dim[0] - 1, (int)ceil(blast_center.x + blast_radius));
		int min_y = max(0, (int)floor(blast_center.y - blast_radius)), max_y = min(dim[1] - 1, (int)ceil(blast_center.y + blast_radius));
		int min_z = max(0, (int)floor(blast_center.z - blast_radius)), max_z = min(dim[2] - 1, (int)ceil(blast_center.z + blast_radius));

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
