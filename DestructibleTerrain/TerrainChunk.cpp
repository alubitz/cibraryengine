#include "StdAfx.h"
#include "TerrainChunk.h"
#include "TerrainLeaf.h"
#include "VoxelMaterial.h"

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
							if(chunk_x > 0)
							{
								use_chunk_x = chunk_x - 1;
								use_x = dim[0] - 1;
							}
							else
								continue;
						else if(xx == dim[0])
							if(chunk_x + 1 < owner_dim_x)
							{
								use_chunk_x = chunk_x + 1;
								use_x = 0;
							}
							else
								continue;

						for(int yy = y - 1; yy <= y + 1 && pass; yy++)
						{
							int use_chunk_y = chunk_y, use_y = yy;

							if(yy == -1)
								if(chunk_y > 0)
								{
									use_chunk_y = chunk_y - 1;
									use_y = dim[1] - 1;
								}
								else
									continue;
							else if(yy == dim[1])
								if(chunk_y + 1 < owner_dim_y)
								{
									use_chunk_y = chunk_y + 1;
									use_y = 0;
								}
								else
									continue;

							for(int zz = z - 1; zz <= z + 1 && pass; zz++)
							{
								int use_chunk_z = chunk_z, use_z = zz;

								if(zz == -1)
									if(chunk_z > 0)
									{
										use_chunk_z = chunk_z - 1;
										use_z = dim[0] - 1;
									}
									else
										continue;
								else if(zz == dim[0])
									if(chunk_z + 1 < owner_dim_z)
									{
										use_chunk_z = chunk_z + 1;
										use_z = 0;
									}
									else
										continue;

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

	void TerrainChunk::GetDimensions(int& x, int& y, int& z)
	{
		x = dim[0];
		y = dim[1];
		z = dim[2];
	}

	// internal struct for the erosion simulation
	struct ErosionNode
	{
		static const unsigned char saturation_n = 1;
		static const unsigned char saturation_d = 8;

		short int stone;
		short int dirt, nu_dirt;
		short int water, nu_water;
		short int air;

		ErosionNode(TerrainLeaf& leaf)
		{
			short int leaf_stone = leaf.GetMaterialAmount(1);
			short int leaf_dirt = leaf.GetMaterialAmount(2);
			short int total = leaf_stone + leaf_dirt;

			if(total > 0)
			{
				stone = (short int)leaf_stone * leaf.solidity / total;
				dirt = 0;
				water = (short int)leaf_dirt * leaf.solidity / total;
			}
			else
				water = stone = dirt = 0;

			//water = nu_water = 0;
			air = 255 - stone - dirt - max(0, water - dirt * saturation_n / saturation_d);;

			nu_water = water;
			nu_dirt = dirt;
		}

		TerrainLeaf ToTerrainLeaf()
		{
			TerrainLeaf result;

			assert(stone >= 0);
			assert(dirt >= 0);
			//assert(stone + dirt <= 255);

			result.solidity = stone + water;
			if(stone > water)
				result.SetMaterialAmount(1, 255);
			else
				result.SetMaterialAmount(2, 255);
			/*
			result.SetMaterialAmount(1, (unsigned char)min(255, stone));
			result.SetMaterialAmount(2, (unsigned char)min(255, water));
			*/

			return result;
		}

		// NOTE: this changes both * and nu_* based on nu_*
		void FillWithWater()
		{
			water = nu_water = max(water, 128 - stone);
			air = 0;
		}

		void FinishUpdate()
		{
			dirt = nu_dirt = max(0, nu_dirt);

			// evaporation
			water = nu_water = max(0, nu_water);
			/*
			for(int i = 0; i < nu_water; i++)
				if(Random3D::RandInt() % 100 == 0)
					water--;
			nu_water = water;
			*/

			air = 255 - stone - dirt - max(0, water - dirt * saturation_n / saturation_d);

			//assert(dirt >= 0);
			//assert(water >= 0);
			//assert(dirt >= 0 && dirt <= 255);
			//assert(water >= 0 && water <= 255);

			//assert(dirt + stone <= 255);
		}

		/**
		 * Computes how much water and dirt are in the runoff from this node. The results should be subtracted from the node's contents in the updated node.
		 *
		 * @param runoff_water how much water runs off
		 * @param runoff_dirt how much dirt is in the runoff water
		 * @return true if there is any runoff, false otherwise
		 */
		bool GetRunoff(short int& runoff_water, short int& runoff_dirt)
		{
			short int saturation_point = dirt * saturation_n / saturation_d;
			if(water <= saturation_point)
				return false;

			runoff_water = (water - saturation_point) / 8;
			if(runoff_water == 0)
				return false;

			runoff_dirt = min(dirt, runoff_water * 8);

			return true;
		}

		void TransferMaterial(ErosionNode& to, short int xfer_water, short int xfer_dirt)
		{
			short int xfer_air = xfer_water + xfer_dirt;

			nu_water -= xfer_water;
			nu_dirt -= xfer_dirt;

			to.nu_water += xfer_water;
			to.nu_dirt += xfer_dirt;
		}
	};

	struct NodeIndex 
	{ 
		int x, y, z; 
		NodeIndex(int x, int y, int z) : x(x), y(y), z(z) { }

		// NOTE: horizontal neighbors!
		vector<NodeIndex> GetNeighbors(int dim[3])
		{
			vector<NodeIndex> results;

			if(x > 0)
			{
				results.push_back(NodeIndex(x - 1, y, z));
				if(z > 0)
					results.push_back(NodeIndex(x - 1, y, z - 1));
				if(z + 1 < dim[2])
					results.push_back(NodeIndex(x - 1, y, z + 1));
			}
			if(x + 1 < dim[0])
			{
				results.push_back(NodeIndex(x + 1, y, z));
				if(z > 0)
					results.push_back(NodeIndex(x + 1, y, z - 1));
				if(z + 1 < dim[2])
					results.push_back(NodeIndex(x + 1, y, z + 1));
			}
			if(z > 0)
				results.push_back(NodeIndex(x, y, z - 1));
			if(z + 1 < dim[2])
				results.push_back(NodeIndex(x, y, z + 1));

			return results;
		}
		
		// this is so that we can have a set<NodeIndex>
		struct Comp { bool operator ()(const NodeIndex& a, const NodeIndex& b) { return a.x < b.x ? true : a.x == b.x ? a.y < b.y ? true : a.y == b.y ? a.z < b.z : false : false; } };
	};

	void TerrainChunk::Erode()
	{
		// convert TerrainLeaf to ErosionNode
		vector<ErosionNode> nodes = vector<ErosionNode>();
		for(vector<TerrainLeaf>::iterator iter = data.begin(); iter != data.end(); iter++)
			nodes.push_back(ErosionNode(*iter));

		// do a few episodic "rainfalls" (set max to something more than 1)
		for(int i = 0; i < 1; i++)
		{
			vector<NodeIndex> wet_nodes;

			for(int x = 0; x < dim[0]; x++)
				for(int y = 0; y < dim[1]; y++)
					for(int z = 0; z < dim[2]; z++)
						if(nodes[x * x_span + y * dim[2] + z].water > 0)
							wet_nodes.push_back(NodeIndex(x, y, z));

			// main simulation loop
			int tick = 0;
			//while(true)
			while(tick++ < 200)
			{
				/*
				tick++;
				if(tick <= 10)		// rain falls for the first several ticks, flows for the rest
				{
					// place some "rain" in the top layer of the grid
					int y_offset = (dim[1] - 1) * dim[2];
					for(int x = 0; x < dim[0]; x++)
						for(int z = 0; z < dim[2]; z++)
						{
							nodes[x * x_span + y_offset + z].FillWithWater();
							wet_nodes.push_back(NodeIndex(x, dim[1] - 1, z));
						}
				}*/

				// put all the nodes which have water or are neighboring a node which has water into a set
				set<NodeIndex, NodeIndex::Comp> maybe_wet_nodes;
				for(vector<NodeIndex>::iterator iter = wet_nodes.begin(); iter != wet_nodes.end(); iter++)
				{
					maybe_wet_nodes.insert(*iter);

					if(iter->y > 0)
						maybe_wet_nodes.insert(NodeIndex(iter->x, iter->y - 1, iter->z));

					vector<NodeIndex> neighbors = iter->GetNeighbors(dim);

					for(vector<NodeIndex>::iterator jter = neighbors.begin(); jter != neighbors.end(); jter++)
						maybe_wet_nodes.insert(*jter);
				}

				// change some nodes
				for(set<NodeIndex, NodeIndex::Comp>::iterator iter = maybe_wet_nodes.begin(); iter != maybe_wet_nodes.end(); iter++)
				{
					int x = iter->x, y = iter->y, z = iter->z;

					unsigned int index = x * x_span + y * dim[2] + z;
					ErosionNode& node = nodes[index];

					short int water, dirt;
					if(node.GetRunoff(water, dirt))
					{	
						short int total_runoff = water + dirt;
						if(y > 0)												// is there a node directly below us the runoff can flow too?
						{
							ErosionNode& below = nodes[index - dim[1]];

							if(below.air >= water + dirt)						// can we move all of this runoff to the node below?
							{
								node.TransferMaterial(below, water, dirt);
								continue;										// done here; process the next node
							}
							else												// move as much as possible	
							{
								short int xfer_water = water * below.air / total_runoff;
								short int xfer_dirt = dirt * below.air / total_runoff;

								node.TransferMaterial(below, xfer_water, xfer_dirt);

								water -= xfer_water;
								dirt -= xfer_dirt;
							}
						}
					
						// there must still some runoff to deal with... let's try to move it sideways
						// we'll do that by trying to level out the amount of air in this and neighboring nodes...

						// map to each neighboring node the amount of available space we can transfer to it
						// treat diagonals as having less space, because they are further away

						vector<NodeIndex> neighbor_indices = iter->GetNeighbors(dim);
						map<ErosionNode*, short int> neighbors;

						short int total_air = 0;
						for(vector<NodeIndex>::iterator jter = neighbor_indices.begin(); jter != neighbor_indices.end(); jter++)
						{
							ErosionNode& neighbor = nodes[jter->x * x_span + jter->y * dim[2] + jter->z];
							short int amount = neighbor.air;
							if(amount > 0)
							{
								total_air += amount;
								switch(abs(jter->x - iter->x) + abs(jter->z - iter->z))
								{
								case 1:

									neighbors[&neighbor] = amount;
									break;

								case 2:

									neighbors[&neighbor] = amount * 3 / 4;
									break;

								default:

									assert(0 && "not a valid neighbor!");
									break;
								}
							}
						}

						if(total_air > 0)
						{
							short int spare_water = water, spare_dirt = dirt;
							for(map<ErosionNode*, short int>::iterator iter = neighbors.begin(); iter != neighbors.end(); iter++)
							{
								ErosionNode& neighbor = *iter->first;
								short int air_available = iter->second;

								short int xfer_water = water * air_available / total_air;
								short int xfer_dirt = dirt * air_available / total_air;

								node.TransferMaterial(neighbor, xfer_water, xfer_dirt);
								spare_water -= xfer_water;
								spare_dirt -= xfer_dirt;
							}
						}
					}
				}

				wet_nodes.clear();
				long unsigned int total_water = 0;

				for(set<NodeIndex, NodeIndex::Comp>::iterator iter = maybe_wet_nodes.begin(); iter != maybe_wet_nodes.end(); iter++)
				{
					ErosionNode& node = nodes[iter->x * x_span + iter->y * dim[2] + iter->z];

					node.FinishUpdate();

					if(node.water > 0)
					{
						wet_nodes.push_back(*iter);
						total_water += node.water;
					}
				}

				stringstream ss;
				ss << "tick " << tick << ": " << total_water << " water in " << wet_nodes.size() << " nodes" << endl;
				Debug(ss.str());
			}
		}

		// convert ErosionNode back to TerrainLeaf
		data.clear();
		for(vector<ErosionNode>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
			data.push_back(iter->ToTerrainLeaf());

		Solidify();

		InvalidateVBO();
	}

	TerrainChunk::~TerrainChunk() { InvalidateVBO(); }

	void TerrainChunk::InvalidateVBO()
	{
		if(model != NULL)
		{
			model->Dispose(); 
			delete model; 
			model = NULL;
		}
	}

	TerrainLeaf& TerrainChunk::Element(int x, int y, int z) { return data[x * x_span + y * dim[2] + z]; }

	void TerrainChunk::Vis(SceneRenderer *renderer, Mat4 main_xform)
	{
		if(model == NULL)
			model = CreateVBO();

		renderer->objects.push_back(RenderNode(material, new VoxelMaterialNodeData(model, main_xform * xform), 0));
	}
	
	// Used to generate model via marching cubes algorithm
	template <class I, class O> void Polygonize(Vec3 base_xyz, I grid[8], float isolevel, vector<O>& target);

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
						if(chunk_x + 1 < world_dim_x)
						{
							chunk_x++;
							use_x = 0;
						}
						else
							assert(false);
							
					if(y == t->dim[1])
						if(chunk_y + 1 < world_dim_y)
						{
							chunk_y++;
							use_y = 0;
						}
						else
							assert(false);

					if(z == t->dim[2])
						if(chunk_z + 1 < world_dim_z)
						{
							chunk_z++;
							use_z = 0;
						}
						else
							assert(false);

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
			for(int y = 1; y < max_y; y++)
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
					Polygonize(Vec3(float(x), float(y), float(z)), grid, 0.0f, cube_vertex_data);

					num_verts += cube_vertex_data.size();

					vertex_data[x * vbo_x_span + y * max_z + z] = cube_vertex_data;
				}

		model->SetNumVerts(num_verts);

		float* vertex_ptr = model->GetFloatPointer("gl_Vertex");
		float* normal_ptr = model->GetFloatPointer("gl_Normal");
		float* color_ptr = model->GetFloatPointer("gl_Color");

		// now go back through them and find the normal vectors...
		for(int x = 1; x < max_x; x++)
			for(int y = 1; y < max_y; y++)
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
							for(int j = y - 1; j <= y + 1 && j < max_y; j++)
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

		delete[] vertex_data;

		model->BuildVBO();

		stringstream ss;
		ss << "Marching cubes algorithm produced a mesh with " << (num_verts / 3) << " triangles" << endl;
		Debug(ss.str());

		return model;
	}

	void TerrainChunk::Explode(Vec3 blast_center, float blast_force)
	{
		blast_center -= Vec3(float(chunk_x * 8), float(chunk_y * 8), float(chunk_z * 8));

		float blast_radius_sq = blast_force * blast_force;

		const float blast_force_multiplier = 100.0f;

		int blast_radius = (int)ceil(sqrtf(blast_force_multiplier * blast_force - 1.0f));

		int min_x = max(0, (int)floor(blast_center.x - blast_radius)), max_x = min(dim[0] - 1, (int)ceil(blast_center.x + blast_radius));
		int min_y = max(0, (int)floor(blast_center.y - blast_radius)), max_y = min(dim[1] - 1, (int)ceil(blast_center.y + blast_radius));
		int min_z = max(0, (int)floor(blast_center.z - blast_radius)), max_z = min(dim[2] - 1, (int)ceil(blast_center.z + blast_radius));

		bool any = false;

		for(int xx = min_x; xx <= max_x; xx++)
			for(int yy = min_y; yy <= max_y; yy++)
				for(int zz = min_z; zz <= max_z; zz++)
				{
					Vec3 point = Vec3(float(xx), float(yy), float(zz));
					Vec3 radius_vec = point - blast_center;

					float dist_sq = radius_vec.ComputeMagnitudeSquared();
					float damage = blast_force * blast_force_multiplier / (dist_sq + 1.0f);

					if(damage >= 1)
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

		if(any)
		{
			Solidify();
			InvalidateVBO();
		}
	}

	// used in Polygonize
	template <class O, class I> O InterpolateVertex(float isolevel, I p1, I p2);

	template <class I, class O> void Polygonize(Vec3 base_xyz, I grid[8], float isolevel, vector<O>& target)
	{
		// tables and much of the algorithm is from http://paulbourke.net/geometry/polygonise
		int classification = 0;

		if(grid[0].value < isolevel) { classification |= 1; }
		if(grid[1].value < isolevel) { classification |= 2; }
		if(grid[2].value < isolevel) { classification |= 4; }
		if(grid[3].value < isolevel) { classification |= 8; }
		if(grid[4].value < isolevel) { classification |= 16; }
		if(grid[5].value < isolevel) { classification |= 32; }
		if(grid[6].value < isolevel) { classification |= 64; }
		if(grid[7].value < isolevel) { classification |= 128; }

		static const int edge_table[256] = 
		{
			0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
			0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
			0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
			0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
			0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
			0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
			0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
			0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
			0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
			0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
			0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
			0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
			0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
			0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
			0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
			0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
			0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
			0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
			0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
			0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
			0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
			0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
			0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
			0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
			0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
			0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
			0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
			0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
			0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
			0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
			0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
			0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
		};
		static const int tri_table[256][16] =
		{
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1},
			{3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1},
			{3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1},
			{3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1},
			{9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1},
			{9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
			{2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1},
			{8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1},
			{9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
			{4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1},
			{3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1},
			{1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1},
			{4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1},
			{4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
			{5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1},
			{2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1},
			{9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1},
			{0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1},
			{2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1},
			{10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1},
			{4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1},
			{5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1},
			{5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1},
			{9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1},
			{0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1},
			{1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1},
			{10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1},
			{8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1},
			{2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1},
			{7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1},
			{2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1},
			{11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1},
			{5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1},
			{11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1},
			{11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
			{1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1},
			{9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1},
			{5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1},
			{2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1},
			{5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1},
			{6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1},
			{3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1},
			{6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1},
			{5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1},
			{1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1},
			{10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1},
			{6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1},
			{8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1},
			{7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1},
			{3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1},
			{5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1},
			{0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1},
			{9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1},
			{8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1},
			{5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1},
			{0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1},
			{6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1},
			{10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1},
			{10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1},
			{8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1},
			{1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1},
			{0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1},
			{10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1},
			{3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1},
			{6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1},
			{9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1},
			{8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1},
			{3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1},
			{6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1},
			{0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1},
			{10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1},
			{10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1},
			{2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1},
			{7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1},
			{7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1},
			{2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1},
			{1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1},
			{11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1},
			{8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1},
			{0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1},
			{7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
			{10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
			{2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1},
			{6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1},
			{7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1},
			{2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1},
			{1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1},
			{10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1},
			{10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1},
			{0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1},
			{7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1},
			{6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1},
			{8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1},
			{9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1},
			{6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1},
			{4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1},
			{10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1},
			{8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1},
			{0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1},
			{1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1},
			{8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1},
			{10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1},
			{4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1},
			{10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1},
			{5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
			{11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1},
			{9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1},
			{6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1},
			{7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1},
			{3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1},
			{7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1},
			{9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1},
			{3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1},
			{6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1},
			{9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1},
			{1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1},
			{4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1},
			{7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1},
			{6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1},
			{3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1},
			{0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1},
			{6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1},
			{0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1},
			{11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1},
			{6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1},
			{5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1},
			{9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1},
			{1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1},
			{1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1},
			{10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1},
			{0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1},
			{5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1},
			{10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1},
			{11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1},
			{9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1},
			{7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1},
			{2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1},
			{8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1},
			{9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1},
			{9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1},
			{1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1},
			{9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1},
			{9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1},
			{5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1},
			{0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1},
			{10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1},
			{2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1},
			{0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1},
			{0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1},
			{9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1},
			{5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1},
			{3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1},
			{5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1},
			{8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1},
			{0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1},
			{9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1},
			{0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1},
			{1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1},
			{3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1},
			{4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1},
			{9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1},
			{11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1},
			{11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1},
			{2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1},
			{9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1},
			{3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1},
			{1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1},
			{4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1},
			{4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1},
			{0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1},
			{3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1},
			{3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1},
			{0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1},
			{9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1},
			{1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
			{-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
		};

		if (edge_table[classification] == 0)
			return;

		O vert_list[12];

		/* Find the vertices where the surface intersects the cube */
		if (edge_table[classification] & 1)
			vert_list[0] = InterpolateVertex<O>(isolevel, grid[0], grid[1]);
		if (edge_table[classification] & 2)
			vert_list[1] = InterpolateVertex<O>(isolevel, grid[1], grid[2]);
		if (edge_table[classification] & 4)
			vert_list[2] = InterpolateVertex<O>(isolevel, grid[2], grid[3]);
		if (edge_table[classification] & 8)
			vert_list[3] = InterpolateVertex<O>(isolevel, grid[3], grid[0]);
		if (edge_table[classification] & 16)
			vert_list[4] = InterpolateVertex<O>(isolevel, grid[4], grid[5]);
		if (edge_table[classification] & 32)
			vert_list[5] = InterpolateVertex<O>(isolevel, grid[5], grid[6]);
		if (edge_table[classification] & 64)
			vert_list[6] = InterpolateVertex<O>(isolevel, grid[6], grid[7]);
		if (edge_table[classification] & 128)
			vert_list[7] = InterpolateVertex<O>(isolevel, grid[7], grid[4]);
		if (edge_table[classification] & 256)
			vert_list[8] = InterpolateVertex<O>(isolevel, grid[0], grid[4]);
		if (edge_table[classification] & 512)
			vert_list[9] = InterpolateVertex<O>(isolevel, grid[1], grid[5]);
		if (edge_table[classification] & 1024)
			vert_list[10] = InterpolateVertex<O>(isolevel, grid[2], grid[6]);
		if (edge_table[classification] & 2048)
			vert_list[11] = InterpolateVertex<O>(isolevel, grid[3], grid[7]);

		/* Add the triangles to the vertex buffer */
		for(int i = 0; tri_table[classification][i] != -1; i += 3)
		{	
			target.push_back(vert_list[tri_table[classification][i    ]]);
			target.push_back(vert_list[tri_table[classification][i + 1]]);
			target.push_back(vert_list[tri_table[classification][i + 2]]);
		}
	}

	template <class O, class I> O InterpolateVertex(float isolevel, I p1, I p2)
	{
		float v1 = p1.value, v2 = p2.value;

		if (fabs(isolevel - v1) < 0.00001f)
			return(O(p1));
		if (fabs(isolevel - v2) < 0.00001f)
			return(O(p2));
		if (fabs(v1 - v2) < 0.00001f)
			return(O(p1));

		float mu = (isolevel - v1) / (v2 - v1);
		return O(p1) * (1.0f - mu) + O(p2) * mu;
	}
}
