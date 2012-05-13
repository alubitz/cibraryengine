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
		vbos(),
		depth_vbo(NULL),
		vbo_valid(false),
		solidified(false),
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

	void TerrainChunk::GetChunkPosition(int& x, int& y, int& z) { x = chunk_x; y = chunk_y; z = chunk_z; }

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

		for(boost::unordered_map<unsigned char, VoxelMaterialVBO>::iterator iter = vbos.begin(); iter != vbos.end(); ++iter)
		{
			VertexBuffer* model = iter->second.vbo;

			model->Dispose();
			delete model;
		}
		vbos.clear();

		if(depth_vbo != NULL)
		{
			depth_vbo->Dispose();
			delete depth_vbo;

			depth_vbo = NULL;
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

		InvalidateCubeNormalsRelative(	x - 2,	y - 2,	z - 2	);
		InvalidateCubeNormalsRelative(	x - 2,	y - 2,	z - 1	);
		InvalidateCubeNormalsRelative(	x - 2,	y - 2,	z		);
		InvalidateCubeNormalsRelative(	x - 2,	y - 2,	z + 1	);
		InvalidateCubeNormalsRelative(	x - 2,	y - 1,	z - 2	);
		InvalidateCubeNormalsRelative(	x - 2,	y - 1,	z - 1	);
		InvalidateCubeNormalsRelative(	x - 2,	y - 1,	z		);
		InvalidateCubeNormalsRelative(	x - 2,	y - 1,	z + 1	);
		InvalidateCubeNormalsRelative(	x - 2,	y,		z - 2	);
		InvalidateCubeNormalsRelative(	x - 2,	y,		z - 1	);
		InvalidateCubeNormalsRelative(	x - 2,	y,		z		);
		InvalidateCubeNormalsRelative(	x - 2,	y,		z + 1	);
		InvalidateCubeNormalsRelative(	x - 2,	y + 1,	z - 2	);
		InvalidateCubeNormalsRelative(	x - 2,	y + 1,	z - 1	);
		InvalidateCubeNormalsRelative(	x - 2,	y + 1,	z		);
		InvalidateCubeNormalsRelative(	x - 2,	y + 1,	z + 1	);

		InvalidateCubeNormalsRelative(	x - 1,	y - 2,	z - 2	);
		InvalidateCubeNormalsRelative(	x - 1,	y - 2,	z - 1	);
		InvalidateCubeNormalsRelative(	x - 1,	y - 2,	z		);
		InvalidateCubeNormalsRelative(	x - 1,	y - 2,	z + 1	);
		InvalidateCubeNormalsRelative(	x - 1,	y + 1,	z - 2	);
		InvalidateCubeNormalsRelative(	x - 1,	y + 1,	z - 1	);
		InvalidateCubeNormalsRelative(	x - 1,	y + 1,	z		);
		InvalidateCubeNormalsRelative(	x - 1,	y + 1,	z + 1	);

		InvalidateCubeNormalsRelative(	x,		y - 2,	z - 2	);
		InvalidateCubeNormalsRelative(	x,		y - 2,	z - 1	);
		InvalidateCubeNormalsRelative(	x,		y - 2,	z		);
		InvalidateCubeNormalsRelative(	x,		y - 2,	z + 1	);
		InvalidateCubeNormalsRelative(	x,		y + 1,	z - 2	);
		InvalidateCubeNormalsRelative(	x,		y + 1,	z - 1	);
		InvalidateCubeNormalsRelative(	x,		y + 1,	z		);
		InvalidateCubeNormalsRelative(	x,		y + 1,	z + 1	);

		InvalidateCubeNormalsRelative(	x + 1,	y - 2,	z - 2	);
		InvalidateCubeNormalsRelative(	x + 1,	y - 2,	z - 1	);
		InvalidateCubeNormalsRelative(	x + 1,	y - 2,	z		);
		InvalidateCubeNormalsRelative(	x + 1,	y - 2,	z + 1	);
		InvalidateCubeNormalsRelative(	x + 1,	y - 1,	z - 2	);
		InvalidateCubeNormalsRelative(	x + 1,	y - 1,	z - 1	);
		InvalidateCubeNormalsRelative(	x + 1,	y - 1,	z		);
		InvalidateCubeNormalsRelative(	x + 1,	y - 1,	z + 1	);
		InvalidateCubeNormalsRelative(	x + 1,	y,		z - 2	);
		InvalidateCubeNormalsRelative(	x + 1,	y,		z - 1	);
		InvalidateCubeNormalsRelative(	x + 1,	y,		z		);
		InvalidateCubeNormalsRelative(	x + 1,	y,		z + 1	);
		InvalidateCubeNormalsRelative(	x + 1,	y + 1,	z - 2	);
		InvalidateCubeNormalsRelative(	x + 1,	y + 1,	z - 1	);
		InvalidateCubeNormalsRelative(	x + 1,	y + 1,	z		);
		InvalidateCubeNormalsRelative(	x + 1,	y + 1,	z + 1	);
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

	void TerrainChunk::InvalidateCubeNormalsRelative(int x, int y, int z)
	{
		TerrainChunk* chunk;
		int dx, dy, dz;

		if(GetRelativePositionInfo(x, y, z, chunk, dx, dy, dz))
		{
			if(chunk != NULL)
				chunk->GetCube(dx, dy, dz)->InvalidateNormals();
		}
		else
			GetCube(x, y, z)->InvalidateNormals();
	}




	// Nodes with same solidity as all neighbors get 0 or 255 solidity (whichever is appropriate)
	void TerrainChunk::SolidifyAsNeeded() { if(!solidified) { Solidify(); } }
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

		solidified = true;
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

						unsigned char old_solidity = node.solidity;

						float dist = sqrtf(dist_sq);
						action.AffectNode(this, node, xx, yy, zz, max(0, min(255, (int)(255.0f * ((outer_radius - dist) * inv_range)))));
						
						if(node.solidity != old_solidity)	// NOTE: not 100% thorough! nodes changes across chunk edges can affect this too!
							solidified = false;
					}
				}
			}
		}
	}




	void TerrainChunk::Vis(SceneRenderer *renderer, Mat4 main_xform)
	{
		Mat4 net_xform = main_xform * xform;

		AABB aabb = AABB(Vec3(), Vec3(1, 1, 1) * ChunkSize).GetTransformedAABB(net_xform);
		//if(renderer->camera->CheckSphereVisibility((aabb.min + aabb.max) * 0.5f, (aabb.max - aabb.min).ComputeMagnitude() * 0.5f))
		{
			if(!vbo_valid)
			{
				assert(vbos.empty());
				assert(depth_vbo == NULL);

				CreateVBOs(vbos, depth_vbo);
				vbo_valid = true;
			}

			if(depth_vbo != NULL)
				renderer->objects.push_back(RenderNode(material, new VoxelMaterialNodeData(vbos, depth_vbo, Vec3(float(chunk_x), float(chunk_y), float(chunk_z)) * ChunkSize, net_xform), 0));
		}
	}




	/*
	 * TerrainChunk::RelativeTerrainVertex members
	 */
	Vec3 TerrainChunk::RelativeTerrainVertex::GetPosition() { return vertex->pos + offset; }

	vector<TerrainChunk::RelativeTerrainVertex*> TerrainChunk::RelativeTerrainVertex::rel_vert_recycle_bin = vector<TerrainChunk::RelativeTerrainVertex*>();




	/*
	 * CreateVBO is long...
	 */
	void TerrainChunk::CreateVBOs(boost::unordered_map<unsigned char, VoxelMaterialVBO>& results, VertexBuffer*& depth_vbo)
	{
		assert(results.empty());
		assert(depth_vbo == NULL);

		int num_verts = 0;

		// dimensions including neighboring chunks (in order to compute normal vectors)
		int max_x = owner->GetXDim() == chunk_x + 1 ? ChunkSize : ChunkSize + 2;
		int max_y = owner->GetYDim() == chunk_y + 1 ? ChunkSize : ChunkSize + 2;
		int max_z = owner->GetZDim() == chunk_z + 1 ? ChunkSize : ChunkSize + 2;

		vector<RelativeTerrainVertex*> unique_vertices;
		unique_vertices.reserve(max_x * max_y * max_z * 3);

		// dimensions of the chunk itself
		int cmax_x = owner->GetXDim() == chunk_x + 1 ? ChunkSize - 2 : ChunkSize;
		int cmax_y = owner->GetYDim() == chunk_y + 1 ? ChunkSize - 2 : ChunkSize;
		int cmax_z = owner->GetZDim() == chunk_z + 1 ? ChunkSize - 2 : ChunkSize;

		int a_x_span = max_y * max_z;
		int c_x_span = cmax_y * cmax_z;
		vector<unsigned int>* vertex_indices = new vector<unsigned int>[c_x_span * cmax_x];

		for(int x = 0; x < max_x; ++x)
			for(int y = 0; y < max_y; ++y)
				for(int z = 0; z < max_z; ++z)
				{
					// find the verts for this one cube
					CubeTriangles* cube = GetCubeRelative(x - 1, y - 1, z - 1);

					if(cube != NULL)
					{
						cube->BuildAsNeeded();
						
						if(cube->cache != NULL)
						{
							for(char i = 0; i < 3; ++i)
							{
								if(cube->chunk == this)
									unique_vertices.push_back(RelativeTerrainVertex::New(&cube->cache->verts[i]));
								else
								{
									int dx, dy, dz;
									cube->chunk->GetChunkPosition(dx, dy, dz);
									dx -= chunk_x;
									dy -= chunk_y;
									dz -= chunk_z;

									unique_vertices.push_back(RelativeTerrainVertex::New(&cube->cache->verts[i], Vec3(float(dx * ChunkSize), float(dy * ChunkSize), float(dz * ChunkSize))));
								}
							}

							continue;
						}
					}

					for(char i = 0; i < 3; ++i)
						unique_vertices.push_back(NULL);
				}
		
		for(int x = 0; x < cmax_x; ++x)
			for(int y = 0; y < cmax_y; ++y)
				for(int z = 0; z < cmax_z; ++z)
				{
					CubeTriangles* cube = GetCubeRelative(x, y, z);

					if(cube != NULL)
					{
						char cube_vert_count = cube->num_vertices;
						assert(cube_vert_count >= 0 && cube_vert_count < 16);
						if(cube_vert_count == 0)
							continue;

						num_verts += cube_vert_count;

						char* cube_indices = &cube->cache->indices[0];

						vector<unsigned int> cube_vertex_indices;
						for(int i = 0; i < cube_vert_count; ++i)
						{
							char index = cube_indices[i];

							unsigned int global_index;

							unsigned int start = (x + 1) * a_x_span + (y + 1) * max_z + (z + 1);
							switch(index)
							{
								case 0: global_index =	(start								) * 3;		break;
								case 1: global_index =	(start							+ 1	) * 3 + 1;	break;
								case 2: global_index =	(start				+ max_z			) * 3;		break;
								case 3: global_index =	(start								) * 3 + 1;	break;
								case 4: global_index =	(start + a_x_span					) * 3;		break;
								case 5: global_index =	(start + a_x_span				+ 1	) * 3 + 1;	break;
								case 6: global_index =	(start + a_x_span	+ max_z			) * 3;		break;
								case 7: global_index =	(start + a_x_span					) * 3 + 1;	break;
								case 8: global_index =	(start								) * 3 + 2;	break;
								case 9: global_index =	(start							+ 1	) * 3 + 2;	break;
								case 10: global_index = (start				+ max_z		+ 1	) * 3 + 2;	break;
								case 11: global_index = (start				+ max_z			) * 3 + 2;	break;
							}

							assert(unique_vertices[global_index] != NULL);

							cube_vertex_indices.push_back(global_index);
						}

						vertex_indices[x * c_x_span + y * cmax_z + z] = cube_vertex_indices;
					}
				}

		if(num_verts == 0)
		{
			for(vector<RelativeTerrainVertex*>::iterator iter = unique_vertices.begin(); iter != unique_vertices.end(); ++iter)
				if(*iter != NULL)
					RelativeTerrainVertex::Delete(*iter);
			delete[] vertex_indices;

			return;
		}
		else
		{
			// go back through them and find the normal vectors
			for(int x = 0; x < cmax_x; ++x)
				for(int y = 0; y < cmax_y; ++y)
					for(int z = 0; z < cmax_z; ++z)
					{
						vector<unsigned int>& cube_verts = vertex_indices[x * c_x_span + y * cmax_z + z];

						if(cube_verts.empty())
							continue;

						// iterate through all of the verts
						for(vector<unsigned int>::iterator iter = cube_verts.begin(); iter != cube_verts.end(); )
						{
							RelativeTerrainVertex* tri_verts[3];

							bool need_normals = false;
							for(char i = 0; i < 3; ++i)
							{
								tri_verts[i] = unique_vertices[*(iter++)];
								if(!tri_verts[i]->vertex->normal_valid)
									need_normals = true;
							}
						
							if(need_normals)
							{
								Vec3 va = tri_verts[0]->GetPosition(), vb = tri_verts[1]->GetPosition(), vc = tri_verts[2]->GetPosition();

								Vec3 tri_normal = Vec3::Cross(vb - va, vc - va);

								float len_sq = tri_normal.ComputeMagnitudeSquared();
								if(len_sq > 0.0f)
								{
									//tri_normal /= sqrtf(len_sq);

									for(char i = 0; i < 3; ++i)
										if(!tri_verts[i]->vertex->normal_valid)
											tri_verts[i]->vertex->normal += tri_normal;
								}
							}
						}
					}

			depth_vbo = new VertexBuffer(Triangles);
			depth_vbo->AddAttribute("gl_Vertex", Float, 3);
			depth_vbo->SetNumVerts(num_verts);

			float* depth_vert_ptr = depth_vbo->GetFloatPointer("gl_Vertex");

			// now build the actual vbo with the values we computed
			for(int x = 0; x < cmax_x; ++x)
				for(int y = 0; y < cmax_y; ++y)
					for(int z = 0; z < cmax_z; ++z)
					{
						vector<unsigned int>& cube_verts = vertex_indices[x * c_x_span + y * cmax_z + z];

						if(cube_verts.empty())
							continue;

						// iterate through all of the verts in this cube
						for(vector<unsigned int>::iterator iter = cube_verts.begin(); iter != cube_verts.end();)
						{
							RelativeTerrainVertex* v1 = unique_vertices[*(iter++)];
							RelativeTerrainVertex* v2 = unique_vertices[*(iter++)];
							RelativeTerrainVertex* v3 = unique_vertices[*(iter++)];

							ProcessTriangle(v1, v2, v3, results, depth_vert_ptr, num_verts);
						}
					}

			for(boost::unordered_map<unsigned char, VoxelMaterialVBO>::iterator iter = results.begin(); iter != results.end(); ++iter)
				iter->second.vbo->BuildVBO();

			for(vector<RelativeTerrainVertex*>::iterator iter = unique_vertices.begin(); iter != unique_vertices.end(); ++iter)
				if(*iter != NULL)
					RelativeTerrainVertex::Delete(*iter);
			delete[] vertex_indices;
		}
	}

	static unsigned int PickTriangleMaterials(const MultiMaterial& m1, const MultiMaterial& m2, const MultiMaterial& m3, unsigned char* pick);
	void TerrainChunk::ProcessTriangle(RelativeTerrainVertex* v1, RelativeTerrainVertex* v2, RelativeTerrainVertex* v3, boost::unordered_map<unsigned char, VoxelMaterialVBO>& vbos, float*& depth_vert_ptr, unsigned int num_verts)
	{
		RelativeTerrainVertex* verts[] = { v1, v2, v3 };

		unsigned char use_materials[4];
		unsigned int use_count = PickTriangleMaterials(v1->vertex->material, v2->vertex->material, v3->vertex->material, use_materials);

		// for each vert, compute the inverse of the total weight of used materials
		float inv_totals[] = { 0.0f, 0.0f, 0.0f };

		for(unsigned int i = 0; i < use_count; ++i)
		{
			unsigned char mat = use_materials[i];
			for(int j = 0; j < 3; ++j)
				inv_totals[j] += verts[j]->vertex->material.GetMaterialAmount(mat);
		}
		for(unsigned int i = 0; i < 3; ++i)
		{
			assert(inv_totals[i] != 0.0f);
			inv_totals[i] = 1.0f / inv_totals[i];
		}

		for(unsigned int i = 0; i < use_count; ++i)
		{
			unsigned char mat = use_materials[i];

			// now find a compatible vbo
			VertexBuffer* target_vbo = GetOrCreateVBO(vbos, mat, num_verts);

			// finally put the verts into the vbo
			ProcessVert(*v1, target_vbo, mat, inv_totals[0]);
			ProcessVert(*v2, target_vbo, mat, inv_totals[1]);
			ProcessVert(*v3, target_vbo, mat, inv_totals[2]);
		}

		// add verts to depth vbo as well
		for(int i = 0; i < 3; ++i)
		{ 
			Vec3 xyz = verts[i]->GetPosition();
			*(depth_vert_ptr++) = xyz.x;
			*(depth_vert_ptr++) = xyz.y;
			*(depth_vert_ptr++) = xyz.z;
		}
		
	}

	VertexBuffer* TerrainChunk::GetOrCreateVBO(boost::unordered_map<unsigned char, VoxelMaterialVBO>& get_from, unsigned char material, unsigned int size_to_create)
	{
		boost::unordered_map<unsigned char, VoxelMaterialVBO>::iterator found = get_from.find(material);
		if(found != get_from.end())
		{
			VoxelMaterialVBO result = found->second;
			return result.vbo;
		}
		else
		{
			// there is no existing vbo for this material
			VertexBuffer* vbo = CreateVBO(size_to_create);
			VoxelMaterialVBO result = VoxelMaterialVBO(material, vbo);
			get_from[material] = result;

			return vbo;
		}
	}

	VertexBuffer* TerrainChunk::CreateVBO(unsigned int allocate_n)
	{
		VertexBuffer* vbo = new VertexBuffer(Triangles);

		vbo->AddAttribute("gl_Vertex",			Float, 3);
		vbo->AddAttribute("gl_Normal",			Float, 3);
		vbo->AddAttribute("material_weight",	Float, 1);

		vbo->SetAllocatedSize(allocate_n);
		vbo->SetNumVerts(0);

		return vbo;
	}

	void TerrainChunk::ProcessVert(RelativeTerrainVertex& vert, VertexBuffer* target_vbo, unsigned char material, float inv_total)
	{
		Vec3 pos = vert.GetPosition();

		Vec3 normal = vert.vertex->normal = Vec3::Normalize(vert.vertex->normal);
		vert.vertex->normal_valid = true;

		unsigned int num_verts = target_vbo->GetNumVerts();
		
		target_vbo->SetNumVerts(num_verts + 1);

		float* vert_ptr =	&target_vbo->GetFloatPointer("gl_Vertex")		[num_verts * 3];
		float* normal_ptr =	&target_vbo->GetFloatPointer("gl_Normal")		[num_verts * 3];
		float* mat_ptr =	&target_vbo->GetFloatPointer("material_weight")	[num_verts];

		*(vert_ptr++) =		pos.x;
		*(vert_ptr++) =		pos.y;
		*(vert_ptr++) =		pos.z;

		*(normal_ptr++) =	normal.x;
		*(normal_ptr++) =	normal.y;
		*(normal_ptr++) =	normal.z;

		*(mat_ptr++) =		(float)vert.vertex->material.GetMaterialAmount(material) * inv_total;
	}

	unsigned int PickTriangleMaterials(const MultiMaterial& m1, const MultiMaterial& m2, const MultiMaterial& m3, unsigned char* pick)
	{
		MultiMaterial mats[] = { m1, m2, m3 };

		unsigned char types[3][4];
		unsigned short weights[3][4];

		// put data into arrays
		for(char i = 0; i < 3; ++i)
		{
			unsigned char* my_types = &types[i][0];
			unsigned short* my_weights = &weights[i][0];

			const MultiMaterial& mat = mats[i];
			for(char j = 0; j < 4; ++j)
			{
				my_types[j] = mat.types[j];
				my_weights[j] = unsigned short(mat.weights[j]);
			}
		}

		// sort materials within each multimaterial
		for(char i = 0; i < 3; ++i)
		{
			unsigned char* my_types = &types[i][0];
			unsigned short* my_weights = &weights[i][0];

			// first combine duplicate materials and get rid of empty ones
			for(char j = 0; j < 4; ++j)
				if(unsigned char type = my_types[j])
				{
					if(my_weights[j] > 0)
						for(char k = j + 1; k < 4; ++k)
							if(my_types[k] == type)
							{
								my_weights[j] += my_weights[k];
								my_types[k] = 0;
							}
				}
				else
					my_weights[j] = 0;

			// now bubble sort!
			for(char j = 0; j < 4; ++j)
				for(char k = 0; k + j < 3; ++k)
					if(my_weights[k + 1] > my_weights[k])
					{
						swap(my_types[k], my_types[k + 1]);
						swap(my_weights[k], my_weights[k + 1]);
					}
		}

		// clear the output array
		for(char i = 0; i < 4; ++i)
			pick[i] = 0;

		unsigned char extra_types[9];
		unsigned short extra_weights[9];

		char used = 0, extras = 0;

		// make sure each vertex has its most prevalent material represented
		for(char i = 0; i < 3; ++i)
		{
			unsigned char* my_types = &types[i][0];
			unsigned short* my_weights = &weights[i][0];

			unsigned char type = my_types[0];
			assert(type != 0);
			
			char j;
			for(j = 0; j < used; ++j)
				if(pick[j] == type)
					break;

			if(j == used)
				pick[used++] = type;

			// put other types into extras list
			for(j = 1; j < 4; ++j)
				if(unsigned char extra_type = my_types[j])
				{
					char k;
					for(k = 0; k < extras; ++k)
						if(extra_types[k] == extra_type)
						{
							extra_weights[k] += my_weights[j];
							break;
						}

					if(k == extras)				// type not already present in extras list
					{
						extra_types[extras] = extra_type;
						extra_weights[extras] = my_weights[j];
						++extras;
					}
				}
		}

		// pick the highest-weighted of the extras
		while(used < 4 && extras)
		{
			unsigned char best_index = 0;
			unsigned short best_weight = extra_weights[0];

			for(char i = 1; i < extras; ++i)
				if(extra_weights[i] > best_weight)
				{
					best_index = i;
					best_weight = extra_weights[i];
				}

			if(best_weight)
			{
				pick[used++] = extra_types[best_index];

				// remove that item from the search pool without having to shift everything else over by one
				swap(extra_types[best_index], extra_types[extras - 1]);
				swap(extra_weights[best_index], extra_weights[extras - 1]);
				--extras;
			}
			else
				break;
		}

		return used;
	}

	

	bool TerrainChunk::IsEmpty()
	{
		for(vector<TerrainNode>::iterator iter = node_data.begin(); iter != node_data.end(); ++iter)
			if(iter->solidity != 0)
				return false;

		return true;
	}

	bool TerrainChunk::IsEntirelySolid()
	{
		for(vector<TerrainNode>::iterator iter = node_data.begin(); iter != node_data.end(); ++iter)
			if(iter->solidity != 255)
				return false;

		return true;
	}




	/*
	 * TerrainChunk I/O functions
	 */
	unsigned int TerrainChunk::Write(ostream& stream)
	{
		if(IsEmpty())
		{
			WriteByte(1, stream);
			return 0;
		}
		else
		{
			WriteByte(0, stream);

			for(vector<TerrainNode>::iterator iter = node_data.begin(); iter != node_data.end(); ++iter)
				if(unsigned int node_write_error = iter->Write(stream))
					return node_write_error;

			return 0;
		}
	}

	unsigned int TerrainChunk::Read(istream& stream)
	{
		int emptiness_code = ReadByte(stream);

		if(emptiness_code == 1)
		{
			for(vector<TerrainNode>::iterator iter = node_data.begin(); iter != node_data.end(); ++iter)
				iter->solidity = 0;
		}
		else
		{
			for(vector<TerrainNode>::iterator iter = node_data.begin(); iter != node_data.end(); ++iter)
				if(unsigned int node_read_error = iter->Read(stream))
					return node_read_error;
		}
		return 0;
	}
}
