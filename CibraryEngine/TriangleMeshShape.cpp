#include "StdAfx.h"
#include "TriangleMeshShape.h"

#include "Physics.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "VertexBuffer.h"
#include "DebugDrawMaterial.h"

#include "Serialize.h"
#include "Util.h"

//#define DEBUG_OCTREE_EFFICIENCY

#ifdef DEBUG_OCTREE_EFFICIENCY
	#include "Texture2D.h"
	#include "ImageIO.h"
#endif
	
namespace CibraryEngine
{
	/*
	 * TriangleMeshShape methods
	 */
	TriangleMeshShape::TriangleMeshShape() : CollisionShape(ST_TriangleMesh), octree(NULL), vertices(), triangles() { InitCache(); }

	TriangleMeshShape::TriangleMeshShape(VertexBuffer* vbo) : CollisionShape(ST_TriangleMesh), octree(NULL), vertices(), triangles()
	{
		InitCache();

		if(vbo)
		{
			if(unsigned int num_verts = vbo->GetNumVerts())
			{
				if(float* pos_ptr = vbo->GetFloatPointer("gl_Vertex"))
				{
					bool skip_fourth = vbo->GetAttribute("gl_Vertex").n_per_vertex == 4;

					// put all the verts in a vector temporarily (and find the aabb while we're at it)
					vector<Vec3> verts;
					verts.resize(num_verts);

					AABB aabb;
					for(unsigned int i = 0; i < num_verts;)
					{	
						for(char j = 0; j < 3; ++i, ++j)
						{
							float x = *(pos_ptr++);
							float y = *(pos_ptr++);
							float z = *(pos_ptr++);

							if(skip_fourth)
								++pos_ptr;

							Vec3 vec(x, y, z);
							verts[i] = vec;

							if(i == 0)
								aabb = AABB(vec);
							else
								aabb.Expand(vec);
						}
					}

					// now put that vector's data into a tree
					struct VertStruct
					{
						Vec3 pos;
						unsigned int index;

						VertStruct(Vec3 pos, unsigned int index) : pos(pos), index(index)  { }
					};

					Octree<vector<VertStruct> > tree(aabb.min, aabb.max);

					for(vector<Vec3>::iterator iter = verts.begin(); iter != verts.end();)
					{
						Tri tri;
						for(char j = 0; j < 3; ++iter, ++j)
						{
							Vec3& vert = *iter;

							struct Action
							{
								Vec3 vert;
								vector<Vec3>& vertices;
								unsigned int index;
								bool done;

								Action(Vec3 vert, vector<Vec3>& vertices) : vert(vert), vertices(vertices), done(false) { }

								void operator()(Octree<vector<VertStruct> >* node)
								{
									if(!done && node->bounds.ContainsPoint(vert))		// vert relevant to this node?
									{
										if(node->IsLeaf())
										{
											// look for the node in the leaf's vertex list
											for(vector<VertStruct>::iterator jter = node->contents.begin(); jter != node->contents.end(); ++jter)
												if((jter->pos - vert).ComputeMagnitudeSquared() < 0.000000001f)
												{
													index = jter->index;
													done = true;
													break;
												}

											if(!done)									// first occurrence of vertex, add it to the node
											{
												index = vertices.size();

												vertices.push_back(vert);
												node->contents.push_back(VertStruct(vert, index));

												if(node->contents.size() >= 20)			// node too big? split into octants
												{
													node->Split(1);
													for(vector<VertStruct>::iterator jter = node->contents.begin(); jter != node->contents.end(); ++jter)
													{							
														Action adder(jter->pos, vertices);
														adder(node);
													}
												}

												done = true;
											}
										}
										else
											node->ForEach(*this);
									}
								}

							} action(vert, vertices);
							action(&tree);
							tri.indices[j] = action.index;
						}

						triangles.push_back(tri);
					}
				}
			}
		}
	}

	void TriangleMeshShape::InitCache()
	{
		built = false;
		cache = NULL;
	}
	
	void TriangleMeshShape::DeleteCache()
	{
		delete[] cache;

		InitCache();
	}

#ifdef DEBUG_OCTREE_EFFICIENCY
	static vector<unsigned char> efficiency_tex;

	#define EFF_TEX_W 128
	#define EFF_TEX_H 256

#endif

	void TriangleMeshShape::InnerDispose()
	{
#ifdef DEBUG_OCTREE_EFFICIENCY
		unsigned char* color_ptr = &efficiency_tex[0];
		for(unsigned int y = 0; y < EFF_TEX_H; ++y)
			for(unsigned int x = 0; x < EFF_TEX_W; ++x)
			{
				unsigned char* temp = color_ptr;

				float f = 0;

				for(unsigned char i = 0; i < 4; ++i)
					f += float(*(color_ptr++) * (0x1 << (8 * i)));
				
				f /= f + 1.0f;

				float f_sq = f * f;
				float g = f * 255.0f;

				*(temp++) = (unsigned char)min(255.0f, g);
				g *= f_sq;
				*(temp++) = (unsigned char)min(255.0f, g);
				g *= f_sq;
				*(temp++) = (unsigned char)min(255.0f, g);
				*temp = 255;
			}
		ImageIO::SaveTGA("octree_efficiency.tga", efficiency_tex, EFF_TEX_W, EFF_TEX_H);
#endif
		DeleteCache();
	}

	void TriangleMeshShape::BuildCache()
	{
		DeleteCache();

		size_t num_faces = triangles.size();

		cache = new TriCache[num_faces];

		for(size_t face_index = 0; face_index < num_faces; ++face_index)
		{
			Tri& tri = triangles[face_index];
			TriCache& tri_cache = cache[face_index];

			tri_cache.a = vertices[tri.indices[0]];
			tri_cache.b = vertices[tri.indices[1]];
			tri_cache.c = vertices[tri.indices[2]];
			tri_cache.ab = tri_cache.b - tri_cache.a;
			tri_cache.ac = tri_cache.c - tri_cache.a;

			Vec3 normal = Vec3::Normalize(Vec3::Cross(tri_cache.ab, tri_cache.ac));
			float offset = Vec3::Dot(normal, tri_cache.a);

			tri_cache.plane = Plane(normal, offset);

			tri_cache.p = Vec3::Cross(tri_cache.ac, normal);
			tri_cache.p /= Vec3::Dot(tri_cache.p, tri_cache.ab);
			tri_cache.q = Vec3::Cross(tri_cache.ab, normal);
			tri_cache.q /= Vec3::Dot(tri_cache.q, tri_cache.ac);
			tri_cache.u_offset = Vec3::Dot(tri_cache.p, tri_cache.a);
			tri_cache.v_offset = Vec3::Dot(tri_cache.q, tri_cache.a);

			tri_cache.bc = tri_cache.c - tri_cache.b;

			tri_cache.inv_len_ab = 1.0f / tri_cache.ab.ComputeMagnitude();
			tri_cache.inv_len_bc = 1.0f / tri_cache.bc.ComputeMagnitude();
			tri_cache.inv_len_ca = 1.0f / tri_cache.ac.ComputeMagnitude();

			tri_cache.n_ab = Vec3::Cross(tri_cache.ab, normal) * tri_cache.inv_len_ab;
			tri_cache.n_bc = Vec3::Cross(tri_cache.bc, normal) * tri_cache.inv_len_bc;
			tri_cache.n_ca = Vec3::Cross(-tri_cache.ac, normal) * tri_cache.inv_len_ca;
			
			tri_cache.dot_n_ab_a = Vec3::Dot(tri_cache.n_ab, tri_cache.a);
			tri_cache.dot_n_bc_b = Vec3::Dot(tri_cache.n_bc, tri_cache.b);
			tri_cache.dot_n_ca_c = Vec3::Dot(tri_cache.n_ca, tri_cache.c);
		}

		built = true;
	}

	void TriangleMeshShape::BuildOctree()
	{
		if(octree != NULL)
		{
			delete octree;
			octree = NULL;
		}

		if(vertices.empty())
			return;					// lolwut?

		if(!built)
			BuildCache();
		
		// find the bounds of the octree
		AABB aabb;
		bool first = true;
		for(vector<Vec3>::iterator iter = vertices.begin(); iter != vertices.end(); ++iter)
		{
			if(first)
			{
				aabb = AABB(*iter);
				first = false;
			}
			else
				aabb.Expand(*iter);
		}

		// now create the octree
		octree = new Octree<NodeData>(aabb.min, aabb.max);

		// functor(?) to place a triangle into the octree
		struct Action
		{
			NodeData::Tri tri;

			Action(NodeData::Tri tri) : tri(tri) { }

			void operator()(Octree<NodeData>* node)
			{
				if(AABB::IntersectTest(node->bounds, tri.aabb))
				{
					if(node->IsLeaf())
					{
						node->contents.Add(tri);

						if(node->contents.tri_count >= 2 && (node->parent == NULL || node->parent->contents.tri_count >= node->contents.tri_count + 1))			// node is too crowded (and subdivision seems to be working), therefore subdivide it
						{
							node->Split(1);
							for(vector<NodeData::Tri>::iterator iter = node->contents.triangles.begin(); iter != node->contents.triangles.end(); ++iter)
							{
								Action adder(*iter);
								node->ForEach(adder);
							}
							node->contents.triangles = vector<NodeData::Tri>();
						}
					}
					else
					{
						++node->contents.tri_count;
						node->ForEach(*this);
					}
				}
			}
		};

		// insert all the triangles into the octree
		TriCache* tri_ptr = cache;
		unsigned int count = triangles.size();
		for(unsigned int i = 0; i < count; ++i, ++tri_ptr)
		{
			TriCache& tri = *tri_ptr;

			AABB aabb(tri.a);
			aabb.Expand(tri.b);
			aabb.Expand(tri.c);

			Action action(NodeData::Tri(i, aabb));
			action(octree);
		}
	}

	void TriangleMeshShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
	{
		Mat3 rm(ori.ToMat3());

		vector<Vec3> transformed;
		for(vector<Vec3>::iterator iter = vertices.begin(); iter != vertices.end(); ++iter)
			transformed.push_back(rm * *iter + pos);

		Material* mat = DebugDrawMaterial::GetDebugDrawMaterial();

		for(vector<Tri>::iterator iter = triangles.begin(); iter != triangles.end(); ++iter)
		{
			Vec3& a = transformed[iter->indices[0]];
			Vec3& b = transformed[iter->indices[1]];
			Vec3& c = transformed[iter->indices[2]];

			renderer->objects.push_back(RenderNode(mat, new DebugDrawMaterialNodeData(a, b), 1.0f));
			renderer->objects.push_back(RenderNode(mat, new DebugDrawMaterialNodeData(b, c), 1.0f));
			renderer->objects.push_back(RenderNode(mat, new DebugDrawMaterialNodeData(c, a), 1.0f));
		}
	}

	vector<unsigned int> TriangleMeshShape::GetRelevantTriangles(const AABB& aabb)
	{
		if(triangles.empty())
			return vector<unsigned int>();						// lolwut?

		if(octree == NULL)
			BuildOctree();										// this will in turn call BuildCache if necessary

		// functor(?) to get a list of relevant triangles
		struct Action
		{
#ifdef DEBUG_OCTREE_EFFICIENCY
			unsigned int ops;
#endif

			boost::unordered_set<unsigned int> relevant_list;	// list of unique maybe-relevant triangles encountered

			const AABB& aabb;									// temporal aabb for the ray

			Action(const AABB& aabb) :
#ifdef DEBUG_OCTREE_EFFICIENCY
				ops(0),
#endif
				relevant_list(),
				aabb(aabb)
			{
			}

			void operator() (Octree<NodeData>* node)
			{
#ifdef DEBUG_OCTREE_EFFICIENCY
				++ops;
#endif
				if(AABB::IntersectTest(node->bounds, aabb))
				{
					if(node->IsLeaf())
					{
						for(vector<NodeData::Tri>::iterator iter = node->contents.triangles.begin(); iter != node->contents.triangles.end(); ++iter)
							if(AABB::IntersectTest(iter->aabb, aabb))
								relevant_list.insert(iter->face_index);
					}
					else
						node->ForEach(*this);
				}
			}
		} action(aabb);

		action(octree);

		vector<unsigned int> results;

#ifdef DEBUG_OCTREE_EFFICIENCY

		unsigned int x = (action.ops - 1) / 8, y = action.relevant_list.size();
		if(efficiency_tex.empty())
			efficiency_tex.resize(EFF_TEX_W * EFF_TEX_H * 4);

		if(x < EFF_TEX_W && y < EFF_TEX_H)
		{
			unsigned char* color_ptr = &efficiency_tex[(y * EFF_TEX_W + x) * 4];

			unsigned char* next = &color_ptr[4];
			do { ++*color_ptr; } while(*(color_ptr++) == 0 && color_ptr != next);
		}

#endif

		for(boost::unordered_set<unsigned int>::iterator iter = action.relevant_list.begin(); iter != action.relevant_list.end(); ++iter)
			results.push_back(*iter);

		return results;
	}

	vector<Intersection> TriangleMeshShape::RayTest(const Ray& ray)
	{
		AABB aabb(ray.origin);
		aabb.Expand(ray.origin + ray.direction);

		vector<unsigned int> relevant_triangles = GetRelevantTriangles(aabb);

		vector<Intersection> test;
		for(vector<unsigned int>::iterator iter = relevant_triangles.begin(); iter != relevant_triangles.end(); ++iter)
		{
			unsigned int face_index = *iter;

			Intersection intersection;
			if(cache[face_index].RayTest(ray, face_index, intersection))
				test.push_back(intersection);			
		}

		return test;
	}



	bool TriangleMeshShape::TriCache::RayTest(const Ray& ray, unsigned int index, Intersection& intersection)
	{
		float hit = Util::RayPlaneIntersect(ray, plane);

		if(hit > 0 || hit <= 0)						// for now just check that it's a real number
		{
			Vec3 pos = ray.origin + ray.direction * hit;
			float u = Vec3::Dot(p, pos) - u_offset;
			float v = Vec3::Dot(q, pos) - v_offset;
			if(u >= 0 && v >= 0 && u + v <= 1)
			{
				intersection.face = index;
				intersection.i = u;
				intersection.j = v;
				intersection.position = pos;
				intersection.time = hit;
				intersection.normal = plane.normal;
				intersection.ray = ray;

				return true;
			}
		}
		return false;
	}

	float TriangleMeshShape::TriCache::DistanceToPoint(const Vec3& x)
	{
		Vec3 ax = x - a, bx = x - b, cx = x - c;					// vectors from verts to X
		const Vec3& normal = plane.normal;

		// squares of distances from verts; these distances guaranteed to be valid...
		// we will take the square root of the minimum instead of the minimum of the square roots!
		float dist_a_x_sq = ax.ComputeMagnitudeSquared();
		float dist_b_x_sq = bx.ComputeMagnitudeSquared();
		float dist_c_x_sq = cx.ComputeMagnitudeSquared();

		// minimum value so far encountered...
		float min_d = sqrtf(min(dist_a_x_sq, min(dist_b_x_sq, dist_c_x_sq)));		

		// finding the positions of X along each of the edges (necessary to determine if the edge distances are valid)
		float part_x_ab = Vec3::Dot(ax, ab) * inv_len_ab * inv_len_ab;
		float part_x_bc = Vec3::Dot(bx, bc) * inv_len_bc * inv_len_bc;
		float part_x_ca = Vec3::Dot(cx, -ac) * inv_len_ca * inv_len_ca;

		// determining whether or not the edge distances are valid
		if (part_x_ab >= 0 && part_x_ab <= 1)
			min_d = min(min_d, Vec3::Cross(ab, ax).ComputeMagnitude() * inv_len_ab);
		if (part_x_bc >= 0 && part_x_bc <= 1)
			min_d = min(min_d, Vec3::Cross(bc, bx).ComputeMagnitude() * inv_len_bc);
		if (part_x_ca >= 0 && part_x_ca <= 1)
			min_d = min(min_d, Vec3::Cross(-ac, cx).ComputeMagnitude() * inv_len_ca);

		// finding the distance from the plane; valid under the least frequently satisfied conditions
		if ((Vec3::Dot(n_ab, x) - dot_n_ab_a) * (Vec3::Dot(n_ab, c) - dot_n_ab_a) > 0)					// if they're on the same side, this product is positive
			if ((Vec3::Dot(n_bc, x) - dot_n_bc_b) * (Vec3::Dot(n_bc, a) - dot_n_bc_b) > 0)
				if ((Vec3::Dot(n_ca, x) - dot_n_ca_c) * (Vec3::Dot(n_ca, b) - dot_n_ca_c) > 0)
					min_d = min(min_d, fabs(Vec3::Dot(normal, ax)));									// too bad it's so much harder to find out if it's valid than it is to calculate the value itself

		return min_d;
	}



	unsigned int TriangleMeshShape::Read(istream& stream)
	{
		unsigned int num_vertices = ReadUInt32(stream);
		vertices.resize(num_vertices);
		for(unsigned int i = 0; i < num_vertices; ++i)
			vertices[i] = ReadVec3(stream);

		unsigned int num_triangles = ReadUInt32(stream);
		triangles.resize(num_triangles);
		for(unsigned int i = 0; i < num_triangles; ++i)
		{
			Tri tri;
			tri.indices[0] = ReadUInt32(stream);
			tri.indices[1] = ReadUInt32(stream);
			tri.indices[2] = ReadUInt32(stream);
			triangles[i] = tri;
		}

		return stream.fail() ? 1 : 0;
	}

	void TriangleMeshShape::Write(ostream& stream)
	{
		WriteUInt32(vertices.size(), stream);
		for(vector<Vec3>::iterator iter = vertices.begin(); iter != vertices.end(); ++iter)
			WriteVec3(*iter, stream);

		WriteUInt32(triangles.size(), stream);
		for(vector<Tri>::iterator iter = triangles.begin(); iter != triangles.end(); ++iter)
		{
			Tri& tri = *iter;
			WriteUInt32(tri.indices[0], stream);
			WriteUInt32(tri.indices[1], stream);
			WriteUInt32(tri.indices[2], stream);
		}
	}
}
