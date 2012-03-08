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

namespace CibraryEngine
{
	/*
	 * TriangleMeshShape methods
	 */
	TriangleMeshShape::TriangleMeshShape() : CollisionShape(ST_TriangleMesh), vertices(), triangles() { InitCache(); }

	TriangleMeshShape::TriangleMeshShape(VertexBuffer* vbo) : CollisionShape(ST_TriangleMesh), vertices(), triangles()
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

		A = B = C = AB = AC = P = Q = NULL;
		planes = NULL;
		UOffset = VOffset = NULL;
	}
	
	void TriangleMeshShape::DeleteCache()
	{
		delete[] A;
		delete[] B;
		delete[] C;
		delete[] AB;
		delete[] AC;
		delete[] planes;
		delete[] P;
		delete[] Q;
		delete[] UOffset;
		delete[] VOffset;

		InitCache();
	}

	void TriangleMeshShape::InnerDispose() { DeleteCache(); }

	void TriangleMeshShape::BuildCache()
	{
		DeleteCache();

		size_t num_faces = triangles.size();

		A = new Vec3[num_faces];
		B = new Vec3[num_faces];
		C = new Vec3[num_faces];
		AB = new Vec3[num_faces];
		AC = new Vec3[num_faces];
		planes = new Plane[num_faces];
		P = new Vec3[num_faces];
		Q = new Vec3[num_faces];
		UOffset = new float[num_faces];
		VOffset = new float[num_faces];

		for(size_t face_index = 0; face_index < num_faces; ++face_index)
		{
			Tri& tri = triangles[face_index];

			A[face_index] = vertices[tri.indices[0]];
			B[face_index] = vertices[tri.indices[1]];
			C[face_index] = vertices[tri.indices[2]];
			AB[face_index] = B[face_index] - A[face_index];
			AC[face_index] = C[face_index] - A[face_index];

			Vec3 normal = Vec3::Normalize(Vec3::Cross(AB[face_index], AC[face_index]));
			float offset = Vec3::Dot(normal, A[face_index]);

			planes[face_index] = Plane(normal, offset);

			P[face_index] = Vec3::Cross(AC[face_index], normal);
			P[face_index] = P[face_index] / Vec3::Dot(P[face_index], AB[face_index]);
			Q[face_index] = Vec3::Cross(AB[face_index], normal);
			Q[face_index] = Q[face_index] / Vec3::Dot(Q[face_index], AC[face_index]);
			UOffset[face_index] = Vec3::Dot(P[face_index], A[face_index]);
			VOffset[face_index] = Vec3::Dot(Q[face_index], A[face_index]);
		}

		built = true;
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

	vector<Intersection> TriangleMeshShape::RayTest(const Ray& ray)
	{
		if(!built)
			BuildCache();

		size_t num_faces = triangles.size();

		vector<Intersection> test;
		for(size_t face_index = 0; face_index < num_faces; ++face_index)
		{
			Plane& plane = planes[face_index];
			float hit = Util::RayPlaneIntersect(ray, plane);

			// TODO: maybe put (optional) conditions here to cull 'backward' items, etc.
			if(hit > 0 || hit <= 0)						// for now just check that it's a real number
			{
				Vec3 pos = ray.origin + ray.direction * hit;
				float u = Vec3::Dot(P[face_index], pos) - UOffset[face_index];
				float v = Vec3::Dot(Q[face_index], pos) - VOffset[face_index];
				if(u >= 0 && v >= 0 && u + v <= 1)
				{
					Intersection intersection;
					intersection.face = face_index;
					intersection.i = u;
					intersection.j = v;
					intersection.position = pos;
					intersection.time = hit;
					intersection.normal = plane.normal;
					intersection.ray = ray;
					test.push_back(intersection);
				}
			}
		}

		return test;
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
