#include "StdAfx.h"
#include "ConvexMeshShape.h"

#include "MassInfo.h"
#include "AABB.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "VertexBuffer.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * ConvexMeshShape private implementation struct
	 */
	struct ConvexMeshShape::Imp
	{
		struct Vertex
		{
			Vec3 pos;

			Vertex(const Vec3& pos) : pos(pos) { }
		};

		struct Edge
		{
			unsigned int v1, v2;		// indices into verts array

			Edge(unsigned int v1, unsigned int v2) : v1(v1), v2(v2) { }
		};

		struct Face
		{
			Plane plane;
			vector<unsigned int> verts;
			vector<Plane> boundary_planes;

			Face(const Plane& plane) : plane(plane), verts(), boundary_planes() { }
		};

		struct ConvexPoly
		{
			struct PolyData
			{
				unsigned int index;
				Vec2 pos;

				Vec2 normal;
				float offset;

				PolyData *next, *prev;

			} data[32], *start, *unused;

			unsigned int max_count, count;

			Plane plane;
			Vec3 x_axis, y_axis;

			ConvexPoly(const Plane& plane) : start(NULL), unused(&data[0]), max_count(32), count(0), plane(plane)
			{
				float x = fabs(plane.normal.x), y = fabs(plane.normal.y), z = fabs(plane.normal.z);
				if(x <= y && x <= z)
					x_axis = Vec3::Normalize(Vec3::Cross(Vec3(1, 0, 0), plane.normal));
				else if(y <= x && y <= z)
					x_axis = Vec3::Normalize(Vec3::Cross(Vec3(0, 1, 0), plane.normal));
				else
					x_axis = Vec3::Normalize(Vec3::Cross(Vec3(0, 0, 1), plane.normal));
				y_axis = Vec3::Cross(plane.normal, x_axis);

				for(unsigned int i = 1; i < max_count; ++i)
					data[i - 1].next = &data[i];
				data[max_count - 1].next = NULL;
			}

			void AddVert(const Vec3& vertex, unsigned int index)
			{
				assert(unused != NULL);

				if(PolyData* iter = start)
				{
					do
					{
						if(iter->index == index)
							return;
						iter = iter->next;

					} while(iter != start);
				}

				Vec3 in_plane = vertex - plane.normal * (Vec3::Dot(plane.normal, vertex) - plane.offset);

				PolyData* noob = unused;
				unused = unused->next;

				noob->index = index;

				Vec2& pos = noob->pos;
				pos.x = Vec3::Dot(in_plane, x_axis);
				pos.y = Vec3::Dot(in_plane, y_axis);

				switch(count)
				{
					case 0:

						start = noob->next = noob->prev = noob;

						++count;

						break;

					case 1:

						start->normal = Vec2::Normalize(start->pos.y - noob->pos.y, noob->pos.x - start->pos.x);
						start->offset = Vec2::Dot(start->normal, start->pos);

						noob->normal = -start->normal;
						noob->offset = -start->offset;

						start->next = start->prev = noob;
						noob->next = noob->prev = start;

						++count;

						break;

					default:
					{
						static const float offset_threshold = 0.000001f;
						static const float colinearity_threshold = 0.000001f;

						PolyData* iter = start;
						do
						{
							if(Vec2::Dot(iter->normal, noob->pos) > iter->offset + offset_threshold)
							{
								PolyData *first = iter, *last = iter;

								if(iter == start)
									while(Vec2::Dot(first->prev->normal, noob->pos) > first->prev->offset - offset_threshold)
										first = first->prev;

								while(Vec2::Dot(last->next->normal, noob->pos) > last->next->offset - offset_threshold)
									last = last->next;

								PolyData* temp = last->next;
								last->next = unused;
								unused = first->next;
								last = temp;

								first->next = last->prev = noob;
								noob->prev = first;
								noob->next = last;

								first->normal = Vec2::Normalize(first->pos.y - noob->pos.y, noob->pos.x - first->pos.x);
								first->offset = Vec2::Dot(first->normal, first->pos);

								noob->normal = Vec2::Normalize(noob->pos.y - last->pos.y, last->pos.x - noob->pos.x);
								noob->offset = Vec2::Dot(noob->normal, noob->pos);

								start = noob;

								count = 0;			// indicates the point was successfully added

								break;
							}

							iter = iter->next;
						} while(iter != start);

						if(count == 0)
						{
							// point was successfully added; recompute count and eliminate any unnecessary colinear verts along the way
							iter = start;
							do
							{
								if((iter->normal - iter->prev->normal).ComputeMagnitudeSquared() < colinearity_threshold && iter->offset - iter->prev->offset < colinearity_threshold)
								{
									iter->prev->next = iter->next;
									iter->next->prev = iter->prev;

									PolyData* temp = iter->next;
									if(iter == start)
										start = temp;

									iter->next = unused;
									unused = iter;

									iter = temp;
								}
								else
								{
									++count;
									iter = iter->next;
								}
							} while(iter != start);
						}
						else
						{
							noob->next = unused;
							unused = noob;
						}

						break;
					}
				}
			}
		};

		vector<Vertex> verts;
		vector<Edge> edges;
		vector<Face> faces;

		AABB aabb;

		Imp() : verts(), edges(), faces(), aabb() { }
		Imp(Vec3* verts, unsigned int count) { Init(verts, count); }
		~Imp() { }

		void Init(Vec3* in_verts, unsigned int count)
		{
			verts = vector<Vertex>();
			edges = vector<Edge>();
			faces = vector<Face>();

			if(count > 0)
			{
				static const float xprod_discard_threshold = 0.0000001f;
				static const float plane_discard_threshold = 0.0001f;
				static const float plane_merge_threshold = 0.00001f;

				// determine aabb
				for(unsigned int i = 0; i < count; ++i)
					if(i == 0)
						aabb = AABB(in_verts[i]);
					else
						aabb.Expand(in_verts[i]);

				// generate unique exterior planes
				vector<ConvexPoly*> planes;
				for(unsigned int i = 2; i < count; ++i)
					for(unsigned int j = 1; j < i; ++j)
						for(unsigned int k = 0; k < j; ++k)
						{
							Vec3 normal = Vec3::Cross(in_verts[j] - in_verts[i], in_verts[k] - in_verts[i]);

							float mag = normal.ComputeMagnitude();
							if(mag > xprod_discard_threshold)
							{
								normal /= mag;
								float offset = Vec3::Dot(normal, in_verts[i]);

								Plane test_planes[2] = { Plane(normal, offset), Plane(-normal, -offset) };
								for(unsigned int m = 0; m < 2; ++m)
								{
									Plane& plane = test_planes[m];

									bool ok = true;
									for(unsigned int n = 0; n < count; ++n)
										if(plane.PointDistance(in_verts[n]) > plane_discard_threshold)
										{
											ok = false;
											break;
										}

									if(ok)
									{								
										for(vector<ConvexPoly*>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
										{
											ConvexPoly* poly = *iter;
											if((plane.normal - poly->plane.normal).ComputeMagnitudeSquared() < plane_merge_threshold && fabs(plane.offset - poly->plane.offset) < plane_merge_threshold)
											{
												poly->AddVert(in_verts[i], i);
												poly->AddVert(in_verts[j], j);
												poly->AddVert(in_verts[k], k);

												ok = false;
												break;
											}
										}

										if(ok)
										{
											ConvexPoly* poly = new ConvexPoly(plane);
											poly->AddVert(in_verts[i], i);
											poly->AddVert(in_verts[j], j);
											poly->AddVert(in_verts[k], k);

											planes.push_back(poly);
										}
									}
								}
							}
						}

				// produce final vert, edge, and face data
				vector<unsigned int> index_map(count);
				memset(index_map.data(), 0, count * sizeof(unsigned int));

				for(vector<ConvexPoly*>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
				{
					ConvexPoly* poly = *iter;
					if(poly->count <= 2)
					{
						DEBUG();

						delete poly;
						continue;
					}

					Face face(poly->plane);

					ConvexPoly::PolyData *start = poly->start, *jter = start;
					do
					{
						assert(jter >= poly->data && jter < poly->data + sizeof(poly->data) / sizeof(ConvexPoly::PolyData));

						// create unique vertices
						unsigned int index = jter->index, mapped_index = index_map[index];
						if(mapped_index == 0)
						{
							mapped_index = index_map[index] = verts.size() + 1;
							verts.push_back(in_verts[index]);
						}
						--mapped_index;

						unsigned int next = jter->next->index, mapped_next = index_map[next];
						if(mapped_next == 0)
						{
							mapped_next = index_map[next] = verts.size() + 1;
							verts.push_back(in_verts[next]);
						}
						--mapped_next;

						// create unique edges
						bool ok = true;
						for(vector<Edge>::iterator kter = edges.begin(); kter != edges.end(); ++kter)
							if(kter->v1 == mapped_index && kter->v2 == mapped_next || kter->v1 == mapped_next && kter->v2 == mapped_index)
							{
								ok = false;
								break;
							}

						if(ok)
							edges.push_back(Edge(mapped_index, mapped_next));

						face.verts.push_back(mapped_index);
						face.boundary_planes.push_back(Plane::FromPositionNormal(in_verts[index], Vec3::Cross(poly->plane.normal, jter->normal.x * poly->x_axis + jter->normal.y * poly->y_axis)));

						jter = jter->next;

					} while(jter != start);

					faces.push_back(face);

					delete poly;
				}

				OutputParts();
			}
			else
				aabb = AABB();
		}

		void OutputParts()
		{
			Debug(((stringstream&)(stringstream() << "Shape contains " << verts.size() << " verts, " << edges.size() << " edges, and " << faces.size() << " faces" << endl)).str());

			Debug("Verts:\n");
			for(unsigned int i = 0; i < verts.size(); ++i)
			{
				Vec3& pos = verts[i].pos;
				Debug(((stringstream&)(stringstream() << "\tVertex " << i << " =\t(" << pos.x << ", " << pos.y << ", " << pos.z << ")" << endl)).str());
			}

			Debug("Edges:\n");
			for(unsigned int i = 0; i < edges.size(); ++i)
			{
				unsigned int v1 = edges[i].v1, v2 = edges[i].v2;
				const Vec3& a = verts[v1].pos;
				const Vec3& b = verts[v2].pos;

				Debug(((stringstream&)(stringstream() << "\tEdge " << i << " is between vertex " << v1 << " and vertex " << v2 << endl)).str());
				Debug(((stringstream&)(stringstream() << "\t\tVertex " << v1 << " =\t(" << a.x << ", " << a.y << ", " << a.z << ")" << endl)).str());
				Debug(((stringstream&)(stringstream() << "\t\tVertex " << v2 << " =\t(" << b.x << ", " << b.y << ", " << b.z << ")" << endl)).str());
			}

			
			Debug("Faces:\n");
			for(unsigned int i = 0; i < faces.size(); ++i)
			{
				Face& face = faces[i];
				Vec3& normal = face.plane.normal;

				Debug(((stringstream&)(stringstream() << "\tFace " << i << " has normal vector (" << normal.x << ", " << normal.y << ", " << normal.z << ") and offset = " << face.plane.offset << "; it contains " << face.verts.size() << " vertices:" << endl)).str());
				for(unsigned int j = 0; j < face.verts.size(); ++j)
				{
					unsigned int index = face.verts[j];
					const Vec3& vert = verts[index].pos;
					Debug(((stringstream&)(stringstream() << "\t\tVertex " << index << " =\t(" << vert.x << ", " << vert.y << ", " << vert.z << ")" << endl)).str());
				}
			}
		}


		// misc. utility stuff
		void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori) { }		// TODO: implement this

		AABB GetTransformedAABB(const Mat4& xform)
		{
			// this will produce a tighter fitting AABB than aabb.GetTransformedAABB(xform), but it may be slower (especially if there are more than 8 vertices!)
			vector<Vertex>::const_iterator iter = verts.begin(), verts_end = verts.end();

			AABB xformed_aabb(iter->pos);
			for(++iter; iter != verts_end; ++iter)
				xformed_aabb.Expand(xform.TransformVec3_1(iter->pos));

			return xformed_aabb;
		}

		AABB ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache)
		{
			ConvexMeshShapeInstanceCache* cmsic = (ConvexMeshShapeInstanceCache*)cache;
			if(!cmsic)
				cache = cmsic = new ConvexMeshShapeInstanceCache();

			cmsic->Update(xform, this);

			return cmsic->aabb;
		}

		MassInfo ComputeMassInfo()
		{
			if(aabb.IsDegenerate())
				return MassInfo();
			else
			{
				// approximate the ConvexMeshShape as a box (same as how Bullet does it for its MultiSphereShape type)
				MassInfo temp;

				Vec3 dim = aabb.max - aabb.min;

				temp.mass = dim.x * dim.y * dim.z;							// assumes density = 1
				temp.com = (aabb.min + aabb.max) * 0.5f;

				float coeff = temp.mass / 12.0f;
				temp.moi[0] = coeff * (dim.y * dim.y + dim.z * dim.z);
				temp.moi[4] = coeff * (dim.x * dim.x + dim.z * dim.z);
				temp.moi[8] = coeff * (dim.x * dim.x + dim.y * dim.y);

				return temp;
			}
		}


		// i/o functions
		void Write(ostream& stream)
		{
			WriteUInt32(verts.size(), stream);
			for(vector<Vertex>::iterator iter = verts.begin(), verts_end = verts.end(); iter != verts_end; ++iter)
				WriteVec3(iter->pos, stream);
		}

		unsigned int Read(istream& stream)
		{
			if(unsigned int count = ReadUInt32(stream))
			{
				Vec3* verts = new Vec3[count];
				for(unsigned int i = 0; i < count; ++i)
					verts[i] = ReadVec3(stream);

				Init(verts, count);

				delete[] verts;
			}

			return stream.fail() ? 1 : 0;
		}
	};




	/*
	 * ConvexMeshShape methods
	 */
	ConvexMeshShape::ConvexMeshShape() : CollisionShape(ST_ConvexMesh), imp(new Imp())												{ }
	ConvexMeshShape::ConvexMeshShape(Vec3* verts, unsigned int count) : CollisionShape(ST_ConvexMesh), imp(new Imp(verts, count))	{ }

	void ConvexMeshShape::InnerDispose()																{ if(imp) { delete imp; imp = NULL; } }

	MassInfo ConvexMeshShape::ComputeMassInfo()															{ return imp->ComputeMassInfo(); }

	AABB ConvexMeshShape::GetTransformedAABB(const Mat4& xform)											{ return imp->GetTransformedAABB(xform); }
	AABB ConvexMeshShape::ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache)			{ return imp->ComputeCachedWorldAABB(xform, cache); }

	void ConvexMeshShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)	{ imp->DebugDraw(renderer, pos, ori); }

	void ConvexMeshShape::Write(ostream& stream)														{ imp->Write(stream); }
	unsigned int ConvexMeshShape::Read(istream& stream)													{ return imp->Read(stream); }




	/*
	 *  ConvexMeshShape conversion functions
	 */
	ConvexMeshShape* ConvexMeshShape::FromVBO(VertexBuffer* vbo)
	{
		unsigned int num_verts = vbo->GetNumVerts();
		float* vert_ptr = vbo->GetFloatPointer("gl_Vertex");

		Vec3* verts = new Vec3[num_verts];
		if(vbo->GetAttribNPerVertex("gl_Vertex") == 4)
		{
			for(unsigned int i = 0; i < num_verts; ++i, vert_ptr += 2)
			{
				Vec3& vert = verts[i];
				vert.x = *(vert_ptr++);
				vert.y = *(vert_ptr++);
				vert.z = *vert_ptr;
			}
		}
		else
		{
			for(unsigned int i = 0; i < num_verts; ++i)
			{
				Vec3& vert = verts[i];
				vert.x = *(vert_ptr++);
				vert.y = *(vert_ptr++);
				vert.z = *(vert_ptr++);
			}
		}

		unsigned int unique_count = 0;
		for(unsigned int i = 0; i < num_verts; ++i)
		{
			bool ok = true;
			for(unsigned int j = 0; j < unique_count; ++j)
				if((verts[i] - verts[j]).ComputeMagnitudeSquared() == 0.0f)
				{
					ok = false;
					break;
				}

			if(ok)
				verts[unique_count++] = verts[i];
		}

		ConvexMeshShape* result = new ConvexMeshShape();
		result->imp->Init(verts, unique_count);

		delete[] verts;

		return result;
	}




	/*
	 * ConvexMeshShapeInstanceCache methods
	 */
	ConvexMeshShapeInstanceCache::ConvexMeshShapeInstanceCache() : verts(), face_normals(), aabb() { }

	void ConvexMeshShapeInstanceCache::Update(const Mat4& xform, ConvexMeshShape::Imp* shape)
	{
		verts.clear();
		face_normals.clear();

		vector<ConvexMeshShape::Imp::Vertex>::iterator vert_iter = shape->verts.begin(), verts_end = shape->verts.end();
		Vec3 xformed_vert = xform.TransformVec3_1(vert_iter->pos);
		aabb = AABB(xformed_vert);
		verts.push_back(xformed_vert);

		for(++vert_iter; vert_iter != verts_end; ++vert_iter)
		{
			xformed_vert = xform.TransformVec3_1(vert_iter->pos);

			aabb.Expand(xformed_vert);
			verts.push_back(xformed_vert);
		}

		for(vector<ConvexMeshShape::Imp::Face>::iterator iter = shape->faces.begin(), faces_end = shape->faces.end(); iter != faces_end; ++iter)
			face_normals.push_back(xform.TransformVec3_0(iter->plane.normal));
	}
}
