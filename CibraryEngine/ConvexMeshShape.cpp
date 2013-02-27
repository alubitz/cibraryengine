#include "StdAfx.h"
#include "ConvexMeshShape.h"

#include "MassInfo.h"
#include "AABB.h"

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
			vector<Plane> boundary_planes;

			Face(const Plane& plane) : plane(plane), boundary_planes() { }
		};

		vector<Vertex> verts;
		vector<Edge> edges;
		vector<Face> faces;

		Imp() : verts(), edges(), faces() { }
		Imp(Vec3* verts, unsigned int count) { Init(verts, count); }
		~Imp() { }

		void Init(Vec3* in_verts, unsigned int count)
		{
			verts = vector<Vertex>();
			edges = vector<Edge>();
			faces = vector<Face>();
	
			if(count > 0)
			{
				static const float point_discard_threshold = 0.0000001f;

				// generate all possible planes
				vector<Plane> planes;
				for(unsigned int i = 2; i < count; ++i)
					for(unsigned int j = 1; j < i; ++j)
						for(unsigned int k = 0; k < j; ++k)
						{
							Plane plane = Plane::FromTriangleVertices(in_verts[i], in_verts[j], in_verts[k]);
							planes.push_back(plane);
							planes.push_back(Plane::Reverse(plane));
						}

				// discard non-exterior planes
				for(vector<Plane>::iterator iter = planes.begin(); iter != planes.end(); ++iter)
				{
					bool ok = true;
					for(unsigned int i = 0; i < count; ++i)
						if(iter->PointDistance(in_verts[i]) < point_discard_threshold)
						{
							ok = false;
							break;
						}

					// TODO: also make sure planes are unique

					if(ok)
						faces.push_back(Face(*iter));
				}

				// discard interior verts
				vector<Vec3> kept_verts;
				vector<Face>::iterator faces_begin = faces.begin(), faces_end = faces.end();
				for(unsigned int i = 0; i < count; ++i)
				{
					const Vec3& vert = in_verts[i];

					bool ok = true;
					for(vector<Face>::iterator iter = faces_begin; iter != faces_end; ++iter)
						if(iter->plane.PointDistance(vert) < point_discard_threshold)
						{
							ok = false;
							break;
						}

					if(ok)
						kept_verts.push_back(vert);
				}

				// produce convex polygons for each face
				for(vector<Face>::iterator iter = faces_begin; iter != faces_end; ++iter)
				{
					// TODO: implement this
				}
			}
		}

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
	ConvexMeshShape::ConvexMeshShape() : CollisionShape(ST_ConvexMesh), imp(new Imp()) { }
	ConvexMeshShape::ConvexMeshShape(Vec3* verts, unsigned int count) : CollisionShape(ST_ConvexMesh), imp(new Imp(verts, count)) { }

	void ConvexMeshShape::InnerDispose()					{ if(imp) { delete imp; imp = NULL; } }

	MassInfo ConvexMeshShape::ComputeMassInfo()
	{
		// TODO: implement this
		return CollisionShape::ComputeMassInfo();
	}

	AABB ConvexMeshShape::GetTransformedAABB(const Mat4& xform)
	{
		// TODO: implement this
		return CollisionShape::GetTransformedAABB(xform);
	}

	AABB ConvexMeshShape::ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache)
	{
		// TODO: implement this
		return CollisionShape::ComputeCachedWorldAABB(xform, cache);
	}

	void ConvexMeshShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
	{
		// TODO: implement this
	}

	unsigned int ConvexMeshShape::Read(istream& stream)		{ return imp->Read(stream); }
	void ConvexMeshShape::Write(ostream& stream)			{ return imp->Write(stream); }




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

		ConvexMeshShape* result = new ConvexMeshShape();
		result->imp->Init(verts, num_verts);

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
		verts.push_back(aabb);

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
