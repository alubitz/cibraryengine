#include "StdAfx.h"
#include "CollisionShape.h"

#include "Serialize.h"
#include "Physics.h"

#include "VertexBuffer.h"
#include "Content.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "DebugDrawMaterial.h"

namespace CibraryEngine
{
	/*
	 * CollisionShape methods
	 */
	CollisionShape::CollisionShape(ShapeType type) : type(type) { }

	Material* CollisionShape::GetDebugDrawMaterial()
	{
		static DebugDrawMaterial* mat = new DebugDrawMaterial();
		return mat;
	}

	MassInfo CollisionShape::ComputeMassInfo() { return MassInfo(); }

	ShapeType CollisionShape::GetShapeType() { return type; }

	bool CollisionShape::CanMove()
	{
		switch(type)
		{
			case ST_Ray:
			case ST_Sphere:
				return true;

			case ST_TriangleMesh:
			case ST_InfinitePlane:
			default:
				return false;
		}
	}

	void CollisionShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori) { }

	unsigned int CollisionShape::Read(istream& stream) { return 1; }
	void CollisionShape::Write(ostream& stream) { }

	unsigned int CollisionShape::ReadCollisionShape(CollisionShape*& shape, istream& stream)
	{
		string data = ReadString4(stream);
		istringstream ss(data);

		ShapeType type = (ShapeType)ReadUInt16(ss);

		CollisionShape* temp = NULL;
		switch(type)
		{
			case ST_Ray:
				temp = new RayShape();
				break;
			case ST_Sphere:
				temp = new SphereShape();
				break;
			case ST_TriangleMesh:
				temp = new TriangleMeshShape();
				break;
			case ST_InfinitePlane:
				temp = new InfinitePlaneShape();
				break;
			default:
				return 1;				// error code 1 = invalid shape type
		}

		if(unsigned int error = temp->Read(ss))
		{
			temp->Dispose();
			delete temp;

			return error + 1;			// error code > 1 = shape load error
		}
		else
		{
			shape = temp;
			return 0;					// return 0 = all clear
		}
	}

	unsigned int CollisionShape::WriteCollisionShape(CollisionShape* shape, ostream& stream)
	{
		stringstream ss;

		WriteUInt16((unsigned short)shape->type, ss);
		shape->Write(ss);

		WriteString4(ss.str(), stream);

		return stream.fail() ? 1 : 0;
	}




	/*
	 * RayShape methods
	 */
	RayShape::RayShape() : CollisionShape(ST_Ray) { }

	void RayShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
	{
		Mat3 rm(Mat3::Identity());
		Vec3 x = Vec3(rm[0], rm[1], rm[2]) * 0.1f;
		Vec3 y = Vec3(rm[3], rm[4], rm[5]) * 0.1f;
		Vec3 z = Vec3(rm[6], rm[7], rm[8]) * 0.1f;

		renderer->objects.push_back(RenderNode(GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - x, pos + x), 1.0f));
		renderer->objects.push_back(RenderNode(GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - y, pos + y), 1.0f));
		renderer->objects.push_back(RenderNode(GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - z, pos + z), 1.0f));
	}




	/*
	 * SphereShape methods
	 */
	SphereShape::SphereShape() : CollisionShape(ST_Sphere) { }
	SphereShape::SphereShape(float radius) : CollisionShape(ST_Sphere), radius(radius) { }

	MassInfo SphereShape::ComputeMassInfo()
	{
		static const float sphere_volume_coeff = 4.0f / 3.0f * float(M_PI);
		
		float r_squared = radius * radius;
		float mass = sphere_volume_coeff * r_squared * radius;
		float moi = 0.4f * mass * r_squared;

		MassInfo result(Vec3(), mass);
		result.moi[0] = result.moi[4] = result.moi[8] = moi;
		
		return result;
	}

	void SphereShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
	{
		Mat3 rm = ori.ToMat3();
		Vec3 x = Vec3(rm[0], rm[1], rm[2]) * radius;
		Vec3 y = Vec3(rm[3], rm[4], rm[5]) * radius;
		Vec3 z = Vec3(rm[6], rm[7], rm[8]) * radius;

		static const Vec3 r(1, 0, 0), g(0, 1, 0), b(0, 0, 1);

		renderer->objects.push_back(RenderNode(GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - x, pos + x, r), 1.0f));
		renderer->objects.push_back(RenderNode(GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - y, pos + y, g), 1.0f));
		renderer->objects.push_back(RenderNode(GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - z, pos + z, b), 1.0f));
	}

	void SphereShape::Write(ostream& stream) { WriteSingle(radius, stream); }
	unsigned int SphereShape::Read(istream& stream) { radius = ReadSingle(stream); return 0; }




	/*
	 * TriangleMeshShape methods
	 */
	TriangleMeshShape::TriangleMeshShape() : CollisionShape(ST_TriangleMesh), vertices(), triangles() { }

	TriangleMeshShape::TriangleMeshShape(VertexBuffer* vbo) : CollisionShape(ST_TriangleMesh), vertices(), triangles()
	{
		if(vbo)
		{
			if(unsigned int num_verts = vbo->GetNumVerts())
			{
				if(float* pos_ptr = vbo->GetFloatPointer("gl_Vertex"))
				{
					bool skip_fourth = vbo->GetAttribute("gl_Vertex").n_per_vertex == 4;

					for(unsigned int i = 0; i < num_verts;)
					{
						Tri tri;

						for(char j = 0; j < 3; ++j)
						{
							float x = *(pos_ptr++);
							float y = *(pos_ptr++);
							float z = *(pos_ptr++);

							if(skip_fourth)
								++pos_ptr;

							// TODO: don't store duplicate vertices

							Vec3 vec(x, y, z);
							vertices.push_back(vec);
							tri.indices[j] = i++;
						}
						triangles.push_back(tri);
					}
				}
			}
		}
	}

	void TriangleMeshShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori)
	{
		Mat3 rm(ori.ToMat3());

		vector<Vec3> transformed;
		for(vector<Vec3>::iterator iter = vertices.begin(); iter != vertices.end(); ++iter)
			transformed.push_back(rm * *iter + pos);

		Material* mat = GetDebugDrawMaterial();

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




	/*
	 * InfinitePlaneShape methods
	 */
	InfinitePlaneShape::InfinitePlaneShape() : CollisionShape(ST_InfinitePlane) { }
	InfinitePlaneShape::InfinitePlaneShape(const Plane& plane) : CollisionShape(ST_InfinitePlane), plane(plane) { }

	void InfinitePlaneShape::Write(ostream& stream)
	{
		WriteVec3(plane.normal, stream);
		WriteSingle(plane.offset, stream);
	}

	unsigned int InfinitePlaneShape::Read(istream& stream)
	{
		plane.normal = ReadVec3(stream);
		plane.offset = ReadSingle(stream);

		return 0;
	}
}
