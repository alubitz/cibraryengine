#include "StdAfx.h"
#include "CollisionShape.h"

#include "Serialize.h"
#include "Physics.h"

#include "VertexBuffer.h"
#include "Content.h"

namespace CibraryEngine
{
	/*
	 * CollisionShape I/O methods
	 */
	CollisionShape* CollisionShape::ReadCollisionShape(istream& stream)
	{
		unsigned int buffer_size = ReadUInt32(stream);
		stream.ignore(buffer_size);

		return NULL;
	}

	void CollisionShape::WriteCollisionShape(CollisionShape* shape, ostream& stream) { WriteUInt32(0, stream); }




	/*
	 * CollisionShape methods
	 */
	CollisionShape::CollisionShape(ShapeType type) : type(type) { }

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




	/*
	 * RayShape method
	 */
	RayShape::RayShape() : CollisionShape(ST_Ray) { }




	/*
	 * SphereShape methods
	 */
	SphereShape::SphereShape(float radius) : CollisionShape(ST_Sphere), radius() { }

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
				if(float* pos_ptr = vbo->GetFloatPointer("gl_Position"))
				{
					bool skip_fourth = vbo->GetAttribute("gl_Position").n_per_vertex == 4;

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




	/*
	 * InfinitePlaneShape method
	 */
	InfinitePlaneShape::InfinitePlaneShape(const Plane& plane) : CollisionShape(ST_InfinitePlane), plane(plane) { }




	/*
	 * CollisionShapeLoader methods
	 */
	CollisionShapeLoader::CollisionShapeLoader(ContentMan* man) : ContentTypeHandler<CollisionShape>(man) { }

	CollisionShape* CollisionShapeLoader::Load(ContentMetadata& what)
	{
		string filename = "Files/Physics/" + what.name + ".csh";

		CollisionShape* shape = NULL;
		if(unsigned int csh_result = CollisionShapeLoader::LoadCSH(shape, filename))
		{
			stringstream csh_msg;
			csh_msg << "LoadCSH (" << what.name << ") returned with status " << csh_result << "!" << endl;
			Debug(csh_msg.str());
		}

		return shape;
	}
	void CollisionShapeLoader::Unload(CollisionShape* content, ContentMetadata& what)
	{
		content->Dispose();
		delete content;
	}

	unsigned int CollisionShapeLoader::LoadCSH(CollisionShape*& shape, string filename)
	{
		// TODO: implement this
		return 1;
	}

	unsigned int CollisionShapeLoader::SaveCSH(CollisionShape* shape, string filename)
	{
		// TODO: implement this
		return 1;
	}
}
