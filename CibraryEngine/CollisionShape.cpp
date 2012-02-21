#include "StdAfx.h"
#include "CollisionShape.h"

#include "Serialize.h"
#include "Physics.h"

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
	void CollisionShape::InnerDispose() { }

	MassInfo CollisionShape::ComputeMassInfo() { return MassInfo(); }

	ShapeType CollisionShape::GetShapeType() { return type; }




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
	TriangleMeshShape::TriangleMeshShape() : CollisionShape(ST_TriangleMesh) { }

	MassInfo TriangleMeshShape::ComputeMassInfo() { return MassInfo(); }




	/*
	 * InfinitePlaneShape methods
	 */
	InfinitePlaneShape::InfinitePlaneShape(const Plane& plane) : CollisionShape(ST_InfinitePlane), plane(plane) { }

	MassInfo InfinitePlaneShape::ComputeMassInfo() { return MassInfo(); }
}
