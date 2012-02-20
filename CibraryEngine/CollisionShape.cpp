#include "StdAfx.h"
#include "CollisionShape.h"

#include "Serialize.h"

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
	CollisionShape::CollisionShape() { }
	void CollisionShape::InnerDispose() { }

	float CollisionShape::ComputeVolume() { return 0.0f; }




	/*
	 * RayShape method
	 */
	RayShape::RayShape() : CollisionShape() { }




	/*
	 * SphereShape methods
	 */
	SphereShape::SphereShape(float radius) : CollisionShape(), radius() { }

	float SphereShape::ComputeVolume()
	{
		static const float sphere_volume_coeff = 4.0f / 3.0f * float(M_PI);
		return sphere_volume_coeff * radius * radius * radius;
	}




	/*
	 * TriangleMeshShape methods
	 */
	TriangleMeshShape::TriangleMeshShape() : CollisionShape() { }

	float TriangleMeshShape::ComputeVolume() { return 0.0f; }
}
