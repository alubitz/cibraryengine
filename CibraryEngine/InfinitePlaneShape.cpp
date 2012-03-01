#include "StdAfx.h"
#include "InfinitePlaneShape.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "DebugDrawMaterial.h"

#include "Serialize.h"

namespace CibraryEngine
{
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
