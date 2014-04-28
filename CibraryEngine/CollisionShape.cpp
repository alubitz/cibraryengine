#include "StdAfx.h"
#include "CollisionShape.h"

#include "AABB.h"

#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"
#include "ConvexMeshShape.h"

#include "Serialize.h"
#include "Physics.h"

#include "VertexBuffer.h"
#include "Content.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "DebugDrawMaterial.h"

#include "MassInfo.h"

namespace CibraryEngine
{
	/*
	 * CollisionShape methods
	 */
	MassInfo CollisionShape::ComputeMassInfo()													{ return MassInfo(); }

	// default orientation returns a degenerate AABB
	AABB CollisionShape::GetTransformedAABB(const Mat4& xform)									{ return AABB(); }

	AABB CollisionShape::ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache)	{ return GetTransformedAABB(xform); }
	bool CollisionShape::CanShapeTypeMove(ShapeType type)
	{
		switch(type)
		{
			case ST_Ray:			// no longer supported
			case ST_Sphere:
			case ST_MultiSphere:
			case ST_ConvexMesh:
				return true;

			case ST_TriangleMesh:
			case ST_InfinitePlane:
			default:
				return false;
		}
	}

	string CollisionShape::GetShapeTypeName(ShapeType type)
	{
		switch(type)
		{
			case 0:
				return "<zero>";

			case ST_Ray:
				return "ST_Ray";
			case ST_Sphere:
				return "ST_Sphere";
			case ST_TriangleMesh:
				return "ST_TriangleMesh";
			case ST_InfinitePlane:
				return "ST_InfinitePlane";
			case ST_MultiSphere:
				return "ST_MultiSphere";
			case ST_ConvexMesh:
				return "ST_ConvexMesh";

			case ST_ShapeTypeMax:
				return "<shape type max>";

			default:
				return "<unknown>";
		}
	}

	unsigned int CollisionShape::ReadCollisionShape(CollisionShape*& shape, istream& stream)
	{
		string data = ReadString4(stream);
		istringstream ss(data);

		ShapeType type = (ShapeType)ReadUInt16(ss);

		CollisionShape* temp = NULL;
		switch(type)
		{
			case ST_Sphere:
				temp = new SphereShape();
				break;
			case ST_TriangleMesh:
				temp = new TriangleMeshShape();
				break;
			case ST_InfinitePlane:
				temp = new InfinitePlaneShape();
				break;
			case ST_MultiSphere:
				temp = new MultiSphereShape();
				break;
			case ST_ConvexMesh:
				temp = new ConvexMeshShape();
				break;

			case ST_Ray:
				return 2;				// error code 2 = shape type no longer supported

			default:
				return 1;				// error code 1 = invalid shape type
		}

		if(unsigned int error = temp->Read(ss))
		{
			temp->Dispose();
			delete temp;

			return error + 2;			// error code > 2 = shape load error
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
}
