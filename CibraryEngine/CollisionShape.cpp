#include "StdAfx.h"
#include "CollisionShape.h"

#include "AABB.h"

#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

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

	MassInfo CollisionShape::ComputeMassInfo() { return MassInfo(); }

	// default orientation returns a degenerate AABB
	AABB CollisionShape::GetTransformedAABB(const Mat4& xform) { return AABB(); }

	ShapeType CollisionShape::GetShapeType() { return type; }

	bool CollisionShape::CanMove() { return CanShapeTypeMove(type); }

	bool CollisionShape::CanShapeTypeMove(ShapeType type)
	{
		switch(type)
		{
			case ST_Ray:
			case ST_Sphere:
			case ST_MultiSphere:
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
			case ST_MultiSphere:
				temp = new MultiSphereShape();
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
}
