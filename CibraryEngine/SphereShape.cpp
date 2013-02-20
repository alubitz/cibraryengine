#include "StdAfx.h"
#include "SphereShape.h"

#include "AABB.h"

#include "Physics.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "DebugDrawMaterial.h"

#include "Serialize.h"

namespace CibraryEngine
{
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

		DebugDrawMaterial* ddm = DebugDrawMaterial::GetDebugDrawMaterial();

		renderer->objects.push_back(RenderNode(ddm, ddm->New(pos - x, pos + x, r), 1.0f));
		renderer->objects.push_back(RenderNode(ddm, ddm->New(pos - y, pos + y, g), 1.0f));
		renderer->objects.push_back(RenderNode(ddm, ddm->New(pos - z, pos + z, b), 1.0f));
	}

	AABB SphereShape::GetTransformedAABB(const Mat4& xform) { return AABB(xform.TransformVec3(0, 0, 0, 1), radius); }

	void SphereShape::Write(ostream& stream) { WriteSingle(radius, stream); }
	unsigned int SphereShape::Read(istream& stream) { radius = ReadSingle(stream); return 0; }
}
