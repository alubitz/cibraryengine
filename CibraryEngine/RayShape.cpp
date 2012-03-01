#include "StdAfx.h"
#include "RayShape.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "DebugDrawMaterial.h"

namespace CibraryEngine
{
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

		renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - x, pos + x), 1.0f));
		renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - y, pos + y), 1.0f));
		renderer->objects.push_back(RenderNode(DebugDrawMaterial::GetDebugDrawMaterial(), new DebugDrawMaterialNodeData(pos - z, pos + z), 1.0f));
	}
}