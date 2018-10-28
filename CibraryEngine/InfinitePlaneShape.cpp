#include "StdAfx.h"
#include "InfinitePlaneShape.h"

#include "Matrix.h"
#include "Quaternion.h"

#include "SceneRenderer.h"
#include "RenderNode.h"
#include "DebugDrawMaterial.h"
#include "CameraView.h"

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

	void InfinitePlaneShape::DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori, const Vec3& color)
	{
		// hack: only applies to ground plane
		if(plane.normal.y != 1.0f)
			return;

		// pos and ori are ignored
		Vec3 origin = plane.normal * plane.offset;
		Vec3 u = Vec3::Cross(plane.normal, Vec3(1, 0, 0));
		if(u.ComputeMagnitudeSquared() == 0)
			u = Vec3::Cross(plane.normal, Vec3(0, 1, 0));
		u = Vec3::Normalize(u);
		Vec3 v = Vec3::Cross(plane.normal, u);

		DebugDrawMaterial* ddm = DebugDrawMaterial::GetDebugDrawMaterial();

		Vec3 u0, u1, v0, v1, use_color;

		// draw big grid
		u0 = origin - u * 100.0f;
		u1 = origin + u * 100.0f;
		v0 = origin - v * 100.0f;
		v1 = origin + v * 100.0f;
		use_color = color * 0.5f;

		for(int i = -10; i <= 10; ++i)
		{
			float ii = 10.0f * i;
			renderer->objects.push_back(RenderNode(ddm, ddm->New(u0 + v * ii, u1 + v * ii, use_color), 1.0f));
			renderer->objects.push_back(RenderNode(ddm, ddm->New(v0 + u * ii, v1 + u * ii, use_color), 1.0f));
		}

		// draw small grid, focused around the area closest to the camera

		Vec3 camera_pos = renderer->camera->GetPosition();
		float udot = Vec3::Dot(camera_pos, u);
		while(udot < -5)
		{
			udot += 10;
			origin -= u * 10;
		}
		while(udot > 5)
		{
			udot -= 10;
			origin += u * 10;
		}
		float vdot = Vec3::Dot(camera_pos, v);
		while(vdot < -5)
		{
			vdot += 10;
			origin -= v * 10;
		}
		while(vdot > 5)
		{
			vdot -= 10;
			origin += v * 10;
		}

		u0 = origin - u * 20.0f;
		u1 = origin + u * 20.0f;
		v0 = origin - v * 20.0f;
		v1 = origin + v * 20.0f;
		use_color = color * 0.2f;

		for(int i = -19; i <= 19; ++i)
			if(i != -10 && i != 0 && i != 10)
			{
				float ii = 1.0f * i;
				renderer->objects.push_back(RenderNode(ddm, ddm->New(u0 + v * ii, u1 + v * ii, use_color), 1.0f));
				renderer->objects.push_back(RenderNode(ddm, ddm->New(v0 + u * ii, v1 + u * ii, use_color), 1.0f));
			}
	}
}
