#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

#include "Plane.h"

namespace CibraryEngine
{
	class InfinitePlaneShape : public CollisionShape
	{
		public:

			Plane plane;

			InfinitePlaneShape();
			InfinitePlaneShape(const Plane& plane);

			void Write(ostream& stream);
			unsigned int Read(istream& stream);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori, const Vec3& color);
	};
}
