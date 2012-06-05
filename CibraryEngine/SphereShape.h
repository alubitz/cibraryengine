#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	/** Slightly less simple collision shape */
	class SphereShape : public CollisionShape
	{
		public:

			float radius;

			SphereShape();
			SphereShape(float radius);

			MassInfo ComputeMassInfo();

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			AABB GetTransformedAABB(const Mat4& xform);

			void Write(ostream& stream);
			unsigned int Read(istream& stream);
	};
}
