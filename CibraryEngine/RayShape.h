#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	/** The simplest of all collision shapes */
	class RayShape : public CollisionShape
	{
		public:

			RayShape();

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);
	};
}
