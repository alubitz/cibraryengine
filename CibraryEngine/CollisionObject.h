#pragma once

#include "StdAfx.h"

#include "Disposable.h"

namespace CibraryEngine
{
	struct AABB;
	class SceneRenderer;

	// TODO: move more members from RigidBody up to CollisionObject
	class CollisionObject : public Disposable
	{
		public:

			virtual void DebugDraw(SceneRenderer* renderer) = 0;

			virtual AABB GetAABB(float timestep) = 0;
	};
}
