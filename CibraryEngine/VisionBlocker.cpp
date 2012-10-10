#include "StdAfx.h"

#include "VisionBlocker.h"

#include "DebugLog.h"

#include "Entity.h"
#include "Physics.h"
#include "RigidBody.h"
#include "RayCollider.h"

namespace CibraryEngine
{
	bool VisionBlocker::CheckLineOfSight(PhysicsWorld* physics, Vec3 from, Vec3 to)
	{
		struct MyRayCallback : public RayCallback
		{
			bool result;
			MyRayCallback() : result(true) { }
			bool OnCollision(RayResult& rr)
			{
				if(Entity* entity = rr.body->GetUserEntity())
					if(VisionBlocker* vision_blocker = dynamic_cast<VisionBlocker*>(entity))
					{
						result = false;
						return true;
					}
				return false;
			}
		} ray_callback;

		physics->RayTest(from, to, ray_callback);
		return ray_callback.result;
	}
}
