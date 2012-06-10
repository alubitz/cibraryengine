#include "StdAfx.h"

#include "VisionBlocker.h"

#include "DebugLog.h"

#include "Entity.h"
#include "Physics.h"
#include "RigidBody.h"

namespace CibraryEngine
{
	bool VisionBlocker::CheckLineOfSight(PhysicsWorld* physics, Vec3 from, Vec3 to)
	{
		struct RayCallback : public CollisionCallback
		{
			bool result;
			RayCallback() : result(true) { }
			bool OnCollision(const ContactPoint& cp)
			{
				if(Entity* entity = cp.obj_b->GetUserEntity())
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
