#include "StdAfx.h"

#include "VisionBlocker.h"

#include "DebugLog.h"

#include "Entity.h"
#include "Physics.h"

namespace CibraryEngine
{
	bool VisionBlocker::CheckLineOfSight(PhysicsWorld* physics, Vec3 from, Vec3 to)
	{
		// define a callback for when a ray intersects an object
		struct : btCollisionWorld::RayResultCallback
		{
			float result;

			btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
			{
				void* void_pointer = rayResult.m_collisionObject->getUserPointer();
				if(void_pointer != NULL)
				{
					VisionBlocker* vb = dynamic_cast<VisionBlocker*>((Entity*)void_pointer);
					if(vb != NULL)
					{
						float frac = rayResult.m_hitFraction;
						if(frac > result)
							result = frac;
					}
				}
				return 1;
			}
		} ray_callback;

		ray_callback.result = 0.0f;
		physics->RayTest(from, to, ray_callback);				// run that function for anything on this ray...
		return ray_callback.result == 0.0f;						// if it's still zero, that means nothing is between those two points
	}
}
