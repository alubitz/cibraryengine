#pragma once

#include "StdAfx.h"

#include "Physics.h"

namespace CibraryEngine
{
	struct ContactPoint;

	class ContactRegion
	{
		public:

			RigidBody* obj_a;
			RigidBody* obj_b;

			vector<ContactPoint*> points;

			ContactRegion() :                           obj_a(NULL), obj_b(NULL), points()   { }
			ContactRegion(RigidBody* a, RigidBody* b) : obj_a(a), obj_b(b), points()         { }

			~ContactRegion() { }
	};
}
