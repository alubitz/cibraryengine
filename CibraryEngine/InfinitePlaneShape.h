#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

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
	};
}
