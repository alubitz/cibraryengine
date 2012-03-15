#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	class MultiSphereShape : public CollisionShape
	{
		protected:

			void InnerDispose();

		public:

			Vec3* centers;
			float* radii;
			unsigned int count;

			MultiSphereShape();
			MultiSphereShape(Vec3* centers, float* radii, unsigned int count);

			MassInfo ComputeMassInfo();

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			void Write(ostream& stream);
			unsigned int Read(istream& stream);
	};
}
