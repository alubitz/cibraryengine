#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	struct Mat4;
	struct ContactPoint;
	struct Sphere;

	class MultiSphereShape : public CollisionShape
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			MultiSphereShape();
			MultiSphereShape(Vec3* centers, float* radii, unsigned int count);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			MassInfo ComputeMassInfo();

			bool Contains(const Vec3& point);

			bool CollisionCheck(const Mat4& xform, const Plane& plane, ContactPoint& result, RigidBody* ibody = NULL, RigidBody* jbody = NULL);
			bool CollisionCheck(const Sphere& sphere, ContactPoint& result, RigidBody* ibody = NULL, RigidBody* jbody = NULL);			// sphere pre-transformed into local coords

			void Write(ostream& stream);
			unsigned int Read(istream& stream);
	};
}
