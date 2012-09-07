#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

#include "TriangleMeshShape.h"

namespace CibraryEngine
{
	struct AABB;

	struct Mat4;
	struct ContactPoint;
	struct Sphere;
	struct Ray;

	class MultiSphereShape : public CollisionShape
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			MultiSphereShape();
			MultiSphereShape(Sphere* spheres, unsigned int count);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			AABB GetTransformedAABB(const Mat4& xform);

			MassInfo ComputeMassInfo();

			bool CollideRay(const Ray& ray, ContactPoint& result, float& time, RigidBody* ibody = NULL, RigidBody* jbody = NULL);						// ray pre-transformed into local coords
			bool CollideSphere(const Sphere& sphere, ContactPoint& result, RigidBody* ibody = NULL, RigidBody* jbody = NULL);								// sphere pre-transformed into local coords
			bool CollidePlane(const Mat4& my_xform, const Plane& plane, vector<ContactPoint>& results, RigidBody* ibody = NULL, RigidBody* jbody = NULL);
			bool CollideMultisphere(const Mat4& xform, const MultiSphereShape* other, ContactPoint& result, RigidBody* ibody = NULL, RigidBody* jbody = NULL);	// xform is product of i xform and inverse j xform
			bool CollideMesh(const Mat4& my_xform, const TriangleMeshShape::TriCache& tri, ContactPoint& result, RigidBody* ibody = NULL, RigidBody* jbody = NULL);	// xform is product of j xform and inverse i xform

			AABB GetAABB();

			void Write(ostream& stream);
			unsigned int Read(istream& stream);
	};
}
