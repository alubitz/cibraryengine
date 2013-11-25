#pragma once

#include "StdAfx.h"
#include "CollisionShape.h"

#include "TriangleMeshShape.h"

namespace CibraryEngine
{
	struct AABB;

	struct Mat4;
	struct ContactPoint;
	class ContactRegion;
	class ContactDataCollector;
	struct Sphere;
	struct Ray;

	class RayCollider;
	struct RayResult;

	class MultiSphereShape : public CollisionShape
	{
		friend class MultiSphereShapeInstanceCache;

		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			MultiSphereShape();
			MultiSphereShape(Sphere* spheres, unsigned int count);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori, const Vec3& color);

			AABB GetAABB();
			AABB GetTransformedAABB(const Mat4& xform);
			AABB ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache);

			MassInfo ComputeMassInfo();

			bool CollideRay(const Ray& ray, RayResult& result, RayCollider* collider = NULL, RigidBody* body = NULL);										// ray pre-transformed into local coords
			ContactRegion* CollideSphere(const Sphere& sphere, ContactDataCollector* collect, RigidBody* ibody = NULL, RigidBody* jbody = NULL);								// sphere pre-transformed into local coords
			ContactRegion* CollidePlane(const Mat4& my_xform, const Plane& plane, ContactDataCollector* collect, RigidBody* ibody = NULL, RigidBody* jbody = NULL);
			ContactRegion* CollideMesh(const Mat4& my_xform, vector<Sphere>& my_spheres, const TriangleMeshShape::TriCache& tri, ContactDataCollector* collect, RigidBody* ibody = NULL, RigidBody* jbody = NULL);	// xform is product of j xform and inverse i xform

			// multisphere-multisphere is now handled in Physics.cpp

			void Write(ostream& stream);
			unsigned int Read(istream& stream);
	};

	class MultiSphereShapeInstanceCache : public ShapeInstanceCache
	{
		public:

			vector<Sphere> spheres;
			AABB aabb;

			MultiSphereShapeInstanceCache();

			void Update(const Mat4& xform, MultiSphereShape::Imp* shape);
	};
}
