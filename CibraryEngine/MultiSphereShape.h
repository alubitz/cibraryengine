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

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			AABB GetTransformedAABB(const Mat4& xform);
			AABB ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache);

			MassInfo ComputeMassInfo();

			bool CollideRay(const Ray& ray, RayResult& result, RayCollider* collider = NULL, RigidBody* body = NULL);										// ray pre-transformed into local coords
			bool CollideSphere(const Sphere& sphere, ContactPoint& result, RigidBody* ibody = NULL, RigidBody* jbody = NULL);								// sphere pre-transformed into local coords
			bool CollidePlane(const Mat4& my_xform, const Plane& plane, vector<ContactPoint>& results, RigidBody* ibody = NULL, RigidBody* jbody = NULL);
			bool CollideMesh(const Mat4& my_xform, vector<Sphere>& my_spheres, const TriangleMeshShape::TriCache& tri, ContactPoint& result, RigidBody* ibody = NULL, RigidBody* jbody = NULL);	// xform is product of j xform and inverse i xform

			// multisphere-multisphere is now handled in Physics.cpp

			AABB GetAABB();

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
