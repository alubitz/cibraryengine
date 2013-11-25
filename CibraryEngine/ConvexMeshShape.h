#pragma once

#include "StdAfx.h"

#include "CollisionShape.h"
#include "AABB.h"

#include "TriangleMeshShape.h"

namespace CibraryEngine
{
	class ContactRegion;
	class ContactDataCollector;

	struct Plane;

	struct Ray;
	class RayCollider;
	struct RayResult;

	struct VertexBuffer;

	class ConvexMeshShape : public CollisionShape
	{
		friend class ConvexMeshShapeInstanceCache;

		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			ConvexMeshShape();
			ConvexMeshShape(Vec3* verts, unsigned int count);

			MassInfo ComputeMassInfo();

			AABB GetTransformedAABB(const Mat4& xform);
			AABB ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori, const Vec3& color);


			bool CollideRay(const Ray& ray, RayResult& result, RayCollider* collider = NULL, RigidBody* body = NULL);					// ray has been transformed into coordinate system of mesh
			ContactRegion* CollidePlane(ConvexMeshShapeInstanceCache* my_cache, const Plane& plane, ContactDataCollector* results, RigidBody* ibody = NULL, RigidBody* jbody = NULL);
			ContactRegion* CollideTri(ConvexMeshShapeInstanceCache* my_cache, const TriangleMeshShape::TriCache& tri, ContactDataCollector* results, RigidBody* ibody = NULL, RigidBody* jbody = NULL);		// my_cache is a cache for collisions between this pair of objects only (not the same as the rigid body's cache)
			ContactRegion* CollideConvexMesh(ConvexMeshShapeInstanceCache* ishape, ConvexMeshShapeInstanceCache* jshape, ContactDataCollector* results, RigidBody* ibody = NULL, RigidBody* jbody = NULL);


			unsigned int Read(istream& stream);
			void Write(ostream& stream);


			// conversion functions
			static ConvexMeshShape* FromVBO(VertexBuffer* vbo);
	};

	class ConvexMeshShapeInstanceCache : public ShapeInstanceCache
	{
		public:
			
			vector<Vec3> verts;
			AABB aabb;

			ConvexMeshShapeInstanceCache();

			void Update(const Mat4& xform, ConvexMeshShape::Imp* shape);
			void Update(const Mat4& xform, ConvexMeshShape* shape);
	};
}
