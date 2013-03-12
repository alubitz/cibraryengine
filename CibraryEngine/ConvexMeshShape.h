#pragma once

#include "StdAfx.h"

#include "CollisionShape.h"
#include "AABB.h"

#include "TriangleMeshShape.h"

namespace CibraryEngine
{
	struct ContactPoint;
	struct ContactPointAllocator;

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

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);


			bool CollideRay(const Ray& ray, RayResult& result, RayCollider* collider = NULL, RigidBody* body = NULL);					// ray has been transformed into coordinate system of mesh
			bool CollidePlane(ConvexMeshShapeInstanceCache* my_cache, const Plane& plane, ContactPointAllocator* alloc, vector<ContactPoint*>& results, RigidBody* ibody = NULL, RigidBody* jbody = NULL);
			bool CollideTri(ConvexMeshShapeInstanceCache* my_cache, const TriangleMeshShape::TriCache& tri, ContactPointAllocator* alloc, vector<ContactPoint*>& results, RigidBody* ibody = NULL, RigidBody* jbody = NULL);		// my_cache is a cache for collisions between this pair of objects only (not the same as the rigid body's cache)
			bool CollideConvexMesh(ConvexMeshShapeInstanceCache* ishape, ConvexMeshShapeInstanceCache* jshape, ContactPointAllocator* alloc, vector<ContactPoint*>& contact_points, RigidBody* ibody = NULL, RigidBody* jbody = NULL);


			unsigned int Read(istream& stream);
			void Write(ostream& stream);


			// conversion functions
			static ConvexMeshShape* FromVBO(VertexBuffer* vbo);
	};

	class ConvexMeshShapeInstanceCache : public ShapeInstanceCache
	{
		public:
			
			vector<Vec3> verts;
			vector<Vec3> edges_normalized;
			vector<Vec3> edge_points;
			vector<Vec3> face_normals;

			AABB aabb;

			ConvexMeshShapeInstanceCache();

			void Update(const Mat4& xform, ConvexMeshShape::Imp* shape);
			void Update(const Mat4& xform, ConvexMeshShape* shape);
	};
}
