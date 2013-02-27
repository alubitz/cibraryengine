#pragma once

#include "StdAfx.h"

#include "CollisionShape.h"

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


			// TODO: add CollideRay, CollideMesh, etc. here


			unsigned int Read(istream& stream);
			void Write(ostream& stream);


			// conversion functions
			static ConvexMeshShape* FromVBO(VertexBuffer* vbo);
	};

	class ConvexMeshShapeInstanceCache : public ShapeInstanceCache
	{
		public:
			
			vector<Vec3> verts;
			vector<Vec3> face_normals;

			AABB aabb;

			ConvexMeshShapeInstanceCache();

			void Update(const Mat4& xform, ConvexMeshShape::Imp* shape);
	};
}
