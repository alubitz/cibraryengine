#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Vector.h"

#include "ContentTypeHandler.h"

namespace CibraryEngine
{
	using namespace std;

	struct Quaternion;
	struct Mat4;

	struct AABB;

	struct MassInfo;
	class RigidBody;

	struct VertexBuffer;
	class SceneRenderer;
	class Material;

	// subclass this!
	class ShapeInstanceCache
	{
		public:
			virtual ~ShapeInstanceCache() { }
	};

	enum ShapeType
	{
		ST_Ray           = 1,				// no longer supported
		ST_Sphere        = 2,
		ST_TriangleMesh  = 3,
		ST_InfinitePlane = 4,
		ST_MultiSphere   = 5,
		ST_ConvexMesh    = 6,

		ST_ShapeTypeMax
	};

	/** A shape usable for collision detection and/or response */
	class CollisionShape : public Disposable
	{
		private:

			ShapeType type;

		protected:

			CollisionShape(ShapeType type) : type(type) { }

		public:

			/** Compute the mass info for this shape, assuming a density of 1 */
			virtual MassInfo ComputeMassInfo();

			/** Get the AABB of this shape transformed by the specified matrix. Default implementation returns a degenerate AABB */
			virtual AABB GetTransformedAABB(const Mat4& xform);

			/** Like GetTransformedAABB, except the xform is a RigidBody's world xform, and it can be used to cache other data with the RigidBody; default implementation returns GetTransformedAABB */
			virtual AABB ComputeCachedWorldAABB(const Mat4& xform, ShapeInstanceCache*& cache);

			ShapeType GetShapeType() const                     { return type; }

			bool CanMove() const                               { return CanShapeTypeMove(type); }
			static bool CanShapeTypeMove(ShapeType type);
			static string GetShapeTypeName(ShapeType type);

			virtual void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori, const Vec3& color = Vec3(1, 1, 1)) { }

			/** Reads a collision shape of the appropriate type from an input stream */
			virtual unsigned int Read(istream& stream)         { return 1; }
			/** Write a collision shape to an output stream */
			virtual void Write(ostream& stream)                { }

			static unsigned int ReadCollisionShape(CollisionShape*& shape, istream& stream);
			static unsigned int WriteCollisionShape(CollisionShape* shape, ostream& stream);
	};
}
