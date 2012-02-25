#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Vector.h"
#include "Plane.h"

#include "ContentTypeHandler.h"

namespace CibraryEngine
{
	using namespace std;

	struct Quaternion;

	struct MassInfo;
	class RigidBody;

	struct VertexBuffer;
	class SceneRenderer;
	class Material;

	enum ShapeType
	{
		ST_Ray = 1,
		ST_Sphere = 2,
		ST_TriangleMesh = 3,
		ST_InfinitePlane = 4
	};

	/** A shape usable for collision detection and/or response */
	class CollisionShape : public Disposable
	{
		private:

			ShapeType type;

		protected:

			CollisionShape(ShapeType type);

			static Material* GetDebugDrawMaterial();

		public:

			/** Compute the mass info for this shape, assuming a density of 1 */
			virtual MassInfo ComputeMassInfo();

			ShapeType GetShapeType();

			bool CanMove();

			virtual void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			/** Reads a collision shape of the appropriate type from an input stream */
			virtual unsigned int Read(istream& stream);
			/** Write a collision shape to an output stream */
			virtual void Write(ostream& stream);

			static unsigned int ReadCollisionShape(CollisionShape*& shape, istream& stream);
			static unsigned int WriteCollisionShape(CollisionShape* shape, ostream& stream);
	};

	/** The simplest of all collision shapes */
	class RayShape : public CollisionShape
	{
		public:

			RayShape();

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);
	};

	/** Slightly less simple collision shape */
	class SphereShape : public CollisionShape
	{
		public:

			float radius;

			SphereShape();
			SphereShape(float radius);

			MassInfo ComputeMassInfo();

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);

			void Write(ostream& stream);
			unsigned int Read(istream& stream);
	};

	class TriangleMeshShape : public CollisionShape
	{
		public:

			vector<Vec3> vertices;

			struct Tri { unsigned int indices[3]; };
			vector<Tri> triangles;

			TriangleMeshShape();
			TriangleMeshShape(VertexBuffer* vbo);

			void Write(ostream& stream);
			unsigned int Read(istream& stream);

			void DebugDraw(SceneRenderer* renderer, const Vec3& pos, const Quaternion& ori);
	};

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
