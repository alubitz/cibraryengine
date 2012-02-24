#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Vector.h"
#include "Plane.h"

#include "ContentTypeHandler.h"

namespace CibraryEngine
{
	using namespace std;

	struct MassInfo;
	class RigidBody;

	struct VertexBuffer;

	enum ShapeType
	{
		ST_Ray,
		ST_Sphere,
		ST_TriangleMesh,
		ST_InfinitePlane
	};

	/** A shape usable for collision detection and/or response */
	class CollisionShape : public Disposable
	{
		private:

			ShapeType type;

		protected:

			CollisionShape(ShapeType type);

		public:

			/** Compute the mass info for this shape, assuming a density of 1 */
			virtual MassInfo ComputeMassInfo();

			ShapeType GetShapeType();


			bool CanMove();
	};

	/** The simplest of all collision shapes */
	class RayShape : public CollisionShape
	{
		public:

			RayShape();
	};

	/** Slightly less simple collision shape */
	class SphereShape : public CollisionShape
	{
		public:

			float radius;

			SphereShape(float radius);

			MassInfo ComputeMassInfo();
	};

	class TriangleMeshShape : public CollisionShape
	{
		public:

			vector<Vec3> vertices;

			struct Tri { unsigned int indices[3]; };
			vector<Tri> triangles;

			TriangleMeshShape();
			TriangleMeshShape(VertexBuffer* vbo);
	};

	class InfinitePlaneShape : public CollisionShape
	{
		public:

			Plane plane;

			InfinitePlaneShape(const Plane& plane);
	};
}
