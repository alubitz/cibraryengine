#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Plane.h"

namespace CibraryEngine
{
	using namespace std;

	struct MassInfo;
	class RigidBody;

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

			virtual void InnerDispose();

		public:

			/** Compute the mass info for this shape, assuming a density of 1 */
			virtual MassInfo ComputeMassInfo();

			ShapeType GetShapeType();


			bool CanMove();

			// static i/o functions
			static CollisionShape* ReadCollisionShape(istream& stream);
			static void WriteCollisionShape(CollisionShape* shape, ostream& stream);
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
		// TODO: implement this

		public:

			TriangleMeshShape();

			MassInfo ComputeMassInfo();
	};

	class InfinitePlaneShape : public CollisionShape
	{
		public:

			Plane plane;

			InfinitePlaneShape(const Plane& plane);

			MassInfo ComputeMassInfo();
	};
}
