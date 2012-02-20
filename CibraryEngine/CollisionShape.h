#pragma once

#include "StdAfx.h"

#include "Disposable.h"

namespace CibraryEngine
{
	using namespace std;

	struct MassInfo;

	/** A shape usable for collision detection and/or response */
	class CollisionShape : public Disposable
	{
		protected:

			CollisionShape();

			virtual void InnerDispose();

		public:

			/** Compute the mass info for this shape, assuming a density of 1 */
			virtual MassInfo ComputeMassInfo();


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

		TriangleMeshShape();

		MassInfo ComputeMassInfo();
	};
}
