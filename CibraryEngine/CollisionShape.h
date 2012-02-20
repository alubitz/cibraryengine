#pragma once

#include "StdAfx.h"

#include "Disposable.h"

namespace CibraryEngine
{
	using namespace std;

	/** A shape usable for collision detection and/or response */
	class CollisionShape : public Disposable
	{
		protected:

			CollisionShape();

			virtual void InnerDispose();

		public:

			/** Compute the volume of this collision shape. Default implementation returns 0 */
			virtual float ComputeVolume();


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

			float ComputeVolume();
	};

	class TriangleMeshShape : public CollisionShape
	{
		// TODO: implement this

		TriangleMeshShape();

		float ComputeVolume();
	};
}
