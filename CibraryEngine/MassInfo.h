#pragma once
#include "StdAfx.h"

#include "Vector.h"

namespace CibraryEngine
{
	class CollisionShape;

	/** Class representing the mass properties of an object */
	struct MassInfo
	{
		/** The mass of the object */
		float mass;

		/** The object's center of mass */
		Vec3 com;

		/** The object's 3D moment of inertia about an axis passing through its center of mass */
		float moi[9];

		/** Initializes a zero mass */
		MassInfo();
		/** Initializes a MassInfo representing a point mass at the origin */
		MassInfo(float mass);
		/** Initializes a MassInfo representing a point mass, at the specified location */
		MassInfo(const Vec3& pos, float mass);

		/** Adds two MassInfo objects */
		void operator +=(const MassInfo& other);
		/** Adds two MassInfo objects */
		MassInfo operator +(const MassInfo& other);

		/** Scales the mass and MoI components of a MassInfo, leaving the CoM unchanged */
		void operator *=(float coeff);
		/** Scales the mass and MoI components of a MassInfo, leaving the CoM unchanged */
		MassInfo operator *(float coeff);

		/** Computes the moment of inertia about a parallel axis, with pivot point translated by the given vector (i.e. parallel axis theorem in 3 dimensions) */
		static void GetAlternatePivotMoI(const Vec3& a, const float* I, float m, float* result);

		/** Like GetAlternatePivotMoI, but adds to result instead of setting it */
		static void AddAlternatePivotMoI(const Vec3& a, const float* I, float m, float* result);

		/** Computes the sum of an array of MassInfo objects */
		static MassInfo Sum(const MassInfo* arr, unsigned int count);

		static MassInfo FromCollisionShape(CollisionShape* shape, float mass);

		// serialization and deserialization functions
		static MassInfo ReadMassInfo(istream& stream);
		void Write(ostream& stream);
	};
}
