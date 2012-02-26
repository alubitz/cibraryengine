#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct Mat3;
	struct Vec3;

	/** Class representing a quaternion */
	struct Quaternion
	{
		/** The w-component of the quaternion */
		float w;
		/** The x-component of the quaternion */
		float x;
		/** The y-component of the quaternion */
		float y;
		/** The z-component of the quaternion */
		float z;

		/** Initializes a zero quaternion. CAUTION! This is not the same as the identity quaternion! */
		Quaternion();
		/** Initializes a quaternion with the specified w, x, y, and z values */
		Quaternion(float w, float x, float y, float z);

		/** Returns the norm (comparable to magnitude) of this quaternion */
		float Norm() const;
		/** Returns a 3-component vector parallel to the axis of rotation represented by this quaternion, whose magnitude is the angle of rotation */
		Vec3 ToPYR() const;

		/** Returns a 3x3 rotation matrix representing the same rotation as this quaternion */
		Mat3 ToMat3() const;

		/** Returns the opposite of this quaternion */
		Quaternion operator -();
		/** Transforms (rotates) a 3-component vector by this quaternion */
		Vec3 operator *(Vec3 right);

		/** Transforms a quaternion by another quaternion (maybe the same as concatenation?) */
		Quaternion operator *(Quaternion right);
		/** Transforms a quaternion by another quaternion (maybe the same as concatenation?) */
		void operator *=(Quaternion right);

		/** Scales this quaternion by the specified amount */
		Quaternion operator *(float right);
		/** Scales this quaternion by the specified amount */
		void operator *=(float right);

		/** Inversely scales this quaternion by the specified amount */
		Quaternion operator /(float right);
		/** Inversely scales this quaternion by the specified amount */
		void operator /=(float right);

		/** Adds two quaternions */
		Quaternion operator +(Quaternion right);
		/** Adds two quaternions */
		void operator +=(Quaternion right);

		/** Subtracts two quaternions */
		Quaternion operator -(Quaternion right);
		/** Subtracts two quaternions */
		void operator -=(Quaternion right);

		/** Returns the identity quaternion */
		static Quaternion Identity();
		/** Returns a quaternion representing the same rotation as the specified 3x3 rotation matrix */
		static Quaternion FromRotationMatrix(Mat3 mat);
		/** Returns a quaternion representing a rotation about the specified axis (should be a unit vector) with the specified magnitude */
		static Quaternion FromAxisAngle(float x, float y, float z, float angle);
		/** Returns a quaternion representing a rotation about the specified axis vector, whose magnitude is the angle of the rotation */
		static Quaternion FromPYR(Vec3 pyrVector);
		/** Returns a quaternion representing a rotation about the specified axis vector, whose magnitude is the angle of the rotation */
		static Quaternion FromPYR(float p, float y, float r);
		/** Normalizes the given quaternion */
		static Quaternion Normalize(Quaternion q);
	};

	void WriteQuaternion(Quaternion& q, ostream& stream);
	Quaternion ReadQuaternion(istream& stream);
}
