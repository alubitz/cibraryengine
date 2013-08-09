#pragma once

#include "StdAfx.h"

#include "Vector.h"

namespace CibraryEngine
{
	using namespace std;

	struct Mat3;

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
		Quaternion() : x(0), y(0), z(0), w(0) { }
		/** Initializes a quaternion with the specified w, x, y, and z values */
		Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) { }

		/** Returns the square of the norm (comparable to magnitude) of this quaternion */
		float NormSquared() const								{ return w * w + x * x + y * y + z * z; }
		/** Returns the norm (comparable to magnitude) of this quaternion */
		float Norm() const										{ return sqrtf(w * w + x * x + y * y + z * z); }
		/** Returns a 3-component vector parallel to the axis of rotation represented by this quaternion, whose magnitude is the angle of rotation */
		Vec3 ToPYR() const
		{
			if(float magsq = Vec3::MagnitudeSquared(x, y, z))
			{
				float mag = sqrtf(magsq), half = atanf(mag / w), coeff = 2.0f * half / mag;
				return Vec3(x * coeff, y * coeff, z * coeff);
			}
			else
				return Vec3();
		}

		/** Returns a 3x3 rotation matrix representing the same rotation as this quaternion; assumes the quaternion is already normalized */
		Mat3 ToMat3() const;

		/** Returns the opposite of this quaternion */
		Quaternion operator -() const							{ return Quaternion(-w, -x, -y, -z); }
		/** Transforms (rotates) a 3-component vector by this quaternion */
		Vec3 operator *(const Vec3& v) const
		{
			// v' = v + 2r cross ((r cross v) + wv)     but the 2x is done by adding it twice instead of multiplying

			Vec3 temp(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);							// cross product of xyz with v
			temp += v * w;
			temp = Vec3(y * temp.z - z * temp.y, z * temp.x - x * temp.z, x * temp.y - y * temp.x);		// cross product of xyz with temp
			return v + temp + temp;
		}

		/** Transforms a quaternion by another quaternion (maybe the same as concatenation?) */
		Quaternion operator *(const Quaternion& right) const	{ Quaternion temp(*this); temp *= right; return temp; }
		/** Transforms a quaternion by another quaternion (maybe the same as concatenation?) */
		void operator *=(const Quaternion& q)
		{
			float w_ = w * q.w - x * q.x - y * q.y - z * q.z;
			float x_ = w * q.x + x * q.w + y * q.z - z * q.y;
			float y_ = w * q.y + y * q.w + z * q.x - x * q.z;
			float z_ = w * q.z + z * q.w + x * q.y - y * q.x;
			
			w = w_; x = x_; y = y_; z = z_;
		}

		/** Scales this quaternion by the specified amount */
		Quaternion operator *(float right) const				{ Quaternion temp(*this); temp *= right; return temp; }
		/** Scales this quaternion by the specified amount */
		void operator *=(float right)							{ w *= right; x *= right; y *= right; z *= right; }

		/** Inversely scales this quaternion by the specified amount */
		Quaternion operator /(float right) const				{ Quaternion temp(*this); temp /= right; return temp; }
		/** Inversely scales this quaternion by the specified amount */
		void operator /=(float right)							{ *this *= (1.0f / right); }

		/** Adds two quaternions */
		Quaternion operator +(const Quaternion& right) const	{ Quaternion temp(*this); temp += right; return temp; }
		/** Adds two quaternions */
		void operator +=(const Quaternion& right)				{ w += right.w; x += right.x; y += right.y; z += right.z; }

		/** Subtracts two quaternions */
		Quaternion operator -(const Quaternion& right) const	{ Quaternion temp(*this); temp -= right; return temp; }
		/** Subtracts two quaternions */
		void operator -=(const Quaternion& right)				{ w -= right.w; x -= right.x; y -= right.y; z -= right.z; }

		/** Test whether two quaternions are equal */
		bool operator ==(const Quaternion& b) const				{ return w == b.w && x == b.x && y == b.y && z == b.z; }
		/** Test whether two quaternions are unequal */
		bool operator !=(const Quaternion& b) const				{ return w != b.w || x != b.x || y != b.y || z != b.z; }

		/** Returns the identity quaternion */
		static Quaternion Identity()							{ return Quaternion(1.0f, 0.0, 0.0, 0.0); }
		/** Returns a quaternion representing the same rotation as the specified 3x3 rotation matrix */
		static Quaternion FromRotationMatrix(const Mat3& mat);
		/** Returns a quaternion representing a rotation about the specified axis (should be a unit vector) with the specified magnitude */
		static Quaternion FromAxisAngle(float x, float y, float z, float angle)
		{
			float half = angle * 0.5f, sine = sinf(half);
			return Quaternion(cosf(half), x * sine, y * sine, z * sine);
		}
		/** Returns a quaternion representing a rotation about the specified axis vector, whose magnitude is the angle of the rotation */
		static Quaternion FromPYR(const Vec3& pyrVector)							{ return Quaternion::FromPYR(pyrVector.x, pyrVector.y, pyrVector.z); }
		/** Returns a quaternion representing a rotation about the specified axis vector, whose magnitude is the angle of the rotation */
		static Quaternion FromPYR(float p, float y, float r)
		{
			if(float magsq = Vec3::MagnitudeSquared(p, y, r))
			{
				float mag = sqrtf(magsq), half = mag * 0.5f, coeff = sinf(half) / mag;
				return Quaternion(cosf(half), p * coeff, y * coeff, r * coeff);
			}
			else
				return Quaternion::Identity();
		}
		/** Normalizes the given quaternion */
		static Quaternion Normalize(const Quaternion& q)							{ return q / q.Norm(); }
		/** Inverts a quaternion */
		static Quaternion Invert(const Quaternion& q)
		{
			float inv = 1.0f / (q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
			return Quaternion(q.w * inv, -q.x * inv, -q.y * inv, -q.z * inv);
		}
		/** Reverses a quaternion rotation; like inverting a quaternion which is already normalized */
		static Quaternion Reverse(const Quaternion& q)								{ return Quaternion(q.w, -q.x, -q.y, -q.z); }
	};

	void WriteQuaternion(const Quaternion& q, ostream& stream);
	Quaternion ReadQuaternion(istream& stream);
}
