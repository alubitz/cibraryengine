#pragma once

#include "StdAfx.h"

namespace CibraryEngine
{
	using namespace std;

	struct Mat3;

	/** Class representing a 2-component vector */
	struct Vec2
	{
		/** The x-component of the vector */
		float x;
		/** The y-component of the vector */
		float y;

		/** Initializes a zero-vector */
		Vec2() : x(0), y(0) { }
		/** Initializes a vector with the specified x and y components */
		Vec2(float x, float y) : x(x), y(y) { }

		/** Returns the square of the magnitude of the vector (one less step than taking a square root) */
		float ComputeMagnitudeSquared()						{ return x * x + y * y; }
		/** Returns the magnitude of the vector */
		float ComputeMagnitude()							{ return sqrtf(x * x + y * y); }

		/** Computes the dot product of two vectors */
		static float Dot(const Vec2& a, const Vec2& b)		{ return a.x * b.x + a.y * b.y; }

		/** Computes the magnitude of a vector with the given x and y components */
		static float Magnitude(float x, float y)			{ return sqrtf(x * x + y * y); }
		/** Computes the square of the magnitude of a vector with the given x and y components (one less step than taking a square root) */
		static float MagnitudeSquared(float x, float y)		{ return x * x + y * y; }



		/** Returns the opposite of the vector */
		Vec2 operator -() const								{ return Vec2(-x, -y); }
		/** Adds two vectors */
		void operator +=(const Vec2& b)						{ x += b.x; y += b.y; }
		/** Adds two vectors */
		Vec2 operator +(const Vec2& b) const				{ Vec2 result(*this); result += b; return result; }
		/** Subtracts two vectors */
		void operator -=(const Vec2& b)						{ x -= b.x; y -= b.y; }
		/** Subtracts two vectors */
		Vec2 operator -(const Vec2& b) const				{ Vec2 result(*this); result -= b; return result; }
		/** Multiplies a vector by a scalar */
		void operator *=(float b)							{ x *= b; y *= b; }
		/** Multiplies a vector by a scalar */
		Vec2 operator *(float b) const						{ Vec2 result(*this); result *= b; return result; }
		/** Multiplies the components of two vectors */
		void operator *=(const Vec2& b)						{ x *= b.x; y *= b.y; }
		/** Multiplies the components of two vectors */
		Vec2 operator *(const Vec2& b) const				{ Vec2 result(*this); result *= b; return result; }
		/** Divides a vector by a scalar */
		void operator /=(float b)							{ *this *= 1.0f / b; }
		/** Divides a vector by a scalar */
		Vec2 operator /(float b) const						{ Vec2 result(*this); result /= b; return result; }
		/** Divide the components of one vector by those of another */
		void operator /=(const Vec2& b)						{ x /= b.x; y /= b.y; }
		/** Divide the components of one vector by those of another */
		Vec2 operator /(const Vec2& b) const				{ Vec2 result(*this); result /= b; return result; }
		/** Tests whether two vectors are equal */
		bool operator ==(const Vec2& b) const				{ return x == b.x && y == b.y; }
	};

	/** Multiplies a vector by a scalar */
	static Vec2 operator *(float b, const Vec2& a)			{ Vec2 result(a); result *= b; return result; }

	

	/** Class representing a 3-component vector */
	struct Vec3
	{
		/** The x-component of the vector */
		float x;
		/** The y-component of the vector */
		float y;
		/** The z-component of the vector */
		float z;

		/** Initializes a zero vector */
		Vec3() : x(0), y(0), z(0) { }
		/** Initializes a vector with the specified x, y, and z components */
		Vec3(float x, float y, float z) : x(x), y(y), z(z) { }

		/** Returns the square of the magnitude of the vector (one less step than taking a square root) */
		float ComputeMagnitudeSquared() const				{ return x * x + y * y + z * z; }
		/** Returns the magnitude of the vector */
		float ComputeMagnitude() const						{ return sqrtf(x * x + y * y + z * z); }

		

		/** Computes the dot product of two vectors */
		static float Dot(const Vec3& a, const Vec3& b)		{ return a.x * b.x + a.y * b.y + a.z * b.z; }
		/** Computes the cross product of two vectors */
		static Vec3 Cross(const Vec3& a, const Vec3& b)		{ return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

		/** Returns a unit vector parallel to the given vector */
		static Vec3 Normalize(const Vec3& a)				{ return a * (1.0f / a.ComputeMagnitude()); }
		/** Returns a vector parallel to the given vector with the specified magnitude */
		static Vec3 Normalize(const Vec3& a, float len)		{ return a * (len / a.ComputeMagnitude()); }

		/** Computes the magnitude of a vector with the given x, y, and z components */
		static float Magnitude(float x, float y, float z)				{ return sqrtf(x * x + y * y + z * z); }
		/** Computes the square of the magnitude of a vector with the given x, y, and z components (one less step than taking a square root) */
		static float MagnitudeSquared(float x, float y, float z)		{ return x * x + y * y + z * z; }




		/** Returns the opposite of the vector */
		Vec3 operator -() const								{ return Vec3(-x, -y, -z); }
		/** Adds two vectors */
		void operator +=(const Vec3& b)						{ x += b.x; y += b.y; z += b.z; }
		/** Adds two vectors */
		Vec3 operator +(const Vec3& b) const				{ Vec3 result(*this); result += b; return result; }
		/** Subtracts two vectors */
		void operator -=(const Vec3& b)						{ x -= b.x; y -= b.y; z -= b.z; }
		/** Subtracts two vectors */
		Vec3 operator -(const Vec3& b) const				{ Vec3 result(*this); result -= b; return result; }
		/** Multiplies a vector by a scalar */
		void operator *=(float b)							{ x *= b; y *= b; z *= b; }
		/** Multiplies a vector by a scalar */
		Vec3 operator *(float b) const						{ Vec3 result(*this); result *= b; return result; }
		/** Multiplies the components of two vectors */
		void operator *=(const Vec3& b)						{ x *= b.x; y *= b.y; z *= b.z; }
		/** Multiplies the components of two vectors */
		Vec3 operator *(const Vec3& b) const				{ Vec3 result(*this); result *= b; return result; }
		/** Divides a vector by a scalar */
		void operator /=(float b)							{ *this *= 1.0f / b; }
		/** Divides a vector by a scalar */
		Vec3 operator /(float b) const						{ Vec3 result(*this); result /= b; return result; }
		/** Divide the components of one vector by those of another */
		void operator /=(const Vec3& b)						{ x /= b.x; y /= b.y; z /= b.z; }
		/** Divide the components of one vector by those of another */
		Vec3 operator /(const Vec3& b) const				{ Vec3 result(*this); result /= b; return result; }
		/** Tests whether two vectors are equal */
		bool operator ==(const Vec3& b) const				{ return x == b.x && y == b.y && z == b.z; }
	};

	/** Multiplies a vector by a scalar */
	static Vec3 operator *(float b, const Vec3& a)			{ Vec3 result(a); result *= b; return result; }


	/** Class representing a 4-component vector */
	struct Vec4
	{
		/** The x-component of the vector */
		float x;
		/** The y-component of the vector */
		float y;
		/** The z-component of the vector */
		float z;
		/** The w-component of the vector */
		float w;

		/** Initializes a zero vector */
		Vec4() : x(0), y(0), z(0), w(0) { }
		/** Initializes a vector with the specified x, y, z, and w components */
		Vec4(const Vec3& xyz, float w) : x(xyz.x), y(xyz.y), z(xyz.z), w(w) { }
		/** Initializes a vector with the specified x, y, z, and w components */
		Vec4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) { }

		/** Returns the square of the magnitude of the vector (one less step than taking a square root) */
		float ComputeMagnitudeSquared()						{ return x * x + y * y + z * z + w * w; }
		/** Returns the magnitude of the vector */
		float ComputeMagnitude()							{ return sqrtf(x * x + y * y + z * z + w * w); }

		
		/** Computes the dot product of two vectors */
		static float Dot(const Vec4& a, const Vec4& b)		{ return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }




		/** Returns the opposite of the vector */
		Vec4 operator -() const								{ return Vec4(-x, -y, -z, -w); }
		/** Adds two vectors */
		void operator +=(const Vec4& b)						{ x += b.x; y += b.y; z += b.z; w += b.w; }
		/** Adds two vectors */
		Vec4 operator +(const Vec4& b) const				{ Vec4 result(*this); result += b; return result; }
		/** Subtracts two vectors */
		void operator -=(const Vec4& b)						{ x -= b.x; y -= b.y; z -= b.z; w -= b.w; }
		/** Subtracts two vectors */
		Vec4 operator -(const Vec4& b) const				{ Vec4 result(*this); result -= b; return result; }
		/** Multiplies a vector by a scalar */
		void operator *=(float b)							{ x *= b; y *= b; z *= b; w *= b; }
		/** Multiplies a vector by a scalar */
		Vec4 operator *(float b) const						{ Vec4 result(*this); result *= b; return result; }
		/** Multiplies the components of two vectors */
		void operator *=(const Vec4& b)						{ x *= b.x; y *= b.y; z *= b.z; w *= b.w; }
		/** Multiplies the components of two vectors */
		Vec4 operator *(const Vec4& b) const				{ Vec4 result(*this); result *= b; return result; }
		/** Divides a vector by a scalar */
		void operator /=(float b)							{ *this *= 1.0f / b; }
		/** Divides a vector by a scalar */
		Vec4 operator /(float b) const						{ Vec4 result(*this); result /= b; return result; }
		/** Divide the components of one vector by those of another */
		void operator /=(const Vec4& b)						{ x /= b.x; y /= b.y; z /= b.z; w /= b.w; }
		/** Divide the components of one vector by those of another */
		Vec4 operator /(const Vec4& b) const				{ Vec4 result(*this); result /= b; return result; }
		/** Tests whether two vectors are equal */
		bool operator ==(const Vec4& b) const				{ return x == b.x && y == b.y && z == b.z && w == b.w; }
	};

	/** Multiplies a vector by a scalar */
	static Vec4 operator *(float b, const Vec4& a)			{ Vec4 result(a); result *= b; return result; }
	



	void WriteVec2(const Vec2& vec, ostream& stream);
	void WriteVec3(const Vec3& vec, ostream& stream);
	void WriteVec4(const Vec4& vec, ostream& stream);

	Vec2 ReadVec2(istream& stream);
	Vec3 ReadVec3(istream& stream);
	Vec4 ReadVec4(istream& stream);




	void PushLuaVector(lua_State* L, const Vec3& vec);
	int ba_createVector(lua_State* L);
}
