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
		Vec2();
		/** Initializes a vector with the specified x and y components */
		Vec2(float x, float y);

		/** Returns the square of the magnitude of the vector (one less step than taking a square root) */
		float ComputeMagnitudeSquared();
		/** Returns the magnitude of the vector */
		float ComputeMagnitude();

		/** Computes the dot product of two vectors */
		static float Dot(Vec2 a, Vec2 b);

		/** Computes the magnitude of a vector with the given x and y components */
		static float Magnitude(float x, float y);
		/** Computes the square of the magnitude of a vector with the given x and y components (one less step than taking a square root) */
		static float MagnitudeSquared(float x, float y);
	};

	/** Returns the opposite of the vector */
	Vec2 operator -(Vec2 a);
	/** Adds two vectors */
	void operator +=(Vec2& a, Vec2 b);
	/** Adds two vectors */
	Vec2 operator +(Vec2 a, Vec2 b);
	/** Subtracts two vectors */
	void operator -=(Vec2& a, Vec2 b);
	/** Subtracts two vectors */
	Vec2 operator -(Vec2 a, Vec2 b);
	/** Multiplies a vector by a scalar */
	void operator *=(Vec2& a, float b);
	/** Multiplies a vector by a scalar */
	Vec2 operator *(Vec2 a, float b);
	/** Multiplies a vector by a scalar */
	Vec2 operator *(float b, Vec2 a);
	/** Divides a vector by a scalar */
	void operator /=(Vec2& a, float b);
	/** Divides a vector by a scalar */
	Vec2 operator /(Vec2 a, float b);
	/** Tests whether two vectors are equal */
	bool operator ==(Vec2 a, Vec2 b);

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
		Vec3();
		/** Initializes a vector with the specified x, y, and z components */
		Vec3(float x, float y, float z);

		/** Returns the square of the magnitude of the vector (one less step than taking a square root) */
		float ComputeMagnitudeSquared();
		/** Returns the magnitude of the vector */
		float ComputeMagnitude();

		

		/** Computes the dot product of two vectors */
		static float Dot(Vec3 a, Vec3 b);
		/** Computes the cross product of two vectors */
		static Vec3 Cross(Vec3 a, Vec3 b);

		/** Returns a unit vector parallel to the given vector */
		static Vec3 Normalize(Vec3 a);
		/** Returns a vector parallel to the given vector with the specified magnitude */
		static Vec3 Normalize(Vec3 a, float len);

		/** Computes the magnitude of a vector with the given x, y, and z components */
		static float Magnitude(float x, float y, float z);
		/** Computes the square of the magnitude of a vector with the given x, y, and z components (one less step than taking a square root) */
		static float MagnitudeSquared(float x, float y, float z);
	};

	/** Returns the opposite of the vector */
	Vec3 operator -(Vec3 a);
	/** Adds two vectors */
	void operator +=(Vec3& a, Vec3 b);
	/** Adds two vectors */
	Vec3 operator +(Vec3 a, Vec3 b);
	/** Subtracts two vectors */
	void operator -=(Vec3& a, Vec3 b);
	/** Subtracts two vectors */
	Vec3 operator -(Vec3 a, Vec3 b);
	/** Multiplies a vector by a scalar */
	void operator *=(Vec3& a, float b);
	/** Multiplies a vector by a scalar */
	Vec3 operator *(Vec3 a, float b);
	/** Multiplies a vector by a scalar */
	Vec3 operator *(float b, Vec3 a);
	/** Divides a vector by a scalar */
	void operator /=(Vec3& a, float b);
	/** Divides a vector by a scalar */
	Vec3 operator /(Vec3 a, float b);
	/** Tests whether two vectors are equal */
	bool operator ==(Vec3 a, Vec3 b);

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
		Vec4();
		/** Initializes a vector with the specified x, y, z, and w components */
		Vec4(Vec3 xyz, float w);
		/** Initializes a vector with the specified x, y, z, and w components */
		Vec4(float x, float y, float z, float w);

		/** Returns the square of the magnitude of the vector (one less step than taking a square root) */
		float ComputeMagnitudeSquared();
		/** Returns the magnitude of the vector */
		float ComputeMagnitude();

		
		/** Computes the dot product of two vectors */
		static float Dot(Vec4 a, Vec4 b);
	};

	/** Returns the opposite of the vector */
	Vec4 operator -(Vec4 a);
	/** Adds two vectors */
	void operator +=(Vec4& a, Vec4 b);
	/** Adds two vectors */
	Vec4 operator +(Vec4 a, Vec4 b);
	/** Subtracts two vectors */
	void operator -=(Vec4& a, Vec4 b);
	/** Subtracts two vectors */
	Vec4 operator -(Vec4 a, Vec4 b);
	/** Multiplies a vector by a scalar */
	void operator *=(Vec4& a, float b);
	/** Multiplies a vector by a scalar */
	Vec4 operator *(Vec4 a, float b);
	/** Multiplies a vector by a scalar */
	Vec4 operator *(float b, Vec4 a);
	/** Divides a vector by a scalar */
	void operator /=(Vec4& a, float b);
	/** Divides a vector by a scalar */
	Vec4 operator /(Vec4 a, float b);
	/** Tests whether two vectors are equal */
	bool operator ==(Vec4 a, Vec4 b);


	void WriteVec2(Vec2& vec, ostream& stream);
	void WriteVec3(Vec3& vec, ostream& stream);
	void WriteVec4(Vec4& vec, ostream& stream);

	Vec2 ReadVec2(istream& stream);
	Vec3 ReadVec3(istream& stream);
	Vec4 ReadVec4(istream& stream);




	void PushLuaVector(lua_State* L, Vec3 vec);
	int ba_createVector(lua_State* L);
}
