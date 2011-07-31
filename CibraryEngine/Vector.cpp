#include "StdAfx.h"
#include "Vector.h"
#include "DebugLog.h"
#include "Serialize.h"

#include "Scripting.h"

namespace CibraryEngine
{
	/*
	 * Vec2 methods
	 */
	Vec2::Vec2() : x(0), y(0) { }
	Vec2::Vec2(float x, float y) : x(x), y(y) { }

	float Vec2::ComputeMagnitudeSquared() { return x * x + y * y; }
	float Vec2::ComputeMagnitude() { return sqrt(x * x + y * y); }
	float Vec2::Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

	float Vec2::Magnitude(float x, float y) { return sqrt(x * x + y * y); }
	float Vec2::MagnitudeSquared(float x, float y) { return x * x + y * y; }




	/*
	 * Vec2 operators
	 */
	Vec2 operator -(Vec2 a) { return Vec2(-a.x, -a.y); }
	void operator +=(Vec2& a, Vec2 b) { a.x += b.x; a.y += b.y; }
	Vec2 operator +(Vec2 a, Vec2 b) { Vec2 result(a); result += b; return result; }
	void operator -=(Vec2& a, Vec2 b) { a.x -= b.x; a.y -= b.y; }
	Vec2 operator -(Vec2 a, Vec2 b) { Vec2 result(a); result -= b; return result; }
	void operator *=(Vec2& a, float b) { a.x *= b; a.y *= b; }
	Vec2 operator *(Vec2 a, float b) { Vec2 result(a); result *= b; return result; }
	Vec2 operator *(float b, Vec2 a) { Vec2 result(a); result *= b; return result; }
	void operator /=(Vec2& a, float b) { a *= 1.0f / b; }
	Vec2 operator /(Vec2 a, float b) { Vec2 result(a); result /= b; return result; }
	bool operator ==(Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }




	/*
	 * Vec3 methods
	 */
	Vec3::Vec3() { x = y = z = 0.0; }
	Vec3::Vec3(float _x, float _y, float _z) { x = _x; y = _y; z = _z; }

	float Vec3::ComputeMagnitudeSquared() { return x * x + y * y + z * z; }
	float Vec3::ComputeMagnitude() { return sqrt(x * x + y * y + z * z); }

	float Vec3::Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

	Vec3 Vec3::Cross(Vec3 a, Vec3 b) { return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

	Vec3 Vec3::Normalize(Vec3 a) { return a * (1.0f / a.ComputeMagnitude()); }
	Vec3 Vec3::Normalize(Vec3 a, float len) { return a * (len / a.ComputeMagnitude()); }

	float Vec3::Magnitude(float x, float y, float z) { return sqrt(x * x + y * y + z * z); }
	float Vec3::MagnitudeSquared(float x, float y, float z) { return x * x + y * y + z * z; }




	/*
	 * Vec3 operators
	 */
	Vec3 operator -(Vec3 a) { return Vec3(-a.x, -a.y, -a.z); }
	void operator +=(Vec3& a, Vec3 b) { a.x += b.x; a.y += b.y; a.z += b.z; }
	Vec3 operator +(Vec3 a, Vec3 b) { Vec3 result(a); result += b; return result; }
	void operator -=(Vec3& a, Vec3 b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; }
	Vec3 operator -(Vec3 a, Vec3 b) { Vec3 result(a); result -= b; return result; }
	void operator *=(Vec3& a, float b) { a.x *= b; a.y *= b; a.z *= b; }
	Vec3 operator *(Vec3 a, float b) { Vec3 result(a); result *= b; return result; }
	Vec3 operator *(float b, Vec3 a) { Vec3 result(a); result *= b; return result; }
	void operator /=(Vec3& a, float b) { a *= 1.0f / b; }
	Vec3 operator /(Vec3 a, float b) { Vec3 result(a); result /= b; return result; }
	bool operator ==(Vec3 a, Vec3 b) { return a.x == b.x && a.y == b.y && a.z == b.z; }




	/*
	 * Vec4 methods
	 */
	Vec4::Vec4() { x = y = z = w = 0.0; }
	Vec4::Vec4(float _x, float _y, float _z, float _w) { x = _x; y = _y; z = _z; w = _w; }

	float Vec4::ComputeMagnitudeSquared() { return x * x + y * y + z * z + w * w; }
	float Vec4::ComputeMagnitude() { return sqrt(x * x + y * y + z * z + w * w); }

	float Vec4::Dot(Vec4 a, Vec4 b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }




	/*
	 * Vec4 operators
	 */
	Vec4 operator -(Vec4 a) { return Vec4(-a.x, -a.y, -a.z, -a.w); }
	void operator +=(Vec4& a, Vec4 b) { a.x += b.x; a.y += b.y; a.z += b.z; a.w += b.w; }
	Vec4 operator +(Vec4 a, Vec4 b) { Vec4 result(a); result += b; return result; }
	void operator -=(Vec4& a, Vec4 b) { a.x -= b.x; a.y -= b.y; a.z -= b.z; a.w -= b.w; }
	Vec4 operator -(Vec4 a, Vec4 b) { Vec4 result(a); result -= b; return result; }
	void operator *=(Vec4& a, float b) { a.x *= b; a.y *= b; a.z *= b; a.w *= b; }
	Vec4 operator *(Vec4 a, float b) { Vec4 result(a); result *= b; return result; }
	Vec4 operator *(float b, Vec4 a) { Vec4 result(a); result *= b; return result; }
	void operator /=(Vec4& a, float b) { a *= 1.0f / b; }
	Vec4 operator /(Vec4 a, float b) { Vec4 result(a); result /= b; return result; }
	bool operator ==(Vec4 a, Vec4 b) { return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w; }




	/*
	 * Vector serialization functions
	 */
	void WriteVec2(Vec2& vec, ostream& stream)
	{
		WriteSingle(vec.x, stream);
		WriteSingle(vec.y, stream);
	}
	void WriteVec3(Vec3& vec, ostream& stream)
	{
		WriteSingle(vec.x, stream);
		WriteSingle(vec.y, stream);
		WriteSingle(vec.z, stream);
	}
	void WriteVec4(Vec4& vec, ostream& stream)
	{
		WriteSingle(vec.x, stream);
		WriteSingle(vec.y, stream);
		WriteSingle(vec.z, stream);
		WriteSingle(vec.w, stream);
	}
	Vec2 ReadVec2(istream& stream)
	{
		float x = ReadSingle(stream);
		float y = ReadSingle(stream);
		return Vec2(x, y);
	}
	Vec3 ReadVec3(istream& stream)
	{
		float x = ReadSingle(stream);
		float y = ReadSingle(stream);
		float z = ReadSingle(stream);
		return Vec3(x, y, z);
	}
	Vec4 ReadVec4(istream& stream)
	{
		float x = ReadSingle(stream);
		float y = ReadSingle(stream);
		float z = ReadSingle(stream);
		float w = ReadSingle(stream);
		return Vec4(x, y, z, w);
	}




	/*
	 * Vec3 scripting stuff
	 */
	void PushLuaVector(lua_State* L, Vec3 vec);

	int ba_vector_index(lua_State* L)
	{
		Vec3* vec = (Vec3*)lua_touserdata(L, 1);

		if(lua_isstring(L, 2))
		{
			string key = lua_tostring(L, 2);

			lua_settop(L, 0);

			if		(key == "x") { lua_pushnumber(L, vec->x); return 1; }
			else if	(key == "y") { lua_pushnumber(L, vec->y); return 1; }
			else if	(key == "z") { lua_pushnumber(L, vec->z); return 1; }
			else if	(key == "length") { lua_settop(L, 0); lua_pushnumber(L, vec->ComputeMagnitude()); return 1; }
		}

		return 0;
	}

	int ba_vector_newindex(lua_State* L)
	{
		Vec3* vec = (Vec3*)lua_touserdata(L, 1);

		if(lua_isstring(L, 2) && lua_isnumber(L, 3))
		{
			string key = lua_tostring(L, 2);

			lua_settop(L, 0);

			if		(key == "x") { vec->x = (float)lua_tonumber(L, 3); }
			else if	(key == "y") { vec->y = (float)lua_tonumber(L, 3); }
			else if	(key == "z") { vec->z = (float)lua_tonumber(L, 3); }
		}

		return 0;
	}

	int ba_vector_eq(lua_State* L)
	{
		Vec3 a = *(Vec3*)lua_touserdata(L, 1);
		Vec3 b = *(Vec3*)lua_touserdata(L, 2);

		lua_settop(L, 0);

		lua_pushboolean(L, (a == b));
		return 1;
	}

	int ba_vector_add(lua_State* L)
	{
		Vec3 a = *(Vec3*)lua_touserdata(L, 1);
		Vec3 b = *(Vec3*)lua_touserdata(L, 2);

		lua_settop(L, 0);

		PushLuaVector(L, a + b);
		return 1;
	}

	int ba_vector_unm(lua_State* L)
	{
		Vec3 a = *(Vec3*)lua_touserdata(L, 1);

		lua_settop(L, 0);

		PushLuaVector(L, -a);
		return 1;
	}

	int ba_vector_sub(lua_State* L)
	{
		Vec3 a = *(Vec3*)lua_touserdata(L, 1);
		Vec3 b = *(Vec3*)lua_touserdata(L, 2);

		lua_settop(L, 0);

		PushLuaVector(L, a - b);
		return 1;
	}

	int ba_vector_mul(lua_State* L)
	{
		Vec3 a = *(Vec3*)lua_touserdata(L, 1);
		float b = (float)lua_tonumber(L, 2);

		lua_settop(L, 0);

		PushLuaVector(L, a * b);
		return 1;
	}

	int ba_vector_div(lua_State* L)
	{
		Vec3 a = *(Vec3*)lua_touserdata(L, 1);
		float b = (float)lua_tonumber(L, 2);

		lua_settop(L, 0);

		PushLuaVector(L, a / b);
		return 1;
	}

	int ba_vector_tostring(lua_State* L)
	{
		Vec3 a = *(Vec3*)lua_touserdata(L, 1);

		lua_pop(L, 1);			// pop; stack = 0 (normally)

		stringstream ss;
		ss << "[" << a.x << " " << a.y << " " << a.z << "]";
		lua_pushstring(L, ss.str().c_str());

		return 1;
	}

	int ba_createVector(lua_State* L)
	{
		int n = lua_gettop(L);

		Vec3 vec;

		if(n == 0 || n == 3)
		{
			if(n == 3 && lua_isnumber(L, 1) && lua_isnumber(L, 2) && lua_isnumber(L, 3))
				vec = Vec3((float)lua_tonumber(L, 1), (float)lua_tonumber(L, 2), (float)lua_tonumber(L, 3));
			else
				vec = Vec3();

			PushLuaVector(L, vec);

			return 1;
		}

		Debug("ba.createVector takes either no parameters, or 3 numbers as parameters; returning nil\n");
		return 0;
	}




	void PushLuaVector(lua_State* L, Vec3 vec)
	{
		lua_settop(L, 0);

		Vec3* ptr = (Vec3*)lua_newuserdata(L, sizeof(Vec3));			// push; top = 1
		*ptr = vec;

		lua_getglobal(L, "VectorMeta");
		if(lua_isnil(L, 2))
		{
			lua_pop(L, 1);
			// we must create the metatable
			lua_newtable(L);												// push; top = 2

			lua_pushcclosure(L, ba_vector_index, 0);						// push; top = 3
			lua_setfield(L, 2, "__index");									// pop; top = 2

			lua_pushcclosure(L, ba_vector_newindex, 0);						// push; top = 3
			lua_setfield(L, 2, "__newindex");								// pop; top = 2

			lua_pushcclosure(L, ba_vector_eq, 0);
			lua_setfield(L, 2, "__eq");

			lua_pushcclosure(L, ba_vector_add, 0);
			lua_setfield(L, 2, "__add");

			lua_pushcclosure(L, ba_vector_unm, 0);
			lua_setfield(L, 2, "__unm");

			lua_pushcclosure(L, ba_vector_sub, 0);
			lua_setfield(L, 2, "__sub");

			lua_pushcclosure(L, ba_vector_mul, 0);
			lua_setfield(L, 2, "__mul");

			lua_pushcclosure(L, ba_vector_div, 0);
			lua_setfield(L, 2, "__div");

			lua_pushcclosure(L, ba_vector_tostring, 0);
			lua_setfield(L, 2, "__tostring");

			lua_pushcclosure(L, ba_generic_concat, 0);
			lua_setfield(L, 2, "__concat");

			lua_setglobal(L, "VectorMeta");
			lua_getglobal(L, "VectorMeta");
		}

		lua_setmetatable(L, 1);											// set field of 1; pop; top = 1
	}
}

