#include "StdAfx.h"
#include "Vector.h"
#include "DebugLog.h"
#include "Serialize.h"

#include "Scripting.h"

namespace CibraryEngine
{
	/*
	 * Vector serialization functions
	 */
	void WriteVec2(const Vec2& vec, ostream& stream)
	{
		WriteSingle(vec.x, stream);
		WriteSingle(vec.y, stream);
	}
	void WriteVec3(const Vec3& vec, ostream& stream)
	{
		WriteSingle(vec.x, stream);
		WriteSingle(vec.y, stream);
		WriteSingle(vec.z, stream);
	}
	void WriteVec4(const Vec4& vec, ostream& stream)
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
	void PushLuaVector(lua_State* L, const Vec3& vec);

	int ba_vector_index(lua_State* L)
	{
		Vec3* vec = (Vec3*)lua_touserdata(L, 1);

		if(lua_isstring(L, 2))
		{
			string key = lua_tostring(L, 2);

			//lua_settop(L, 0);

			if		(key == "x")      { lua_pushnumber(L, vec->x); return 1; }
			else if	(key == "y")      { lua_pushnumber(L, vec->y); return 1; }
			else if	(key == "z")      { lua_pushnumber(L, vec->z); return 1; }
			else if	(key == "length") { lua_pushnumber(L, vec->ComputeMagnitude()); return 1; }
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

		//lua_settop(L, 0);

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
		Vec3* aptr = (Vec3*)lua_touserdata(L, 1);
		Vec3* bptr = (Vec3*)lua_touserdata(L, 2);
		Vec3 a = *aptr;//*(Vec3*)lua_touserdata(L, 1);
		Vec3 b = *bptr;//*(Vec3*)lua_touserdata(L, 2);

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




	void PushLuaVector(lua_State* L, const Vec3& vec)
	{
		//lua_settop(L, 0);

		Vec3* ptr = (Vec3*)lua_newuserdata(L, sizeof(Vec3));				// push; top = 1
		*ptr = vec;

		lua_getglobal(L, "VectorMeta");
		if(lua_isnil(L, -1))
		{
			lua_pop(L, 1);
			// we must create the metatable
			lua_newtable(L);												// push; top = 2

			lua_pushcclosure(L, ba_vector_index, 0);						// push; top = 3
			lua_setfield(L, -2, "__index");									// pop; top = 2

			lua_pushcclosure(L, ba_vector_newindex, 0);						// push; top = 3
			lua_setfield(L, -2, "__newindex");								// pop; top = 2

			lua_pushcclosure(L, ba_vector_eq, 0);
			lua_setfield(L, -2, "__eq");

			lua_pushcclosure(L, ba_vector_add, 0);
			lua_setfield(L, -2, "__add");

			lua_pushcclosure(L, ba_vector_unm, 0);
			lua_setfield(L, -2, "__unm");

			lua_pushcclosure(L, ba_vector_sub, 0);
			lua_setfield(L, -2, "__sub");

			lua_pushcclosure(L, ba_vector_mul, 0);
			lua_setfield(L, -2, "__mul");

			lua_pushcclosure(L, ba_vector_div, 0);
			lua_setfield(L, -2, "__div");

			lua_pushcclosure(L, ba_vector_tostring, 0);
			lua_setfield(L, -2, "__tostring");

			lua_pushcclosure(L, ba_generic_concat, 0);
			lua_setfield(L, -2, "__concat");

			lua_setglobal(L, "VectorMeta");
			lua_getglobal(L, "VectorMeta");
		}

		lua_setmetatable(L, -2);											// set field of 1; pop; top = 1
	}
}

