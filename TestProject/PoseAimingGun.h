#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class PoseAimingGun : public Pose
	{
	public:

		Quaternion pelvis_ori;
		Vec3 aim_dir;

		unsigned int torso1, torso2;

		PoseAimingGun() :
			Pose(),
			pelvis_ori(Quaternion::Identity()),
			aim_dir(0, 0, 1),
			torso1( Bone::string_table[ "torso 1"] ),
			torso2( Bone::string_table[ "torso 2"] )
		{
		}

		void UpdatePose(TimingInfo time)
		{
			if(time.total < 0.1f)
				return;

			Vec3 relative_aim = Vec3::Normalize(Quaternion::Reverse(pelvis_ori) * aim_dir);

			// TODO: do the rest of this better (e.g. implement that 3x3 grid of poses idea)
			float yaw   = -atan2f(relative_aim.x, relative_aim.z);
			float pitch =  asinf(relative_aim.y);

			float x =  0.1f * pitch;
			float y = -0.5f * yaw;
			float z = -0.4f * pitch;
			Vec3 xyz(x, y, z);

			SetBonePose(torso1, xyz, Vec3());
			SetBonePose(torso2, xyz, Vec3());

			ScriptingState s = ScriptSystem::GetGlobalState();
			if(!s.DoFile("Files/Scripts/pose_aiming_gun.lua"))
			{
				lua_State* L = s.GetLuaState();
				if(lua_gettop(L) == 1 && lua_istable(L, 1))
				{
					unsigned int num_entries = lua_objlen(L, 1);
					for(unsigned int i = 1; i <= num_entries; ++i)
					{
						lua_pushinteger(L, i);
						lua_gettable(L, 1);

						if(lua_istable(L,-1))
						{
							string bone_name;
							Vec3 ori;

							lua_pushinteger(L, 1);
							lua_gettable(L, -2);
							if(lua_isstring(L, -1))
								bone_name = lua_tostring(L, -1);
							lua_pop(L, 1);

							lua_pushinteger(L, 2);
							lua_gettable(L, -2);
							if(lua_isnumber(L, -1))
								ori.x = (float)lua_tonumber(L, -1);
							lua_pop(L, 1);

							lua_pushinteger(L, 3);
							lua_gettable(L, -2);
							if(lua_isnumber(L, -1))
								ori.y = (float)lua_tonumber(L, -1);
							lua_pop(L, 1);

							lua_pushinteger(L, 4);
							lua_gettable(L, -2);
							if(lua_isnumber(L, -1))
								ori.z = (float)lua_tonumber(L, -1);
							lua_pop(L, 1);

							if(!bone_name.empty())
								SetBonePose(Bone::string_table[bone_name], ori, Vec3());
						}
						lua_pop(L, 1);
					}
				}
			}
		}
	};
}
