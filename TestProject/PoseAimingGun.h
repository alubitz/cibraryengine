#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	class PoseAimingGun : public Pose
	{

	public:

		float yaw, pitch;

		unsigned int torso1, torso2, head;
		unsigned int larm1, larm2, lhand;
		unsigned int rarm1, rarm2, rhand;

		PoseAimingGun() :
			Pose(),
			yaw(),
			pitch(),
			torso1(	Bone::string_table["torso 1"]	),
			torso2(	Bone::string_table["torso 2"]	),
			head(	Bone::string_table["head"]		),
			larm1(	Bone::string_table["l arm 1"]	),
			larm2(	Bone::string_table["l arm 2"]	),
			lhand(	Bone::string_table["l hand"]	),
			rarm1(	Bone::string_table["r arm 1"]	),
			rarm2(	Bone::string_table["r arm 2"]	),
			rhand(	Bone::string_table["r hand"]	)			
		{
		}

		void UpdatePose(TimingInfo time)
		{
			if(time.total < 0.1f)
				return;

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

			/*
			SetBonePose(torso1,	Vec3(	 0.0f,		 0.0f,		 0.0f	), Vec3());
			SetBonePose(torso2,	Vec3(	 0.0f,		 0.0f,		 0.0f	), Vec3());
			SetBonePose(head,	Vec3(	 0.0f,		 0.0f,		 0.0f	), Vec3());

			SetBonePose(larm1,	Vec3(	 0.6f,		-1.0f,		 0.4f	), Vec3());
			SetBonePose(larm2,	Vec3(	-0.3f,		-0.6f,		 0.0f	), Vec3());
			SetBonePose(lhand,	Vec3(	 0.0f,		 0.0f,		 1.0f	), Vec3());

			SetBonePose(rarm1,	Vec3(	 0.4f,		 1.0f,		 0.6f	), Vec3());
			SetBonePose(rarm2,	Vec3(	-0.5f,		 1.0f,		 0.0f	), Vec3());
			SetBonePose(rhand,	Vec3(	 0.0f,		 0.0f,		-0.5f	), Vec3());
			*/
		}
	};
}
