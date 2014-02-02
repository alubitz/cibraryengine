#include "StdAfx.h"
#include "PoseAimingGun.h"

namespace Test
{
	/*
	 * PoseAimingGun private implementation struct
	 */
	struct PoseAimingGun::Imp
	{
		struct AimPose
		{
			Keyframe pose;
			Vec3 dir;
		};
		vector<AimPose> poses;

		struct PoseTri
		{
			unsigned int indices[3];		// indices into poses
			Vec3 bounds[3];					// like bounding planes, but the offset from the origin is 0
			float opposites[3];				// distance from bounding plane to opposite vert
		};
		vector<PoseTri> tris;

		bool loaded_poses;

		Imp() : poses(), tris(), loaded_poses(false) { }
		~Imp() { }

		void LoadPoses()
		{
			ScriptingState s = ScriptSystem::GetGlobalState();
			if(!s.DoFile("Files/Scripts/pose_aiming_gun.lua"))
			{
				lua_State* L = s.GetLuaState();

				// the script should have returned an array containing some pose entries
				if(lua_gettop(L) == 1 && lua_istable(L, -1))
				{
					unsigned int num_poses = lua_objlen(L, -1);
					for(unsigned int p = 1; p <= num_poses; ++p)
					{
						lua_pushinteger(L, p);
						lua_gettable(L, -2);

						if(lua_istable(L, -1))
						{
							// each pose entry has two parts: the pose data itself, and the forward vector it should produce relative to the pelvis ori
							poses.push_back(AimPose());
							AimPose& pose = *poses.rbegin();

							lua_pushstring(L, "pose");
							lua_gettable(L, -2);

							// the pose is an array of 4-element tuples, containing: the name of a child bone, and the x,y,z of the joint's rvec
							if(lua_istable(L, -1))
							{
								unsigned int num_entries = lua_objlen(L, -1);
								for(unsigned int i = 1; i <= num_entries; ++i)
								{
									lua_pushinteger(L, i);
									lua_gettable(L, -2);

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
											pose.pose.values[Bone::string_table[bone_name]] = BoneInfluence(ori, Vec3());
									}
									lua_pop(L, 1);
								}
							}
							lua_pop(L, 1);

							// also get the forward vector for this pose
							lua_pushstring(L, "fwd");
							lua_gettable(L, -2);

							if(lua_isuserdata(L, -1))
								pose.dir = *(Vec3*)lua_touserdata(L, -1);

							lua_pop(L, 1);
						}
						lua_pop(L, 1);
					}
				}
			}
		}

		void ExtractTri(unsigned int a, unsigned int b, unsigned int c)
		{
			tris.push_back(PoseTri());
			PoseTri& tri = *tris.rbegin();

			tri.indices[0] = a;
			tri.indices[1] = b;
			tri.indices[2] = c;

			tri.bounds[0] = Vec3::Normalize(Vec3::Cross(poses[tri.indices[0]].dir, poses[tri.indices[1]].dir));
			tri.bounds[1] = Vec3::Normalize(Vec3::Cross(poses[tri.indices[1]].dir, poses[tri.indices[2]].dir));
			tri.bounds[2] = Vec3::Normalize(Vec3::Cross(poses[tri.indices[2]].dir, poses[tri.indices[0]].dir));

			tri.opposites[0] = Vec3::Dot(tri.bounds[0], poses[tri.indices[2]].dir);
			tri.opposites[1] = Vec3::Dot(tri.bounds[1], poses[tri.indices[0]].dir);
			tri.opposites[2] = Vec3::Dot(tri.bounds[2], poses[tri.indices[1]].dir);
		}

		void ExtracteGrid()
		{
			if(poses.size() >= 9)				// not sure why there would be more than 9, but whatever
			{
				ExtractTri(0, 1, 3);
				ExtractTri(1, 4, 3);
				ExtractTri(1, 2, 4);
				ExtractTri(2, 5, 4);
				ExtractTri(3, 4, 6);
				ExtractTri(4, 7, 6);
				ExtractTri(4, 5, 7);
				ExtractTri(5, 7, 8);
			}
		}
	};




	/*
	 * PoseAimingGun methods
	 */
	PoseAimingGun::PoseAimingGun() :
		Pose(),
		imp(new Imp()),
		pelvis_ori(Quaternion::Identity()),
		aim_dir(0, 0, 1)
	{
	}

	PoseAimingGun::~PoseAimingGun() { if(imp) { delete imp; imp = NULL; } }



	void PoseAimingGun::UpdatePose(const TimingInfo& time)
	{
		if(!imp->loaded_poses)
		{
			imp->LoadPoses();
			imp->ExtracteGrid();

			imp->loaded_poses = true;
		}

		if(time.total < 0.1f)
			return;

		// TODO: reasonable way of picking weights goes here
		Vec3 relative_aim = Vec3::Normalize(Quaternion::Reverse(pelvis_ori) * aim_dir);

		bool any = false;
		float scores[3];
		for(unsigned int i = 0; i < imp->tris.size(); ++i)
		{
			const Imp::PoseTri& tri = imp->tris[i];

			unsigned int j;
			for(j = 0; j < 3; ++j)
			{
				scores[j] = Vec3::Dot(tri.bounds[j], relative_aim);
				if(scores[j] < 0.0f)
					break;
			}
			if(j == 3)
			{
				if(any)
					Debug("multiple solutions zomgwtfbbq!\n");

				unordered_map<unsigned int, Vec3> bone_values;
				float total_weight = 0.0f;

				for(unsigned int k = 0; k < 3; ++k)
				{
					const Imp::AimPose& aimp = imp->poses[tri.indices[k]];

					unsigned int edge = (k + 1) % 3;
					float weight = scores[edge] / tri.opposites[edge];
					if(weight > 0.0f)
					{
						total_weight += weight;
						for(unordered_map<unsigned int, BoneInfluence>::const_iterator iter = aimp.pose.values.begin(); iter != aimp.pose.values.end(); ++iter)
							if(bone_values.find(iter->first) == bone_values.end())
								bone_values[iter->first]  = weight * iter->second.ori;
							else
								bone_values[iter->first] += weight * iter->second.ori;
					}
				}
				float inv_weight = 1.0f / total_weight;

				for(unordered_map<unsigned int, Vec3>::const_iterator iter = bone_values.begin(); iter != bone_values.end(); ++iter)
					SetBonePose(iter->first, iter->second * inv_weight, Vec3());

				any = true;
			}
		}
	}
}
