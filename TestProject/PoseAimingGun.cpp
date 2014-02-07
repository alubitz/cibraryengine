#include "StdAfx.h"
#include "PoseAimingGun.h"

#define DEBUG_SOLVABILITY 0

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
			Vec2         normals[3];		// normals of bounding lines
			float        offsets[3];		// offsets of bounding lines
			float        inv_opp[3];		// multiplicative inverse (1/x) of the distance from bounding line to opposite vert
		};
		vector<PoseTri> tris;

		bool loaded_poses;

		Imp() : poses(), tris(), loaded_poses(false) { }
		~Imp() { }

		void LoadPoses(const string& filename)
		{
			ScriptingState s = ScriptSystem::GetGlobalState();
			if(!s.DoFile(filename))
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

			Vec3 dirs  [3] = { poses[a].dir,               poses[b].dir,               poses[c].dir               };
			Vec2 points[3] = { Vec2(dirs[0].x, dirs[0].y), Vec2(dirs[1].x, dirs[1].y), Vec2(dirs[2].x, dirs[2].y) };

#if DEBUG_SOLVABILITY
			stringstream ss;
			ss << "triangle with verts " << a << ", " << b << ", and " << c << ":" << endl;
			ss << '\t' << "xy = (" << points[0].x << ", " << points[0].y << ")" << endl;
			ss << '\t' << "xy = (" << points[1].x << ", " << points[1].y << ")" << endl;
			ss << '\t' << "xy = (" << points[2].x << ", " << points[2].y << ")" << endl;
			Debug(ss.str());
#endif

			tri.normals[0] = Vec2::Normalize(points[0].y - points[1].y, points[1].x - points[0].x);
			tri.normals[1] = Vec2::Normalize(points[1].y - points[2].y, points[2].x - points[1].x);
			tri.normals[2] = Vec2::Normalize(points[2].y - points[0].y, points[0].x - points[2].x);

			tri.offsets[0] = Vec2::Dot(tri.normals[0], points[0]);
			tri.offsets[1] = Vec2::Dot(tri.normals[1], points[1]);
			tri.offsets[2] = Vec2::Dot(tri.normals[2], points[2]);

			tri.inv_opp[0] = 1.0f / (Vec2::Dot(tri.normals[0], points[2]) - tri.offsets[0]);
			tri.inv_opp[1] = 1.0f / (Vec2::Dot(tri.normals[1], points[0]) - tri.offsets[1]);
			tri.inv_opp[2] = 1.0f / (Vec2::Dot(tri.normals[2], points[1]) - tri.offsets[2]);
		}

		void ExtractGrid()
		{
			if(poses.size() == 9)
			{
				ExtractTri(0, 3, 1);
				ExtractTri(1, 3, 4);
				ExtractTri(1, 4, 2);
				ExtractTri(2, 4, 5);
				ExtractTri(3, 6, 4);
				ExtractTri(4, 6, 7);
				ExtractTri(4, 7, 5);
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
		yaw(0),
		pitch(0)
	{
	}

	PoseAimingGun::~PoseAimingGun() { if(imp) { delete imp; imp = NULL; } }



	void PoseAimingGun::UpdatePose(const TimingInfo& time)
	{
		if(!imp->loaded_poses)
		{
			imp->LoadPoses("Files/Scripts/pose_aiming_gun.lua");
			imp->ExtractGrid();

			assert(imp->tris.size() <= 255);

			imp->loaded_poses = true;
		}

		if(time.total < 0.1f)
			return;

		Quaternion desired_ori = Quaternion::FromRVec(0, -yaw, 0) * Quaternion::FromRVec(pitch, 0, 0);
		Quaternion ori_off     = Quaternion::Reverse(pelvis_ori) * desired_ori;
		Vec3 ori_off_rvec      = ori_off.ToRVec();

		Vec3 spine_twist_rvec  = Vec3(max(-0.55f, min(0.5f, ori_off.x * 1.0f)), max(-0.5f, min(0.5f, (ori_off.y - 0.6f) * 0.5f)), 0);
		Quaternion spine_twist = Quaternion::FromRVec(spine_twist_rvec);
		Quaternion torso2_ori  = pelvis_ori * spine_twist * spine_twist;

		SetBonePose(Bone::string_table["torso 1"], spine_twist_rvec, Vec3());
		SetBonePose(Bone::string_table["torso 2"], spine_twist_rvec, Vec3());

		Vec3 aim_dir = desired_ori * Vec3(0, 0, 1);
		Vec3 relative_aim = Vec3::Normalize(Quaternion::Reverse(torso2_ori) * aim_dir);

		Vec2 aim_xy(relative_aim.x, relative_aim.y);

		unsigned char count = 0;
		float scores[3];
		for(unsigned int i = 0; i < imp->tris.size(); ++i)
		{
			const Imp::PoseTri& tri = imp->tris[i];

			unsigned int j;
			for(j = 0; j < 3; ++j)
			{
				scores[j] = Vec2::Dot(tri.normals[j], aim_xy) - tri.offsets[j];
				if(scores[j] < 0.0f)
					break;
			}
			if(j == 3)
			{
				++count;

				unordered_map<unsigned int, Vec3> bone_values;
				float total_weight = 0.0f;

				for(unsigned int k = 0; k < 3; ++k)
				{
					const Imp::AimPose& aimp = imp->poses[tri.indices[k]];

					unsigned int edge = (k + 1) % 3;
					float weight = scores[edge] * tri.inv_opp[edge];
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
			}
		}

#if DEBUG_SOLVABILITY
		switch(count)
		{
			case 0:
				Debug(((stringstream&)(stringstream() << "PoseAimingGun: no solution for aim xy = (" << aim_xy.x << ", " << aim_xy.y << ")" << endl)).str());
				break;

			case 1:
				Debug("PoseAimingGun: status OK!\n");
				break;

			default:
				Debug("PoseAimingGun: multiple solutions!\n");
				break;
		}
#endif
	}
}
