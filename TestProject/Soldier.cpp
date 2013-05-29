#include "StdAfx.h"
#include "Soldier.h"

#include "WeaponEquip.h"
#include "PlacedFootConstraint.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

namespace Test
{
	/*
	 * Soldier constants
	 */
	static float fly_accel_up = 15.0f;

	static float jump_to_fly_delay = 0.3f;
	static float jump_speed = 4.0f;

	static float fuel_spend_rate = 0.5f, fuel_refill_rate = 0.4f;
	static float flying_accel = 8.0f;

	static float top_speed_forward = 7.0f;							// running speed of a person can be around 5.8333[...] m/s
	static float top_speed_sideways = 5.0f;




	/*
	 * Soldier animations and related functions
	 */
	static void GenerateRestPose(KeyframeAnimation* ka)
	{
		ka->frames.clear();
		ka->name = "soldier rest pose";

		Keyframe kf;
		kf.duration = 2.0f;
		kf.next = 0;
		kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence();
		kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence();
		kf.values[Bone::string_table["l foot"]] =	BoneInfluence();
		kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence();
		kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence();
		kf.values[Bone::string_table["r foot"]] =	BoneInfluence();

		ka->frames.push_back(kf);
	}

	static void GenerateHardCodedWalkAnimation(KeyframeAnimation* ka)
	{
		const float A = 0.65f;
		const float B = 1.5f;

		ka->frames.clear();
		ka->name = "soldier walk forward";

		{	// right foot forward
			Keyframe kf(0.5f);
			kf.next = 1;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	A,	0,	0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	-A,	0,	0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	-A,	0,	0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	A,	0,	0), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 2;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	B,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	-B,	0,	0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());

			ka->frames.push_back(kf);
		}

		{	// left foot forward
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	-A,	0,	0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	A,	0,	0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	A,	0,	0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	-A,	0,	0), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	B,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	-B,	0,	0), Vec3());

			ka->frames.push_back(kf);
		}
	}

	static void GenerateReverseWalkanimation(KeyframeAnimation* ka)
	{
		KeyframeAnimation kb;
		GenerateHardCodedWalkAnimation(&kb);
		
		ka->frames.clear();
		ka->name = "soldier walk backward";

		int frame_order[] = { 3, 2, 1, 0 };

		for(int i = 0; i < 4; ++i)
		{
			Keyframe kf = kb.frames[frame_order[i]];
			kf.next = (i + 1) % 4;

			ka->frames.push_back(kf);
		}
	}

	static void GenerateRightWalkAnimation(KeyframeAnimation* ka)
	{
		const float A = 0.5f;			// sidestep
		const float B = 1.5f;			// knee bend
		const float C = -0.65f;			// leg bend to match knees
		const float D = B + C;
		const float E = -0.2f;			// left foot past center

		ka->frames.clear();
		ka->name = "soldier walk right";

		{	// right foot forward
			Keyframe kf(0.5f);
			kf.next = 1;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	0,	0,	A), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	0,	0,	-A), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	0,	0,	-A), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	0,	0,	A), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 2;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	C,	0,	A), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	B,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	-D,	0,	-A), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());

			ka->frames.push_back(kf);
		}

		{	// left foot forward
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	0,	0,	E), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	0,	0,	-E), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	0,	0,	-E), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	0,	0,	E), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	C,	0,	-A), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	B,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	-D,	0,	A), Vec3());

			ka->frames.push_back(kf);
		}
	}

	static void MakeMirrorAnimation(KeyframeAnimation* original, KeyframeAnimation* copy)
	{
		copy->frames.clear();

		for(int i = 0; i < 4; ++i)
		{
			const Keyframe& frame = original->frames[(i + 2) % 4];
			Keyframe nu_frame;
			nu_frame.duration = frame.duration;
			nu_frame.next = (i + 1) % 4;

			for(boost::unordered_map<unsigned int, BoneInfluence>::const_iterator jter = frame.values.begin(); jter != frame.values.end(); ++jter)
			{
				string original_name = Bone::string_table[jter->first];
				string changed_name = original_name;

				if(original_name.size() > 2 && original_name[1] == ' ')
					switch(original_name[0])
					{
						case 'l':
							changed_name[0] = 'r';
							break;

						case 'L':
							changed_name[0] = 'R';
							break;

						case 'r':
							changed_name[0] = 'l';
							break;

						case 'R':
							changed_name[0] = 'L';
							break;
					}

				BoneInfluence inf = jter->second;
				const Vec3& pos = inf.pos;
				const Vec3& ori = inf.ori;

				nu_frame.values[Bone::string_table[changed_name]] = BoneInfluence(Vec3(ori.x, -ori.y, -ori.z), Vec3(-pos.x, pos.y, pos.z));
			}

			copy->frames.push_back(nu_frame);
		}
	}

	static void GenerateTurnRightAnimation(KeyframeAnimation* ka)
	{
		const float A = 1.0f;				// foot rotation from center
		const float B = A / 2;
		const float C = 1.5f;				// knee bend
		const float D = 0.65f;				// leg forward/backward

		ka->frames.clear();
		ka->name = "soldier turn right";

		{	// right foot leading
			Keyframe kf(0.5f);
			kf.next = 1;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	0,	A,	0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	-D,	-A,	0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	D,	0,	0), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 2;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	0,	B,	0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	C,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	-C,	0,	0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	0,	-B,	0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());

			ka->frames.push_back(kf);
		}

		{	// left foot leading
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	-D,	0,	0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	D,	0,	0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg 1"]] =	BoneInfluence(Vec3(	0,	B,	0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());
			kf.values[Bone::string_table["l foot"]] =	BoneInfluence(Vec3(	0,	0,	0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] =	BoneInfluence(Vec3(	0,	-B,	0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] =	BoneInfluence(Vec3(	C,	0,	0), Vec3());
			kf.values[Bone::string_table["r foot"]] =	BoneInfluence(Vec3(	-C,	0,	0), Vec3());

			ka->frames.push_back(kf);
		}
	}

	static void GenerateLeftWalkAnimation(KeyframeAnimation* ka)
	{
		KeyframeAnimation kb;
		GenerateRightWalkAnimation(&kb);

		MakeMirrorAnimation(&kb, ka);
		ka->name = "soldier walk left";
	}

	static void GenerateTurnLeftAnimation(KeyframeAnimation* ka)
	{
		KeyframeAnimation kb;
		GenerateTurnRightAnimation(&kb);

		MakeMirrorAnimation(&kb, ka);
		ka->name = "soldier turn left";
	}




	/*
	 * Soldier methods
	 */
	Soldier::Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		gun_hand_bone(NULL),
		p_ag(NULL),
		walk_pose(NULL),
		walk_timer(0.0f),
		jet_fuel(1.0f),
		jet_start_sound(NULL),
		jet_loop_sound(NULL),
		jet_loop(NULL)
	{
		yaw = Random3D::Rand(float(M_PI) * 2.0f);

		gun_hand_bone = character->skeleton->GetNamedBone("r grip");

		Cache<SoundBuffer>* sound_cache = game_state->content->GetCache<SoundBuffer>();
		jet_start_sound = sound_cache->Load("jet_start");
		jet_loop_sound = sound_cache->Load("jet_loop");

		standing_callback.angular_coeff = 1.0f;
	}

	void Soldier::DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward)
	{
		float timestep = time.elapsed;

		bool can_recharge = true;
		bool jetted = false;
		if(control_state->GetBoolControl("jump"))
		{
			if(standing_callback.IsStanding() && time.total > jump_start_timer)							// jump off the ground
			{
				standing_callback.ApplyVelocityChange(Vec3(0, jump_speed, 0));
				standing_callback.BreakAllConstraints();

				jump_start_timer = time.total + jump_to_fly_delay;
			}
			else
			{
				can_recharge = false;

				if(jet_fuel > 0)
				{
					// jetpacking
					if(time.total > jump_start_timer)
					{
						jetted = true;

						if(jet_loop == NULL)
						{
							PlayDoodSound(jet_start_sound, 5.0f, false);
							jet_loop = PlayDoodSound(jet_loop_sound, 1.0f, true);
						}
						else
						{
							jet_loop->pos = pos;
							jet_loop->vel = vel;
						}

						jet_fuel -= timestep * (fuel_spend_rate);

						Vec3 fly_accel_vec = Vec3(0, fly_accel_up, 0);
						Vec3 horizontal_accel = forward * max(-1.0f, min(1.0f, control_state->GetFloatControl("forward"))) + rightward * max(-1.0f, min(1.0f, control_state->GetFloatControl("sidestep")));
						fly_accel_vec += horizontal_accel * (flying_accel);

						float total_mass = 0.0f;

						for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
							total_mass += (*iter)->GetMassInfo().mass;

						Vec3 apply_force = fly_accel_vec * total_mass;
						
						vector<RigidBody*> jet_bones;
						for(unsigned int i = 0; i < character->skeleton->bones.size(); ++i)
							if(character->skeleton->bones[i]->name == Bone::string_table["l shoulder"] || character->skeleton->bones[i]->name == Bone::string_table["r shoulder"])
								jet_bones.push_back(bone_to_rbody[i]);

						for(vector<RigidBody*>::iterator iter = jet_bones.begin(); iter != jet_bones.end(); ++iter)
							(*iter)->ApplyCentralForce(apply_force / float(jet_bones.size()));
					}
				}
				else
				{
					// out of fuel! flash hud gauge if it's relevant
					JumpFailureEvent evt = JumpFailureEvent(this);
					OnJumpFailure(&evt);
				}
			}
		}
		
		if(!jetted && jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}

		if(can_recharge)
			jet_fuel = min(jet_fuel + fuel_refill_rate * timestep, 1.0f);
	}

	void Soldier::PreUpdatePoses(TimingInfo time)
	{
		if(p_ag != NULL)
		{
			p_ag->yaw = yaw - walk_pose->yaw;
			p_ag->pitch = pitch;
		}
	}

	void Soldier::PostUpdatePoses(TimingInfo time)
	{
		// position and orient the gun
		if(equipped_weapon != NULL && gun_hand_bone != NULL)
		{
			equipped_weapon->gun_xform = Mat4::Translation(pos) * gun_hand_bone->GetTransformationMatrix() * Mat4::Translation(gun_hand_bone->rest_pos);
			equipped_weapon->sound_pos = equipped_weapon->pos = equipped_weapon->gun_xform.TransformVec3_1(0, 0, 0);
			equipped_weapon->sound_vel = equipped_weapon->vel = vel;
		}
	}

	void Soldier::RegisterFeet()
	{
		feet.push_back(new FootState(Bone::string_table["l foot"]));
		feet.push_back(new FootState(Bone::string_table["r foot"]));
	}

	void Soldier::Spawned()
	{
		Dood::Spawned();

		if(is_valid)
		{
			p_ag = new PoseAimingGun();
			posey->active_poses.push_back(p_ag);

			KeyframeAnimation rest, kf, kb, kr, kl, turnl, turnr;
			GenerateRestPose(&rest);
			GenerateHardCodedWalkAnimation(&kf);
			GenerateReverseWalkanimation(&kb);
			GenerateRightWalkAnimation(&kr);
			GenerateLeftWalkAnimation(&kl);
			GenerateTurnLeftAnimation(&turnl);
			GenerateTurnRightAnimation(&turnr);

			walk_pose = new WalkPose(this, &rest, &kf, &kb, &kl, &kr, &turnl, &turnr);
			walk_pose->yaw_bone = Bone::string_table["pelvis"];
			walk_pose->side_anim_rate = 2.5f;

			walk_timer = game_state->total_game_time;

			posey->active_poses.push_back(walk_pose);
		}
	}

	void Soldier::DeSpawned() { Dood::DeSpawned(); }

	void Soldier::DoCheatyPose(float timestep, const Vec3& net_vel)
	{
		walk_timer += timestep;
		PoseCharacter(TimingInfo(timestep, walk_timer));

		Dood::DoCheatyPose(timestep, net_vel);
	}

	void Soldier::MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi)
	{
		((TestGame*)game_state)->debug_text = ((stringstream&)(stringstream() << "cheaty rot = " << cheaty_rot.ComputeMagnitude())).str();
		cheaty_rot = Vec3();

		Dood::MaybeSinkCheatyVelocity(timestep, cheaty_vel, cheaty_rot, net_mass, net_moi);
	}
}
