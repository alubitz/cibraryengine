#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"
#include "PlacedFootConstraint.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#define DIE_AFTER_ONE_SECOND   0

#define ENABLE_WALK_ANIMATIONS 0

namespace Test
{
	/*
	 * Soldier constants
	 */
	static const float jump_speed         = 4.0f;

	static const float fly_accel_up       = 15.0f;
	static const float fly_accel_lateral  = 8.0f;

	static const float fuel_spend_rate    = 0.5f;
	static const float fuel_refill_rate   = 0.4f;
	static const float jump_to_fly_delay  = 0.3f;




	/*
	 * Soldier animations and related functions
	 */
#if ENABLE_WALK_ANIMATIONS
	static void GenerateRestPose(KeyframeAnimation* ka)
	{
		ka->frames.clear();
		ka->name = "soldier rest pose";

		Keyframe kf;
		kf.duration = 2.0f;
		kf.next = 0;
		kf.values[Bone::string_table["l leg 1"]] = BoneInfluence();
		kf.values[Bone::string_table["l leg 2"]] = BoneInfluence();
		kf.values[Bone::string_table["l foot" ]] = BoneInfluence();
		kf.values[Bone::string_table["r leg 1"]] = BoneInfluence();
		kf.values[Bone::string_table["r leg 2"]] = BoneInfluence();
		kf.values[Bone::string_table["r foot" ]] = BoneInfluence();

		ka->frames.push_back(kf);
	}

	static void GenerateForwardWalkAnimation(KeyframeAnimation* ka)
	{
		const float A = 0.65f;
		const float B = 1.5f;

		ka->frames.clear();
		ka->name = "soldier walk forward";

		{	// right foot forward
			Keyframe kf(0.5f);
			kf.next = 1;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  A,  0,  0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3( -A,  0,  0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3( -A,  0,  0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3(  A,  0,  0), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 2;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  B,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3( -B,  0,  0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());

			ka->frames.push_back(kf);
		}

		{	// left foot forward
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3( -A,  0,  0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3(  A,  0,  0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  A,  0,  0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3( -A,  0,  0), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  B,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3( -B,  0,  0), Vec3());

			ka->frames.push_back(kf);
		}
	}

	static void GenerateReverseWalkAnimation(KeyframeAnimation* ka)
	{
		KeyframeAnimation kb;
		GenerateForwardWalkAnimation(&kb);

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

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  0,  0,  A), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3(  0,  0, -A), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  0,  0, -A), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3(  0,  0,  A), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 2;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  C,  0,  A), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  B,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3( -D,  0, -A), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());

			ka->frames.push_back(kf);
		}

		{	// left foot forward
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  0,  0,  E), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3(  0,  0, -E), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  0,  0, -E), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3(  0,  0,  E), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  C,  0, -A), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  B,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3( -D,  0,  A), Vec3());

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

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  0,  A,  0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3( -D, -A,  0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3(  D,  0,  0), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 2;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  0,  B,  0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  C,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3( -C,  0,  0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  0, -B,  0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());

			ka->frames.push_back(kf);
		}

		{	// left foot leading
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3( -D,  0,  0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3(  D,  0,  0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());

			ka->frames.push_back(kf);
		}

		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg 1"]] = BoneInfluence(Vec3(  0,  B,  0), Vec3());
			kf.values[Bone::string_table["l leg 2"]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());
			kf.values[Bone::string_table["l foot" ]] = BoneInfluence(Vec3(  0,  0,  0), Vec3());

			kf.values[Bone::string_table["r leg 1"]] = BoneInfluence(Vec3(  0, -B,  0), Vec3());
			kf.values[Bone::string_table["r leg 2"]] = BoneInfluence(Vec3(  C,  0,  0), Vec3());
			kf.values[Bone::string_table["r foot" ]] = BoneInfluence(Vec3( -C,  0,  0), Vec3());

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
#endif




	/*
	 * Soldier's custom FootState class
	 */
	class SoldierFoot : public Dood::FootState
	{
		public:

			Vec3 hip, knee, ankle;
			float hk_dist, ka_dist, hksq, kasq;

			float min_hasq, max_hasq;
			float hksq_m_kasq;
			// TODO: add members here as needed

			ModelPhysics::JointPhysics* joint_protos[3];
			Bone* bones[3];

			SoldierFoot(unsigned int posey_id, const Vec3& ee_pos);

			bool SolveLegIK(const Vec3& pelvis_pos, const Quaternion& pelvis_ori, const Mat4& pelvis_xform, const Vec3& foot_pos, const Quaternion& foot_ori, const Mat4& foot_xform);
			bool FindSuitableFootfall();
			void PoseUngroundedLeg();
	};




	/*
	 * Soldier methods
	 */
	Soldier::Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		gun_hand_bone(NULL),
		p_ag(NULL),
		walk_pose(NULL),
		jet_bones(),
		jet_fuel(1.0f),
		jet_start_sound(NULL),
		jet_loop_sound(NULL),
		jet_loop(NULL)
	{
		use_cheaty_ori = false;

		yaw = Random3D::Rand(float(M_PI) * 2.0f);

		gun_hand_bone = character->skeleton->GetNamedBone("r grip");

		Cache<SoundBuffer>* sound_cache = game_state->content->GetCache<SoundBuffer>();
		jet_start_sound = sound_cache->Load("jet_start");
		jet_loop_sound  = sound_cache->Load("jet_loop" );

		standing_callback.angular_coeff = 1.0f;

		ragdoll_timer = 3600.0f;

		p_ag = new PoseAimingGun();
		posey->active_poses.push_back(p_ag);
	}

	void Soldier::DoJumpControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward)
	{
		float timestep = time.elapsed;

		bool can_recharge = true;
		bool jetted = false;
		if(control_state->GetBoolControl("jump"))
		{
			if(standing_callback.IsStanding() && time.total > jump_start_timer)							// jump off the ground
			{
				standing_callback.ApplyVelocityChange(Vec3(0, jump_speed, 0));

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
						fly_accel_vec += horizontal_accel * fly_accel_lateral;

						float total_mass = 0.0f;

						for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
							total_mass += (*iter)->GetMassInfo().mass;

						Vec3 apply_force = fly_accel_vec * total_mass;
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

	void Soldier::PreUpdatePoses(const TimingInfo& time)
	{
		DoIKStuff(time);

		if(p_ag != NULL)
		{
			p_ag->yaw   = yaw;
			p_ag->pitch = pitch;

#if ENABLE_WALK_ANIMATIONS
			p_ag->pelvis_ori = Quaternion::FromRVec(0, -walk_pose->yaw, 0);
#endif
		}
	}

	void Soldier::PhysicsToCharacter()
	{
		Dood::PhysicsToCharacter();

		// position and orient the gun
		if(equipped_weapon != NULL)
		{
			Gun* gun = dynamic_cast<Gun*>(equipped_weapon);

			if(gun != NULL && gun->rigid_body != NULL)
			{
				RigidBody* gun_rb = gun->rigid_body;
				Mat4 gun_xform = gun_rb->GetTransformationMatrix();

				equipped_weapon->gun_xform = gun_xform;
				equipped_weapon->sound_pos = equipped_weapon->pos = gun_xform.TransformVec3_1(0, 0, 0);
				equipped_weapon->sound_vel = equipped_weapon->vel = vel;
			}
			else if(gun_hand_bone != NULL)
			{
				equipped_weapon->gun_xform = Mat4::Translation(pos) * gun_hand_bone->GetTransformationMatrix() * Mat4::Translation(gun_hand_bone->rest_pos);
				equipped_weapon->sound_pos = equipped_weapon->pos = equipped_weapon->gun_xform.TransformVec3_1(0, 0, 0);
				equipped_weapon->sound_vel = equipped_weapon->vel = vel;
			}
		}
	}

	void Soldier::RegisterFeet()
	{
		feet.push_back(new SoldierFoot(Bone::string_table["l foot"], Vec3( 0.238f, 0.000f, 0.065f)));
		feet.push_back(new SoldierFoot(Bone::string_table["r foot"], Vec3(-0.238f, 0.000f, 0.065f)));
	}

	void Soldier::Update(const TimingInfo& time)
	{
#if DIE_AFTER_ONE_SECOND
		if(time.total > 1.0f)
			Die(Damage());
#endif

		Dood::Update(time);
	}

	void Soldier::Die(const Damage& cause)
	{
		if(jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}

		Dood::Die(cause);
	}

	void Soldier::Spawned()
	{
		Dood::Spawned();

		if(!is_valid)
			return;		

#if ENABLE_WALK_ANIMATIONS
		KeyframeAnimation rest, kf, kb, kr, kl, turnl, turnr;

		GenerateRestPose            (&rest );
		GenerateForwardWalkAnimation(&kf   );
		GenerateReverseWalkAnimation(&kb   );
		GenerateRightWalkAnimation  (&kr   );
		GenerateLeftWalkAnimation   (&kl   );
		GenerateTurnLeftAnimation   (&turnl);
		GenerateTurnRightAnimation  (&turnr);

		walk_pose = new WalkPose(this, &rest, &kf, &kb, &kl, &kr, &turnl, &turnr);
		walk_pose->yaw_bone = Bone::string_table["pelvis"];
		walk_pose->side_anim_rate = 2.5f;
		//walk_pose->InitialSetYawOffset(0.6f);			// was previously 0.322f, based on some old measurement

		posey->active_poses.push_back(walk_pose);
#else
		p_ag->pelvis_ori = Quaternion::FromRVec(0, -yaw, 0);
#endif

		for(unsigned int i = 0; i < character->skeleton->bones.size(); ++i)
			if(character->skeleton->bones[i]->name == Bone::string_table["l shoulder"] || character->skeleton->bones[i]->name == Bone::string_table["r shoulder"])
				jet_bones.push_back(bone_to_rbody[i]);

		for(vector<FootState*>::iterator iter = feet.begin(); iter != feet.end(); ++iter)
		{
			SoldierFoot* foot = (SoldierFoot*)*iter;

			string prefix = iter == feet.begin() ? "l " : "r ";
			string bnames[4] = { "pelvis", prefix + "leg 1", prefix + "leg 2", prefix + "foot" };
			unsigned int bindices[4] = { 0, 0, 0, 0 };

			for(unsigned int i = 0; i < mphys->bones.size(); ++i)
				for(unsigned int j = 0; j < 4; ++j)
					if(mphys->bones[i].bone_name == bnames[j])
						bindices[j] = i + 1;

			for(vector<ModelPhysics::JointPhysics>::iterator jter = mphys->joints.begin(); jter != mphys->joints.end(); ++jter)
			{
				ModelPhysics::JointPhysics& jp = *jter;
				if(     jp.bone_a == bindices[0] && jp.bone_b == bindices[1] || jp.bone_a == bindices[1] && jp.bone_b == bindices[0])
					foot->joint_protos[0] = &jp;
				else if(jp.bone_a == bindices[1] && jp.bone_b == bindices[2] || jp.bone_a == bindices[2] && jp.bone_b == bindices[1])
					foot->joint_protos[1] = &jp;
				else if(jp.bone_a == bindices[2] && jp.bone_b == bindices[3] || jp.bone_a == bindices[3] && jp.bone_b == bindices[2])
					foot->joint_protos[2] = &jp;
			}

			for(unsigned int i = 0; i < 3; ++i)
				foot->bones[i] = posey->skeleton->GetNamedBone(bnames[i + 1]);
		}
	}

	void Soldier::MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi)
	{
#if 0
		if(game_state->total_game_time < 0.5f)
		{
			cheaty_vel = Vec3();
			cheaty_rot = Vec3();
		}
#endif

		Dood::MaybeSinkCheatyVelocity(timestep, cheaty_vel, cheaty_rot, net_mass, net_moi);
	}


	void Soldier::DoInitialPose()
	{
		Dood::DoInitialPose();

		p_ag->yaw = yaw;
		p_ag->pitch = pitch;
		p_ag->pelvis_ori = posey->skeleton->GetNamedBone("pelvis")->ori;
		p_ag->UpdatePose(TimingInfo(0, 0));

		for(boost::unordered_map<unsigned int, BoneInfluence>::iterator iter = p_ag->bones.begin(); iter != p_ag->bones.end(); ++iter)
			posey->skeleton->GetNamedBone(iter->first)->ori = Quaternion::FromRVec(iter->second.ori);
	}


	void Soldier::DoIKStuff(const TimingInfo& time)
	{
		Quaternion desired_ori = Quaternion::FromRVec(0, -yaw, 0) * Quaternion::FromRVec(0.1f, 0, 0);

		Vec3 initial_pos = root_rigid_body->GetPosition();
		Quaternion initial_ori = root_rigid_body->GetOrientation();


		// decide what velocity we want our CoM to have
		Vec3 desired_vel = desired_vel_2d;							// computed in DoMovementControls

		// measure the CoM and average velocity of the Dood
		Vec3 com;
		Vec3 com_vel;
		float total_mass = 0.0f;
		for(set<RigidBody*>::iterator iter = velocity_change_bodies.begin(); iter != velocity_change_bodies.end(); ++iter)
		{
			RigidBody* rb = *iter;

			float mass = rb->GetMass();
			total_mass += mass;

			com_vel    += rb->GetLinearVelocity() * mass;
			com        += rb->GetCenterOfMass()   * mass;
		}
		com     /= total_mass;
		com_vel /= total_mass;

		RigidBody* lfoot = RigidBodyForNamedBone("l foot");
		RigidBody* rfoot = RigidBodyForNamedBone("r foot");
		RigidBody* torso = RigidBodyForNamedBone("torso 2");

		//Vec3 tipping = ((lfoot->GetLinearVelocity() + rfoot->GetLinearVelocity()) * 0.5f - com_vel) - (torso->GetLinearVelocity() - com_vel);

		//Vec3 pelvis_com = root_rigid_body->GetCenterOfMass();
		//Vec3 lfoot_com = lfoot->GetCenterOfMass();
		//Vec3 rfoot_com = rfoot->GetCenterOfMass();
		//Vec3 pelvis_offcenter = (pelvis_com * 0.25f + com * 0.75f) - (lfoot_com * 0.5f + rfoot_com * 0.5f + desired_ori * Vec3(0, 0, -0.05f));
		//pelvis_offcenter.y += 0.7f;
		//desired_vel -= pelvis_offcenter;

		Vec3 pelvis_vel = Vec3();//root_rigid_body->GetLinearVelocity() + (desired_vel - com_vel) * 0.5f;

		Vec3 local_com = root_rigid_body->GetMassInfo().com;
		Quaternion pelvis_ori = initial_ori * 0.2f + desired_ori * 0.8f;
		Vec3 pelvis_pos = initial_pos + pelvis_vel * time.elapsed - initial_ori * local_com + pelvis_ori * local_com;		
		Mat4 pelvis_xform = Mat4::FromPositionAndOrientation(pelvis_pos, pelvis_ori);

		SoldierFoot* left_foot  = (SoldierFoot*)feet[0];
		SoldierFoot* right_foot = (SoldierFoot*)feet[1];

		float T = min(1.0f, time.total * 2.0f);
		float U = T * 2.0f;
		float V = T * 0.1f;
		float W = T * 0.3f;

		/*
		left_foot->bones[0]->ori  = Quaternion::FromRVec( -T,  0,  0 );
		left_foot->bones[1]->ori  = Quaternion::FromRVec(  U,  0,  0 );
		left_foot->bones[2]->ori  = Quaternion::FromRVec( -T,  0,  0 );

		right_foot->bones[0]->ori = Quaternion::FromRVec(  0,  0, -V );
		right_foot->bones[1]->ori = Quaternion::FromRVec( -V,  0,  0 );
		right_foot->bones[2]->ori = Quaternion::FromRVec(  V,  0, -W );
		*/

		/*
		for(vector<FootState*>::iterator iter = feet.begin(); iter != feet.end(); ++iter)
		{
			SoldierFoot* foot = (SoldierFoot*)*iter;

			Vec3       foot_pos = foot->body->GetPosition();
			Quaternion foot_ori = foot->body->GetOrientation();

			foot_ori = foot_ori * 0.2f + desired_ori * 0.8f;

			Mat4 foot_xform = Mat4::FromPositionAndOrientation(foot_pos, foot_ori);

			bool ik_result = foot->SolveLegIK(pelvis_pos, pelvis_ori, pelvis_xform, foot_pos, foot_ori, foot_xform);
			//foot->PoseUngroundedLeg();
		}
		*/
	}




	/*
	 * SoldierFoot methods
	 */
	SoldierFoot::SoldierFoot(unsigned int posey_id, const Vec3& ee_pos) :
		FootState(posey_id, ee_pos)
	{
		for(unsigned int i = 0; i < 3; ++i)
		{
			joint_protos[i] = NULL;
			bones[i] = NULL;
		}

		// TODO: get these joints' positions from the actual joints instead of using hard-coded values
		float x = posey_id == Bone::string_table["l foot"] ? 1.0f : -1.0f;
		hip   = Vec3(x * 0.15f, 1.04f, -0.02f);
		knee  = Vec3(x * 0.19f, 0.63f,  0.05f);
		ankle = Vec3(x * 0.23f, 0.16f, -0.06f);

		hksq = (hip  - knee ).ComputeMagnitudeSquared();
		kasq = (knee - ankle).ComputeMagnitudeSquared();

		hk_dist = sqrtf(hksq);
		ka_dist = sqrtf(kasq);

		hksq_m_kasq = hksq - kasq;

		// TODO: do a more thorough computation here (or maybe load the values from somewhere?)
		float max_ha = hk_dist + ka_dist;
		float min_ha = fabs(hk_dist - ka_dist);
		max_hasq = max_ha * max_ha;
		min_hasq = min_ha * min_ha;
	}

	bool SoldierFoot::SolveLegIK(const Vec3& pelvis_pos, const Quaternion& pelvis_ori, const Mat4& pelvis_xform, const Vec3& foot_pos, const Quaternion& foot_ori, const Mat4& foot_xform)
	{
		Debug(((stringstream&)(stringstream() << Bone::string_table[posey_id] << ":" << endl)).str());

		// figure out where the constrained bone xforms put the hip and ankle joints
		Vec3 goal_hip   = pelvis_xform.TransformVec3_1(hip);
		Vec3 goal_ankle = foot_xform.TransformVec3_1(ankle);

		Vec3 D = goal_ankle - goal_hip;

		// quit early if the joints are too close together or too far apart
		float dsq = D.ComputeMagnitudeSquared();
		if(dsq > max_hasq || dsq < min_hasq)
		{
			if(dsq > max_hasq)
				Debug("\tDistance too big\n");
			else
				Debug("\tDistance too small\n");

			return false;
		}

		// figure out the circle where the knee can go
		float dist = D.ComputeMagnitude();
		float invd = 1.0f / dist;

		float hk_u = (hksq_m_kasq + dsq) * 0.5f * invd;

		assert(hk_u <= hk_dist);

		float circ_rsq = hksq - hk_u * hk_u;
		Vec3 circ_normal = D * invd;
		Vec3 circ_center = goal_hip + hk_u * circ_normal;

		assert(circ_rsq >= 0.0f);

		// get two orthogonal axes in the plane of the circle... try to get one to be approximately 'forward'
		Vec3 knee_fwd = pelvis_xform.TransformVec3_0(0, 0, 1) + foot_xform.TransformVec3_0(0, 0, 1);
		Vec3 knee_left = Vec3::Normalize(Vec3::Cross(circ_normal, knee_fwd));
		knee_fwd = Vec3::Cross(knee_left, circ_normal);

		// use the forward vector to place the knee
		Vec3 knee_pos = circ_center + Vec3::Normalize(Vec3::Cross(circ_normal, Vec3::Cross(circ_normal, knee_fwd)), circ_rsq);

		// rotate the leg bones so their "shaft" (direction between joint positions) matches what we've selected
		Vec3 result_shaft_dirs[2] = {
			Vec3::Normalize(knee_pos   - goal_hip),
			Vec3::Normalize(goal_ankle - knee_pos)
		};

		Vec3 true_rest_dirs[2] = {
			Vec3::Normalize(knee  - hip ),
			Vec3::Normalize(ankle - knee)
		};

		Vec3 rest_shaft_dirs[2] = {
			pelvis_xform.TransformVec3_0(true_rest_dirs[0]),
			pelvis_xform.TransformVec3_0(true_rest_dirs[1])
		};

		Quaternion rtrs[2];				// orientations to rotate rest shaft onto result shafts

		for(unsigned int i = 0; i < 2; ++i)
		{
			Vec3 cross = Vec3::Cross(rest_shaft_dirs[i], result_shaft_dirs[i]);
			float cross_mag = cross.ComputeMagnitude(), inv_cross = 1.0f / cross_mag;
			float dot = Vec3::Dot(result_shaft_dirs[i], rest_shaft_dirs[i]);

			float angle = atan2f(cross_mag, dot);
			rtrs[i] = Quaternion::FromAxisAngle(cross.x * inv_cross, cross.y * inv_cross, cross.z * inv_cross, angle);
		}

		Quaternion joris[3], best_joris[3];
		float best_score = 0.0f;
		float best_a = 0.0f, best_b = 0.0f;

		Quaternion uleg_base = pelvis_ori * rtrs[0];
		Quaternion lleg_base = pelvis_ori * rtrs[1];

		for(unsigned int j = 0; j < 10000; ++j)
		{
			float mut = j < 100 ? 3.2f : best_score * 0.5f + 0.001f;
			float a = best_a + Random3D::Rand(-mut, mut);
			float b = best_b + Random3D::Rand(-mut, mut);

			// get orientations of the bones, and relative orientations across joints
			Quaternion boris[4] = {
				pelvis_ori,
				uleg_base * Quaternion::FromAxisAngle(true_rest_dirs[0], a),
				lleg_base * Quaternion::FromAxisAngle(true_rest_dirs[1], b),
				foot_ori
			};

			// see how well this configuration sticks to the joints' rotation limits
			float score = 0.0f;
			for(unsigned int i = 0; i < 3; ++i)
			{
				joris[i] = Quaternion::Reverse(boris[i]) * boris[i + 1];

				Vec3 rvec       = joris[i].ToRVec();
				Vec3 into_axes  = joint_protos[i]->axes * rvec;
				Vec3 clamped    = joint_protos[i]->GetClampedAngles(into_axes);
				//Vec3 into_world = joint_protos[i]->axes.TransposedMultiply(clamped);
				//bones[i]->ori = Quaternion::FromRVec(into_world);

				float error = (into_axes - clamped).ComputeMagnitude();
				float work = fabs((Quaternion::Reverse(bones[i]->ori) * joris[i]).GetRotationAngle());
				score += error * error + error * work + work * work;
			}

			// finally, set joint orientations
			if(j == 0 || score < best_score)
			{
				best_score = score;
				for(unsigned int i = 0; i < 3; ++i)
					best_joris[i] = joris[i];

				if(score == 0.0f)
					break;

				best_a = a;
				best_b = b;
			}
		}

		Debug(((stringstream&)(stringstream() << '\t' << "best score = " << best_score << endl)).str());

		for(unsigned int i = 0; i < 3; ++i)
			bones[i]->ori = best_joris[i];

		return true;
	}

	bool SoldierFoot::FindSuitableFootfall()
	{
		// TODO:
		// query PhysicsWorld for potentially relevant terrain geometry nearby
		// search for reasonable places to put the foot

		return false;
	}

	void SoldierFoot::PoseUngroundedLeg()
	{
		// TODO: implement this
	}
}
