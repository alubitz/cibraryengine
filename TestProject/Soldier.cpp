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

			Mat4* pelvis_xform_ptr;

			Vec3 hip, knee, ankle;
			float hk_dist, ka_dist, hksq, kasq;

			float min_hasq, max_hasq;
			float hksq_m_kasq;
			// TODO: add members here as needed

			Quaternion hip_out, knee_out, ankle_out;
			bool outputs_valid;

			SoldierFoot(unsigned int posey_id, const Vec3& ee_pos, Mat4* pelvis_xform_ptr);

			bool SolveLegIK();
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
		use_cheaty_ori = true;

		yaw = Random3D::Rand(float(M_PI) * 2.0f);

		gun_hand_bone = character->skeleton->GetNamedBone("r grip");

		Cache<SoundBuffer>* sound_cache = game_state->content->GetCache<SoundBuffer>();
		jet_start_sound = sound_cache->Load("jet_start");
		jet_loop_sound  = sound_cache->Load("jet_loop" );

		standing_callback.angular_coeff = 1.0f;

		ragdoll_timer = 3600.0f;
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
		DoIKStuff();

		for(vector<FootState*>::iterator iter = feet.begin(); iter != feet.end(); ++iter)
		{
			SoldierFoot* foot = (SoldierFoot*)*iter;
			if(foot->outputs_valid)
			{
				if(iter == feet.begin())
				{
					posey->skeleton->GetNamedBone( "l foot"  )->ori = foot->ankle_out;
					posey->skeleton->GetNamedBone( "l leg 2" )->ori = foot->knee_out;
					posey->skeleton->GetNamedBone( "l leg 1" )->ori = foot->hip_out;
				}
				else
				{
					posey->skeleton->GetNamedBone( "r foot"  )->ori = foot->ankle_out;
					posey->skeleton->GetNamedBone( "r leg 2" )->ori = foot->knee_out;
					posey->skeleton->GetNamedBone( "r leg 1" )->ori = foot->hip_out;
				}
			}
		}

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
		feet.push_back(new SoldierFoot(Bone::string_table["l foot"], Vec3( 0.238f, 0.000f, 0.065f), &proposed_pelvis_xform));
		feet.push_back(new SoldierFoot(Bone::string_table["r foot"], Vec3(-0.238f, 0.000f, 0.065f), &proposed_pelvis_xform));
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

		if(is_valid)
		{
			posey->skeleton->GetNamedBone("pelvis")->ori = Quaternion::FromRVec(0, -yaw, 0);

			p_ag = new PoseAimingGun();
			posey->active_poses.push_back(p_ag);

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
		}
	}

	void Soldier::MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi)
	{
		//((TestGame*)game_state)->debug_text = ((stringstream&)(stringstream() << "cheaty rot = " << cheaty_rot.ComputeMagnitude())).str();
		cheaty_rot = Vec3();

		Dood::MaybeSinkCheatyVelocity(timestep, cheaty_vel, cheaty_rot, net_mass, net_moi);
	}



	Vec3 Soldier::ComputeDesiredVelocity()
	{
		Vec3 desired_vel = desired_vel_2d;					// computed by DoMovementControls
		// TODO: compute y component based on terrain slope? and jump control

		return desired_vel;
	}

	void Soldier::SetRootBoneXform(const Vec3& desired_vel)
	{
		Vec3 pos = root_rigid_body->GetPosition();
		Quaternion ori = root_rigid_body->GetOrientation();

		Vec3 com_vel;
		float total_mass = 0.0f;
		for(set<RigidBody*>::iterator iter = velocity_change_bodies.begin(); iter != velocity_change_bodies.end(); ++iter)
		{
			RigidBody* rb = *iter;
			float mass = rb->GetMass();
			total_mass += mass;
			com_vel += rb->GetLinearVelocity() * mass;
		}
		com_vel /= total_mass;

		Vec3 pelvis_vel = desired_vel * 0.4f + root_rigid_body->GetLinearVelocity() * 0.5f + com_vel * 0.1f;

		// TODO:
		// pelvis moves by a velocity somewhere between the desired velocity and the current average velocity of the dood
		// pelvis yaws a little to help the aim yaw
		// pelvis moves forward/backward to balance the aim pitch (and maybe pitch the pelvis up/down a little as well?)

		proposed_pelvis_xform = Mat4::FromPositionAndOrientation(pos + pelvis_vel / 60.0f, ori);
	}




	/*
	 * SoldierFoot methods
	 */
	SoldierFoot::SoldierFoot(unsigned int posey_id, const Vec3& ee_pos, Mat4* pelvis_xform_ptr) : FootState(posey_id, ee_pos), pelvis_xform_ptr(pelvis_xform_ptr)
	{
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

	bool SoldierFoot::SolveLegIK()
	{
		outputs_valid = false;

		// figure out where the constrained bone xforms put the hip and ankle joints
		const Mat4& pelvis_xform = *pelvis_xform_ptr;

		Mat4 foot_xform = body->GetTransformationMatrix();

		Vec3 goal_hip   = pelvis_xform.TransformVec3_1(hip);
		Vec3 goal_ankle = foot_xform.TransformVec3_1(ankle);

		Vec3 D = goal_ankle - goal_hip;

		// quit early if the joints are too close together or too far apart
		float dsq = D.ComputeMagnitudeSquared();
		if(dsq > max_hasq || dsq < min_hasq)
			return false;

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

		// pick orientations for the leg bones
		Vec3 result_uhk = Vec3::Normalize(knee_pos   - goal_hip);
		Vec3 result_uka = Vec3::Normalize(goal_ankle - knee_pos);

		Vec3 rest_uhk = Vec3::Normalize(pelvis_xform.TransformVec3_0(knee  - hip ));
		Vec3 rest_uka = Vec3::Normalize(pelvis_xform.TransformVec3_0(ankle - knee));

		Vec3 uhk_cross = Vec3::Cross(rest_uhk, result_uhk);
		float uhkx_mag = uhk_cross.ComputeMagnitude(), inv_uhkx = 1.0f / uhkx_mag;
		float hkdot = Vec3::Dot(result_uhk, rest_uhk);

		float hk_rtr_angle = atan2f(uhkx_mag, hkdot);
		Quaternion hk_rtr = Quaternion::FromAxisAngle(uhk_cross.x * inv_uhkx, uhk_cross.y * inv_uhkx, uhk_cross.z * inv_uhkx, hk_rtr_angle);
		//Vec3 hktest = Vec3::Normalize((hk_rtr * Quaternion::Reverse(pelvis_ori)) * (knee - hip));
		//float error = acosf(Vec3::Dot(hktest, result_uhk));
		//Debug(((stringstream&)(stringstream() << "error = " << error << "; angle = " << hk_rtr.GetRotationAngle() << "; expected angle = " << hk_rtr_angle << endl)).str());

		Vec3 uka_cross = Vec3::Cross(rest_uka, result_uka);
		float ukax_mag = uka_cross.ComputeMagnitude(), inv_ukax = 1.0f / ukax_mag;
		float kadot = Vec3::Dot(result_uka, rest_uka);

		float ka_rtr_angle = atan2f(ukax_mag, kadot);
		Quaternion ka_rtr = Quaternion::FromAxisAngle(uka_cross.x * inv_ukax, uka_cross.y * inv_ukax, uka_cross.z * inv_ukax, ka_rtr_angle);

		Quaternion pelvis_ori = pelvis_xform.ExtractOrientation();
		Quaternion uleg_ori = hk_rtr * Quaternion::Reverse(pelvis_ori);		// huh? not sure why this works this way
		Quaternion lleg_ori = ka_rtr * Quaternion::Reverse(pelvis_ori);
		Quaternion foot_ori = foot_xform.ExtractOrientation();

		Quaternion hip_ori   = Quaternion::Reverse(pelvis_ori) * uleg_ori;
		Quaternion knee_ori  = Quaternion::Reverse(uleg_ori)   * lleg_ori;
		Quaternion ankle_ori = Quaternion::Reverse(lleg_ori)   * foot_ori;

		// TODO: check joint rotation limits

		hip_out   = hip_ori;
		knee_out  = knee_ori;
		ankle_out = ankle_ori;

		outputs_valid = true;
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
