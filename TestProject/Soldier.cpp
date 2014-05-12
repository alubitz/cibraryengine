#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"
#include "PlacedFootConstraint.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#define DIE_AFTER_ONE_SECOND   0

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
	 * Soldier's custom FootState class
	 */
	class SoldierFoot : public Dood::FootState
	{
		public:

			SoldierFoot(unsigned int posey_id, const Vec3& ee_pos);

			Quaternion OrientBottomToSurface(const Vec3& normal) const;
	};




	/*
	 * Soldier private implementation struct
	 */
	struct Soldier::Imp
	{
		bool init;

		RigidBody *pelvis,    *torso1, *torso2, *head;
		RigidBody *lshoulder, *luarm,  *llarm,  *lhand;
		RigidBody *rshoulder, *ruarm,  *rlarm,  *rhand;

		RigidBody* anchored_body;

		SkeletalJointConstraint *neck;
		SkeletalJointConstraint *lsja, *lsjb, *lelbow, *lwrist;
		SkeletalJointConstraint *rsja, *rsjb, *relbow, *rwrist;

		float inv_timestep;

		Imp() :
			init(false),
			pelvis(NULL), torso1(NULL), torso2(NULL), head(NULL),
			lshoulder(NULL), luarm(NULL), llarm(NULL), lhand(NULL),
			rshoulder(NULL), ruarm(NULL), rlarm(NULL), rhand(NULL),
			anchored_body(NULL),
			neck(NULL),
			lsja(NULL), lsjb(NULL), lelbow(NULL), lwrist(NULL),
			rsja(NULL), rsjb(NULL), relbow(NULL), rwrist(NULL),
			inv_timestep(0)
		{
		}

		~Imp() { }

		void Init(Soldier* dood)
		{
			//dood->collision_group->SetInternalCollisionsEnabled(true);		// TODO: resolve problems arising from torso2-arm1 collisions

			pelvis    = dood->RigidBodyForNamedBone( "pelvis"     );
			torso1    = dood->RigidBodyForNamedBone( "torso1"     );
			torso2    = dood->RigidBodyForNamedBone( "torso 2"    );
			head      = dood->RigidBodyForNamedBone( "head"       );

			lshoulder = dood->RigidBodyForNamedBone( "l shoulder" );
			luarm     = dood->RigidBodyForNamedBone( "l arm 1"    );
			llarm     = dood->RigidBodyForNamedBone( "l arm 2"    );
			lhand     = dood->RigidBodyForNamedBone( "l hand"     );

			rshoulder = dood->RigidBodyForNamedBone( "r shoulder" );
			ruarm     = dood->RigidBodyForNamedBone( "r arm 1"    );
			rlarm     = dood->RigidBodyForNamedBone( "r arm 2"    );
			rhand     = dood->RigidBodyForNamedBone( "r hand"     );

			anchored_body = torso2;

			for(unsigned int i = 0; i < dood->constraints.size(); ++i)
			{
				SkeletalJointConstraint* sjc = (SkeletalJointConstraint*)dood->constraints[i];
				RigidBody *a = sjc->obj_a, *b = sjc->obj_b;

				if     ( b == head      && a == torso2    ) { neck   = sjc; }
				else if( b == lshoulder && a == torso2    ) { lsja   = sjc; }
				else if( b == luarm     && a == lshoulder ) { lsjb   = sjc; }
				else if( b == llarm     && a == luarm     ) { lelbow = sjc; }
				else if( b == lhand     && a == llarm     ) { lwrist = sjc; }
				else if( b == rshoulder && a == torso2    ) { rsja   = sjc; }
				else if( b == ruarm     && a == rshoulder ) { rsjb   = sjc; }
				else if( b == rlarm     && a == ruarm     ) { relbow = sjc; }
				else if( b == rhand     && a == rlarm     ) { rwrist = sjc; }
			}
		}

		bool SetWorldTorque(SkeletalJointConstraint* sjc, const Vec3& torque, Vec3& actual)
		{
			Mat3 oriented_axes = sjc->axes * Quaternion::Reverse(sjc->obj_a->GetOrientation()).ToMat3();
			Vec3 local_torque = oriented_axes * torque;

			const Vec3 &mint = sjc->min_torque, &maxt = sjc->max_torque;

			sjc->apply_torque.x = max(mint.x, min(maxt.x, local_torque.x));
			sjc->apply_torque.y = max(mint.y, min(maxt.y, local_torque.y));
			sjc->apply_torque.z = max(mint.z, min(maxt.z, local_torque.z));

			Vec3 dif = sjc->apply_torque - local_torque;
			if(dif.x != 0 || dif.y != 0 || dif.z != 0)
			{
				actual = oriented_axes.TransposedMultiply(sjc->apply_torque);
				return true;
			}
			else
			{
				actual = torque;			// discrepancy between this and the reconverted value should be minimal
				return false;
			}
		}

		void DoAnchorStuff(Soldier* dood, const TimingInfo& time)
		{
			static const float helper_frac = 0.05f;

			float yaw   = dood->yaw + 0.5f;
			float pitch = dood->pitch * 0.2f + powf(dood->pitch / float(M_PI / 2), 3.0f) * 1.15f;

			Quaternion desired_ori = Quaternion::FromRVec(0, -yaw, 0) * Quaternion::FromRVec(pitch, 0, 0);
			Quaternion current_ori = anchored_body->GetOrientation();
			Quaternion ctd         = current_ori * Quaternion::Reverse(desired_ori);
			Quaternion use_ori     = Quaternion::FromRVec(ctd.ToRVec() * helper_frac) * desired_ori;

			Vec3 local_com = anchored_body->GetMassInfo().com;

			anchored_body->SetPosition(Vec3(0, 1, 0) + local_com - use_ori * local_com);
			anchored_body->SetOrientation(use_ori);
			anchored_body->SetLinearVelocity(Vec3());
			anchored_body->SetAngularVelocity(Vec3());
		}

		void DoNeckStuff(Soldier* dood, const TimingInfo& time)
		{
			Quaternion desired_ori     = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);
			Quaternion head_ori        = head->GetOrientation();
			Quaternion head_to_desired = head_ori * Quaternion::Reverse(desired_ori);

			Vec3 htd_rot  = head_to_desired.ToRVec() * inv_timestep;
			Vec3 head_rot = head->GetAngularVelocity();
			Vec3 aaccel   = (htd_rot - head_rot) * -inv_timestep;							// negative to make the positive torque to go the head, which is obj_b
			Vec3 htorque  = Mat3(head->GetTransformedMassInfo().moi) * aaccel;

			SetWorldTorque(neck, htorque, htorque);
		}

		void DoArmsAimingGun(Soldier* dood, const TimingInfo& time)
		{
			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				// get desired orientations of arm bones
				dood->PreparePAG(time);

				unsigned int bone_names[8] =
				{
					Bone::string_table["l shoulder"], Bone::string_table["l arm 1"], Bone::string_table["l arm 2"], Bone::string_table["l hand"],
					Bone::string_table["r shoulder"], Bone::string_table["r arm 1"], Bone::string_table["r arm 2"], Bone::string_table["r hand"]
				};

				Quaternion desired_oris[8];
				for(unsigned int i = 0; i < 8; ++i)
					desired_oris[i] = dood->posey->skeleton->GetNamedBone(bone_names[i])->GetTransformationMatrix().ExtractOrientation();

				// compute desired per-bone net torques
				RigidBody* rbs[8] = { lshoulder, luarm, llarm, lhand, rshoulder, ruarm, rlarm, rhand };
				RigidBody* gun_rb = gun->rigid_body;

				MassInfo rb_mass_infos[] =
				{
					lshoulder->GetTransformedMassInfo(),
					luarm->GetTransformedMassInfo(),
					llarm->GetTransformedMassInfo(),
					lhand->GetTransformedMassInfo(),
					rshoulder->GetTransformedMassInfo(),
					ruarm->GetTransformedMassInfo(),
					rlarm->GetTransformedMassInfo(),
					rhand->GetTransformedMassInfo(),
					gun_rb->GetTransformedMassInfo()
				};

				// TODO: improve handling of "hands and gun" (HNG) pseudo-RigidBody
				MassInfo hng_mass_infos[] = { rb_mass_infos[3], rb_mass_infos[7], rb_mass_infos[8] };
				Mat3 hng_moi = Mat3(MassInfo::Sum(hng_mass_infos, 3).moi);

				Mat3 use_mois[8] =
				{
					Mat3(rb_mass_infos[0].moi),
					Mat3(rb_mass_infos[1].moi),
					Mat3(rb_mass_infos[2].moi),
					hng_moi,
					Mat3(rb_mass_infos[4].moi),
					Mat3(rb_mass_infos[5].moi),
					Mat3(rb_mass_infos[6].moi),
					hng_moi
				};

				Vec3 bone_torques[8];
				for(unsigned int i = 0; i < 8; ++i)
				{
					RigidBody* rb = rbs[i];

					Quaternion& desired_ori = desired_oris[i];
					Quaternion ori          = rb->GetOrientation();

					Vec3 desired_rot    = (desired_ori * Quaternion::Reverse(ori)).ToRVec() * -inv_timestep;										
					Vec3 desired_aaccel = (desired_rot - rb->GetAngularVelocity()) * inv_timestep;

					bone_torques[i] = use_mois[i] * desired_aaccel;
				}


				// TODO: fudge factor... estimate applied torques which will produce the desired net torques


				// compute applied joint torques to achieve the per-bone applied torques we just came up with				
				Vec3 actual, lw_actual;
				SetWorldTorque( lwrist, -bone_torques[3] * 0.75f,     lw_actual );
				SetWorldTorque( lelbow,  actual - bone_torques[2],    actual    );
				SetWorldTorque( lsjb,    actual - bone_torques[1],    actual    );
				SetWorldTorque( lsja,    actual - bone_torques[0],    actual    );

				SetWorldTorque( rwrist, -bone_torques[7] - lw_actual, actual    );
				SetWorldTorque( relbow,  actual - bone_torques[6],    actual    );
				SetWorldTorque( rsjb,    actual - bone_torques[5],    actual    );
				SetWorldTorque( rsja,    actual - bone_torques[4],    actual    );
			}
		}

		void Update(Soldier* dood, const TimingInfo& time)
		{
			if(!init)
			{
				Init(dood);
				init = true;
			}

			inv_timestep = 1.0f / time.elapsed;

			DoAnchorStuff  (dood, time);
			DoNeckStuff    (dood, time);
			DoArmsAimingGun(dood, time);

			// torso yaw & verticality
			// upper torso orientation?
			// head orientation
			// arms aiming gun (and other stuff the arms can do)
			// left & right leg stuff
		}
	};




	/*
	 * Soldier methods
	 */
	Soldier::Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		imp(NULL),
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
		//posey->active_poses.push_back(p_ag);

		imp = new Imp();
	}

	void Soldier::InnerDispose()
	{
		Dood::InnerDispose();

		if(imp) { delete imp; imp = NULL; }
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
		imp->Update(this, time);
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

		p_ag->pelvis_ori = Quaternion::FromRVec(0, -yaw, 0);

		for(unsigned int i = 0; i < character->skeleton->bones.size(); ++i)
			if(character->skeleton->bones[i]->name == Bone::string_table["l shoulder"] || character->skeleton->bones[i]->name == Bone::string_table["r shoulder"])
				jet_bones.push_back(bone_to_rbody[i]);
	}

	void Soldier::DoInitialPose()
	{
		Dood::DoInitialPose();

		posey->skeleton->GetNamedBone("pelvis")->pos += Vec3(0, 1, 0);

		PreparePAG(TimingInfo(0, 0));
	}

	void Soldier::PreparePAG(const TimingInfo& time)
	{
		p_ag->yaw = yaw;
		p_ag->pitch = pitch;
		p_ag->pelvis_ori = posey->skeleton->bones[0]->ori = Quaternion::FromRVec(0, -(yaw + 0.5f), 0);			// TODO: change this eventually	
		p_ag->UpdatePose(time);

		for(boost::unordered_map<unsigned int, BoneInfluence>::iterator iter = p_ag->bones.begin(); iter != p_ag->bones.end(); ++iter)
			posey->skeleton->GetNamedBone(iter->first)->ori = Quaternion::FromRVec(iter->second.ori);

		posey->skeleton->InvalidateCachedBoneXforms();
	}




	/*
	 * SoldierFoot methods
	 */
	SoldierFoot::SoldierFoot(unsigned int posey_id, const Vec3& ee_pos) : FootState(posey_id, ee_pos) { }

	Quaternion SoldierFoot::OrientBottomToSurface(const Vec3& normal) const
	{
		static const Vec3 dirs[2] = { Vec3(0, 0, 1), Vec3(1, 0, 0) };

		Quaternion foot_ori = body->GetOrientation();
		for(unsigned int i = 0; i < 2; ++i)
		{
			Vec3 dir   = foot_ori * dirs[i];
			Vec3 level = dir - normal * Vec3::Dot(dir, normal);
			Vec3 cross = Vec3::Cross(dir, level);

			float level_mag = level.ComputeMagnitude();
			float cross_mag = cross.ComputeMagnitude();

			if(level_mag != 0 && cross_mag != 0 && fabs(cross_mag) <= fabs(level_mag))
				foot_ori = Quaternion::FromRVec(cross * (asinf(cross_mag / level_mag) / cross_mag)) * foot_ori;
		}

		return foot_ori;
	}
}
