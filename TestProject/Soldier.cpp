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

		Imp() : init(false),
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

		void DoAnchorStuff(Soldier* dood, const TimingInfo& time)
		{
			float yaw   = dood->yaw + 0.4f;
			float pitch = dood->pitch * 0.2f + powf(dood->pitch / float(M_PI / 2), 3.0f) * 1.15f;

			Quaternion desired_ori = Quaternion::FromRVec(0, -yaw, 0) * Quaternion::FromRVec(pitch, 0, 0);
			Quaternion current_ori = anchored_body->GetOrientation();
			Quaternion ctd         = current_ori * Quaternion::Reverse(desired_ori);
			Quaternion use_ori     = Quaternion::FromRVec(ctd.ToRVec() * 0.15f) * desired_ori;

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

			Mat3 oriented_axes = neck->axes * Quaternion::Reverse(head_ori).ToMat3();

			neck->apply_torque = oriented_axes * htorque;
		}

		void DoArmsAimingGun(Soldier* dood, const TimingInfo& time)
		{
			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				static const float helper_frac = 0.05f;
				static const float rest_frac   = 1.0f - helper_frac;

				RigidBody* gun_rb = gun->rigid_body;

				// compute the combined mass info for the hands and gun (HNG)
				RigidBody* hng_rbs  [3] = { lhand, rhand, gun_rb };
				MassInfo   rb_minfos[3] =
				{
					hng_rbs[0]->GetTransformedMassInfo(),
					hng_rbs[1]->GetTransformedMassInfo(),
					hng_rbs[2]->GetTransformedMassInfo()
				};
				MassInfo hng_minfo = MassInfo::Sum(rb_minfos, 3);

				// specify the desired state of the gun											// TODO: do this better
				Quaternion desired_gun_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);
				Vec3 desired_gun_pos = hng_minfo.com;											// desired position for the HNG's center of mass

				// compute torque necessary to achieve the requested orientation
				Quaternion gun_ori        = gun_rb->GetOrientation();							// TODO: maybe use combined HNG ori?
				Quaternion gun_to_desired = gun_ori * Quaternion::Reverse(desired_gun_ori);

				Vec3 gtd_rot = gun_to_desired.ToRVec() * inv_timestep;
				Vec3 gun_rot = gun_rb->GetAngularVelocity();									// TODO: maybe use combined HNG rot?
				Vec3 aaccel  = (gtd_rot - gun_rot) * inv_timestep;
				Vec3 torque  = Mat3(hng_minfo.moi) * aaccel;

				// compute force necessary to move the HNG's center of mass to the requested position
				Vec3 gtd_vel = (desired_gun_pos - hng_minfo.com) * inv_timestep;
				Vec3 gun_vel = gun_rb->GetLinearVelocity();
				Vec3 accel   = (gtd_vel - gun_vel) * inv_timestep;
				Vec3 force   = accel * hng_minfo.mass;


				// TODO: implement this

				// use _sja for ???
				// use _sjb to control direction from _sjb to _wrist
				// use _elbow to control distance from _sjb to _wrist
				// use _wrist to control gun orientation


				// cheaty helper force (for prototyping)
				gun_rb->SetLinearVelocity (gtd_vel * helper_frac + gun_rb->GetLinearVelocity()  * rest_frac);
				gun_rb->SetAngularVelocity(gtd_rot * helper_frac + gun_rb->GetAngularVelocity() * rest_frac);
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
		DoIKStuff(time);
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

		p_ag->yaw = yaw;
		p_ag->pitch = pitch;
		p_ag->pelvis_ori = posey->skeleton->bones[0]->ori;
		p_ag->UpdatePose(TimingInfo(0, 0));

		for(boost::unordered_map<unsigned int, BoneInfluence>::iterator iter = p_ag->bones.begin(); iter != p_ag->bones.end(); ++iter)
			posey->skeleton->GetNamedBone(iter->first)->ori = Quaternion::FromRVec(iter->second.ori);
	}

	void Soldier::DoIKStuff(const TimingInfo& time)
	{
		imp->Update(this, time);	
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
