#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"

#include "TestGame.h"

#include "CBone.h"
#include "CJoint.h"
#include "JetpackNozzle.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#include "Particle.h"

#define PROFILE_CPHFT					1
#define PROFILE_ANY_CPHFT				(PROFILE_CPHFT)

#define DIE_AFTER_ONE_SECOND			0

#define ENABLE_NEW_JETPACKING			1

#define MAX_TICK_AGE					300


namespace Test
{
	/*
	 * Soldier constants
	 */
	static const float jump_speed         = 0;//4.0f;

	static const float fly_accel_up       = 15.0f;
	static const float fly_accel_lateral  = 8.0f;

	static const float fuel_spend_rate    = 0;//0.5f;
	static const float fuel_refill_rate   = 0.4f;
	static const float jump_to_fly_delay  = 0;//0.3f;

	static const float torso2_yaw_offset  = 0.5f;


#if PROFILE_ANY_CPHFT
#if PROFILE_CPHFT
	static float timer_init					= 0.0f;
	static float timer_reset				= 0.0f;
	static float timer_massinfo				= 0.0f;
	static float timer_ub_stuff				= 0.0f;

	static float timer_end_of_test			= 0.0f;
	static float timer_cphft_total			= 0.0f;

	static unsigned int counter_cphft		= 0;
#endif

	static void DebugCPHFTProfilingData()
	{
#if PROFILE_CPHFT
		float sum = timer_init + timer_reset + timer_massinfo + timer_ub_stuff + timer_end_of_test;

		Debug(((stringstream&)(stringstream() << "total for " << counter_cphft << " calls to Soldier::Imp::Update = " << timer_cphft_total << endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "init =\t\t\t\t\t"			<< timer_init				<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "reset =\t\t\t\t\t"		<< timer_reset				<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "massinfo =\t\t\t\t"		<< timer_massinfo			<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "ub_stuff =\t\t\t\t"		<< timer_ub_stuff			<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "end_of_test =\t\t\t"		<< timer_end_of_test		<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< sum						<< endl)).str());
#endif
	}
#endif




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
		bool clean_init;
		bool experiment_done;

		CBone *pelvis,    *torso1, *torso2, *head;
		CBone *lshoulder, *luarm,  *llarm,  *lhand;
		CBone *rshoulder, *ruarm,  *rlarm,  *rhand;
		CBone *luleg,     *llleg,  *lheel,  *ltoe;
		CBone *ruleg,     *rlleg,  *rheel,  *rtoe;

		RigidBody* gun_rb;

		CJoint *spine1, *spine2, *neck;
		CJoint *lsja,   *lsjb,   *lelbow, *lwrist;
		CJoint *rsja,   *rsjb,   *relbow, *rwrist;
		CJoint *lhip,   *lknee,  *lankle, *lht;
		CJoint *rhip,   *rknee,  *rankle, *rht;

		float timestep, inv_timestep;

		unsigned int tick_age, max_tick_age;

		Vec3 initial_pos;

		bool jetpacking;
		Vec3 desired_jp_accel;

		Imp() :
			init(false),
			experiment_done(false),
			timestep(0),
			inv_timestep(0),
			tick_age(0),
			max_tick_age(MAX_TICK_AGE)
		{
		}

		~Imp() { }

		void Init(Soldier* dood)
		{
			initial_pos = dood->pos;
			//dood->collision_group->SetInternalCollisionsEnabled(true);		// TODO: resolve problems arising from torso2-arm1 collisions

			// some misc. initialization
			no_touchy.dood = dood;
			for(set<RigidBody*>::iterator iter = dood->velocity_change_bodies.begin(); iter != dood->velocity_change_bodies.end(); ++iter)
				if(*iter != lheel->rb && *iter != rheel->rb && *iter != ltoe->rb && *iter != rtoe->rb)
					(*iter)->SetContactCallback(&no_touchy);

			foot_touchy.dood = dood;
			foot_touchy.standing_callback = &dood->standing_callback;
			for(vector<FootState*>::iterator iter = dood->feet.begin(); iter != dood->feet.end(); ++iter)
				(*iter)->body->SetContactCallback(&foot_touchy);


			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
				gun_rb = gun->rigid_body;
			else
				gun_rb = NULL;

			SharedInit(dood);
		}

		void ReInit(Soldier* dood)
		{
			dood->yaw = dood->pitch = 0.0f;
			dood->pos = initial_pos;

			dood->DoInitialPose();
			dood->posey->skeleton->InvalidateCachedBoneXforms();

			for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
			{
				CBone& cb = *dood->all_bones[i];
				RigidBody& rb = *cb.rb;

				Bone* posey = cb.posey;
				posey->GetTransformationMatrix().Decompose(cb.initial_pos, cb.initial_ori);		// re-computes bones' initial states

				rb.SetPosition   (cb.initial_pos);
				rb.SetOrientation(cb.initial_ori);
				rb.SetLinearVelocity (Vec3());
				rb.SetAngularVelocity(Vec3());
			}

			if(gun_rb != NULL)
			{
				Mat4 xform = ((Gun*)dood->equipped_weapon)->GetInitialXform();

				Vec3 pos = xform.TransformVec3_1(0, 0, 0);
				Vec3 a   = xform.TransformVec3_0(1, 0, 0);
				Vec3 b   = xform.TransformVec3_0(0, 1, 0);
				Vec3 c   = xform.TransformVec3_0(0, 0, 1);

				gun_rb->SetPosition(pos);
				gun_rb->SetOrientation(Quaternion::FromRotationMatrix(Mat3(a.x, b.x, c.x, a.y, b.y, c.y, a.z, b.z, c.z)));
				gun_rb->SetLinearVelocity (Vec3());
				gun_rb->SetAngularVelocity(Vec3());
			}

			SharedInit(dood);
		}

		void SharedInit(Soldier* dood)
		{
			clean_init = false;
			experiment_done = false;
			tick_age = 0;

			//old_contact_points.clear();
			//new_contact_points.clear();
		}



		void GetDesiredTorsoOris(Soldier* dood, Quaternion& p, Quaternion& t1, Quaternion& t2)
		{
			float yaw_relative_to_pelvis = 0.0f;		// = dood->yaw

			float pfrac = dood->pitch * (2.0f / float(M_PI)), pfsq = pfrac * pfrac;

			float t1_yaw   = yaw_relative_to_pelvis + torso2_yaw_offset;
			float t1_pitch = dood->pitch * 0.4f + pfsq * pfrac * 0.95f;
			float t1_yaw2  = pfsq * 0.7f;

			t2 = Quaternion::FromRVec(0, -t1_yaw, 0) * Quaternion::FromRVec(t1_pitch, 0, 0) * Quaternion::FromRVec(0, -t1_yaw2, 0);

			float t2_yaw   = yaw_relative_to_pelvis + torso2_yaw_offset * 0.5f;
			float t2_pitch = pfrac * 0.05f + pfrac * pfsq * 0.3f;
			float t2_yaw2  = pfsq * 0.15f;

			p = Quaternion::FromRVec(0, -t2_yaw, 0) * Quaternion::FromRVec(t2_pitch, 0, 0) * Quaternion::FromRVec(0, -t2_yaw2, 0);

			Quaternion twist_ori = p * Quaternion::Reverse(t2);
			t1 = Quaternion::FromRVec(twist_ori.ToRVec() * -0.5f) * t2;
		}

		void DoHeadOri(Soldier* dood, const TimingInfo& time)
		{
			Quaternion desired_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);

			head->ComputeDesiredTorqueWithDefaultMoI(desired_ori, inv_timestep);
			neck->SetTorqueToSatisfyB();
		}

		void DoArmsAimingGun(Soldier* dood, const TimingInfo& time, const Quaternion& t2ori)
		{
			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				// compute desired per-bone net torques
				MassInfo hng_mass_infos[] = { lhand->rb->GetTransformedMassInfo(), rhand->rb->GetTransformedMassInfo(), gun->rigid_body->GetTransformedMassInfo() };
				Mat3 hng_moi = Mat3(MassInfo::Sum(hng_mass_infos, 3).moi);

				//dood->PreparePAG(time, t2ori);

				//lhand    ->ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				llarm    ->ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				luarm    ->ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				lshoulder->ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				//rhand    ->ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				rlarm    ->ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				ruarm    ->ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				rshoulder->ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				set<RigidBody*> hng_set;
				hng_set.insert(lhand->rb);
				hng_set.insert(rhand->rb);
				hng_set.insert(gun->rigid_body);
				float hng_mass;
				Vec3 hng_com;
				Vec3 hng_vel;
				Vec3 hng_amom;
				ComputeMomentumStuff(hng_set, hng_mass, hng_com, hng_vel, hng_amom);

				Quaternion desired_gun_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);		// same as desired_head_ori
				Vec3 desired_gun_rot = (desired_gun_ori * Quaternion::Reverse(gun->rigid_body->GetOrientation())).ToRVec() * -inv_timestep;
				Vec3 desired_gun_aaccel = (desired_gun_rot - Mat3::Invert(hng_moi) * hng_amom) * inv_timestep;
				Vec3 gun_undo = hng_moi * -desired_gun_aaccel;

				// compute applied joint torques to achieve the per-bone applied torques we just came up with
				lwrist->SetWorldTorque(gun_undo * 0.25f);
				rwrist->SetWorldTorque(gun_undo - lwrist->actual);
				lwrist->SetWorldTorque(gun_undo - rwrist->actual);
				
				lelbow->SetTorqueToSatisfyB();
				lsjb  ->SetTorqueToSatisfyB();
				lsja  ->SetTorqueToSatisfyB();

				relbow->SetTorqueToSatisfyB();
				rsjb  ->SetTorqueToSatisfyB();
				rsja  ->SetTorqueToSatisfyB();
			}
		}

		void ComputeMomentumStuff(const set<RigidBody*>& included_rbs, float& dood_mass, Vec3& dood_com, Vec3& com_vel, Vec3& angular_momentum)
		{
			com_vel = Vec3();
			dood_mass = 0.0f;
			for(set<RigidBody*>::iterator iter = included_rbs.begin(); iter != included_rbs.end(); ++iter)
			{
				RigidBody* rb = *iter;
				float mass = rb->GetMass();
				dood_mass += mass;
				dood_com  += rb->GetCachedCoM() * mass;
				com_vel   += rb->GetLinearVelocity() * mass;
			}
			dood_com /= dood_mass;
			com_vel  /= dood_mass;

			MassInfo mass_info;
			Mat3& moi = *((Mat3*)((void*)mass_info.moi));						// moi.values and mass_info.moi occupy the same space in memory

			for(set<RigidBody*>::iterator iter = included_rbs.begin(); iter != included_rbs.end(); ++iter)
			{
				RigidBody* body = *iter;
				mass_info = body->GetTransformedMassInfo();

				// linear component
				float mass  = mass_info.mass;
				Vec3 vel    = body->GetLinearVelocity() - com_vel;
				Vec3 radius = mass_info.com - dood_com;
				angular_momentum += Vec3::Cross(vel, radius) * mass;

				// angular component
				angular_momentum += moi * body->GetAngularVelocity();
			}
		}

		void Update(Soldier* dood, const TimingInfo& time)
		{
#if PROFILE_CPHFT
			ProfilingTimer timer, timer2;
			timer2.Start();
			timer.Start();
#endif

			if(!init)
			{
				Init(dood);
				init = true;
				return;
			}
			else if(experiment_done)
			{
				ReInit(dood);
				return;
			}

			if(!clean_init)
			{
				ReInit(dood);
				//if(Random3D::RandInt() % 2 == 0)
				//	clean_init = true;
				//else
				//	return;

				clean_init = true;
			}

			timestep     = time.elapsed;
			inv_timestep = 1.0f / timestep;

			if (Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				gun_rb = gun->rigid_body;
				gun_rb->ComputeInvMoI();			// force recomputation of a few cached fields, including ori_rm 
			}
			else
				gun_rb = NULL;


#if PROFILE_CPHFT
			timer_init += timer.GetAndRestart();
#endif

			// reset all the joints and bones
			int ji = 0;
			for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
			{
				CJoint& joint = *dood->all_joints[i];
				joint.Reset();
				if(tick_age == 0)
				{
					joint.sjc->apply_torque = Vec3();
					joint.actual = Vec3();
				}
				joint.SetOrientedTorque(Vec3());
			}
			for (vector<CBone*>::iterator iter = dood->all_bones.begin(); iter != dood->all_bones.end(); ++iter)
			{
				(*iter)->Reset(inv_timestep);
				(*iter)->rb->ComputeInvMoI();			// force recomputation of a few cached fields, including ori_rm 
			}

#if PROFILE_CPHFT
			timer_reset += timer.GetAndRestart();
#endif

			float dood_mass;
			Vec3 dood_com, com_vel, angular_momentum;
			ComputeMomentumStuff(dood->velocity_change_bodies, dood_mass, dood_com, com_vel, angular_momentum);

#if PROFILE_CPHFT
			timer_massinfo += timer.GetAndRestart();
#endif

			// upper body stuff; mostly working
			Quaternion p, t1, t2;
			Quaternion yaw_ori = Quaternion::FromAxisAngle(0, 1, 0, -dood->yaw);
			Mat3 yawmat = yaw_ori.ToMat3();
			Mat3 unyaw = yawmat.Transpose();

			for(vector<JetpackNozzle>::iterator iter = dood->jetpack_nozzles.begin(); iter != dood->jetpack_nozzles.end(); ++iter)
				iter->SolverInit(dood_com, 0.0f);

			for(vector<CJoint*>::iterator iter = dood->all_joints.begin(); iter != dood->all_joints.end(); ++iter)
				(*iter)->SetOrientedTorque(Vec3());
			
			GetDesiredTorsoOris(dood, p, t1, t2);

			DoHeadOri      ( dood, time     );
			DoArmsAimingGun( dood, time, t2 );

			Quaternion desired_head_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);

			pelvis->posey->ori = Quaternion::FromRVec(0, -dood->yaw, 0) * p;
			torso1->posey->ori = t1 * Quaternion::Reverse(p);
			torso2->posey->ori = t2 * Quaternion::Reverse(t1);
			head  ->posey->ori = desired_head_ori * Quaternion::Reverse(t2);

			torso2->ComputeDesiredTorqueWithDefaultMoIAndPosey(inv_timestep);
			torso1->ComputeDesiredTorqueWithDefaultMoIAndPosey(inv_timestep);
			pelvis->ComputeDesiredTorqueWithDefaultMoIAndPosey(inv_timestep);

			neck  ->SetTorqueToSatisfyB();
			spine2->SetTorqueToSatisfyB();
			spine1->SetTorqueToSatisfyB();

			//pelvis->rb->SetOrientation(Quaternion::FromRVec(0, -dood->yaw, 0) * p);			// cheat

			dood->DoScriptedMotorControl("Files/Scripts/soldier_motor_control.lua");

#if ENABLE_NEW_JETPACKING
			if(true)	//if(jetpacking)		// TODO: change this back (?)
			{
				for(vector<JetpackNozzle>::iterator iter = dood->jetpack_nozzles.begin(); iter != dood->jetpack_nozzles.end(); ++iter)
					iter->ApplySelectedForce(timestep);
			}
			else
			{
				for(vector<JetpackNozzle>::iterator iter = dood->jetpack_nozzles.begin(); iter != dood->jetpack_nozzles.end(); ++iter)
					iter->Reset();
			}

#endif

#if PROFILE_CPHFT
			timer_ub_stuff += timer.GetAndRestart();
#endif

			// update the timer, and check for when the experiment is over
			++tick_age;

#if 0
			if(tick_age >= MAX_TICK_AGE)
				experiment_done = true;
#endif

#if PROFILE_CPHFT
			timer_end_of_test += timer.GetAndRestart();
			timer_cphft_total += timer2.Stop();

			++counter_cphft;
#endif
		}

		struct NoTouchy : public ContactCallback
		{
			Dood* dood;
			void OnContact(const ContactPoint& contact) { dood->new_contact_points.push_back(MyContactPoint(contact, dood)); }
			void AfterResolution(const ContactPoint& cp) { dood->old_contact_points.push_back(MyContactPoint(cp, dood)); }
		} no_touchy;

		struct FootTouchy : public ContactCallback
		{
			Dood::StandingCallback* standing_callback;
			Dood* dood;
			void OnContact(const ContactPoint& contact) { standing_callback->OnContact(contact); dood->new_contact_points.push_back(MyContactPoint(contact, dood)); }
			void AfterResolution(const ContactPoint& cp) { standing_callback->AfterResolution(cp); dood->old_contact_points.push_back(MyContactPoint(cp, dood)); }
		} foot_touchy;
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
		jet_fuel(1.0f),
		jet_start_sound(NULL),
		jet_loop_sound(NULL),
		jet_loop(NULL)
	{
		use_cheaty_ori = false;

		//yaw = Random3D::Rand(float(M_PI) * 2.0f);

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
		imp->desired_jp_accel = Vec3();
		if(control_state->GetBoolControl("jump"))
		{
			if(standing_callback.IsStanding() && time.total > jump_start_timer && jump_to_fly_delay > 0)							// jump off the ground
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

						imp->desired_jp_accel = fly_accel_vec;

						// TODO: move jetpack nozzle control logic here?  (if it doesn't stay in lua script)
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

		imp->jetpacking = jetted;

		if(!jetted && jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}

		if(can_recharge)
			jet_fuel = min(jet_fuel + fuel_refill_rate * timestep, 1.0f);
	}

	void Soldier::PreUpdatePoses(const TimingInfo& time) { imp->Update(this, time); }

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
		feet.push_back(new SoldierFoot(Bone::string_table["l toe" ], Vec3( 0.24f, 0.00f,  0.185f )));
		feet.push_back(new SoldierFoot(Bone::string_table["l heel"], Vec3( 0.23f, 0.00f, -0.07f  )));
		
		feet.push_back(new SoldierFoot(Bone::string_table["r toe" ], Vec3(-0.24f, 0.00f,  0.185f )));
		feet.push_back(new SoldierFoot(Bone::string_table["r heel"], Vec3(-0.23f, 0.00f, -0.07f  )));
	}

	void Soldier::Update(const TimingInfo& time)
	{
#if DIE_AFTER_ONE_SECOND
		if(time.total > 1.0f)
			Die(Damage());
#endif

		Dood::Update(time);
	}

	void Soldier::Vis(SceneRenderer* renderer)
	{
		Dood::Vis(renderer);

		Vec3 forward = renderer->camera->GetForward();
		BillboardMaterial* jetpack_trail = (BillboardMaterial*)((TestGame*)game_state)->mat_cache->Load("jetpack");
		for(unsigned int i = 0; i < jetpack_nozzles.size(); ++i)
			jetpack_nozzles[i].Vis(renderer, forward, jetpack_trail);
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

		p_ag->torso2_ori = Quaternion::FromRVec(0, -(yaw + torso2_yaw_offset), 0);
	}

	void Soldier::DeSpawned()
	{
		Dood::DeSpawned();

		if(jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}
	}

	void Soldier::DoInitialPose()
	{
		Quaternion yaw_ori = Quaternion::FromAxisAngle(0, 1, 0, -yaw);

		pos.y -= 0.01f;

		Dood::DoInitialPose();
		
		/*posey->skeleton->GetNamedBone( "pelvis" )->ori = Quaternion::FromRVec( 0.737025f, -1.70684f, 1.94749f );
		posey->skeleton->GetNamedBone( "torso 1" )->ori = Quaternion::FromRVec( -0.0951396f, -0.00468156f, 0.719183f );
		posey->skeleton->GetNamedBone( "torso 2" )->ori = Quaternion::FromRVec( -0.305723f, -0.320675f, 0.740507f );
		posey->skeleton->GetNamedBone( "head" )->ori = Quaternion::FromRVec( -0.0283016f, 1.12843f, -0.362935f );
		posey->skeleton->GetNamedBone( "l shoulder" )->ori = Quaternion::FromRVec( -0.252179f, -0.111962f, -0.197518f );
		posey->skeleton->GetNamedBone( "l arm 1" )->ori = Quaternion::FromRVec( -1.3233f, 0.628318f, 0.587629f );
		posey->skeleton->GetNamedBone( "l arm 2" )->ori = Quaternion::FromRVec( -0.871514f, -0.569575f, 0.815938f );
		posey->skeleton->GetNamedBone( "l hand" )->ori = Quaternion::FromRVec( -1.76684f, -1.11402f, 0.13909f );
		posey->skeleton->GetNamedBone( "r shoulder" )->ori = Quaternion::FromRVec( -0.242087f, -0.0216953f, -0.191872f );
		posey->skeleton->GetNamedBone( "r arm 1" )->ori = Quaternion::FromRVec( 0.130442f, -0.196354f, 1.29011f );
		posey->skeleton->GetNamedBone( "r arm 2" )->ori = Quaternion::FromRVec( 0.247124f, -0.615489f, 0.621803f );
		posey->skeleton->GetNamedBone( "r hand" )->ori = Quaternion::FromRVec( -0.394564f, 1.23705f, 0.908416f );
		posey->skeleton->GetNamedBone( "l leg 1" )->ori = Quaternion::FromRVec( -0.244577f, 0.549211f, 0.205488f );
		posey->skeleton->GetNamedBone( "l leg 2" )->ori = Quaternion::FromRVec( 0.583199f, -0.0798103f, 0.00289411f );
		posey->skeleton->GetNamedBone( "l heel" )->ori = Quaternion::FromRVec( -0.130158f, -0.130852f, 0.390745f );
		posey->skeleton->GetNamedBone( "l toe" )->ori = Quaternion::FromRVec( 0.00874208f, -0.0782354f, 0.0494762f );
		posey->skeleton->GetNamedBone( "r leg 1" )->ori = Quaternion::FromRVec( -0.550917f, 0.103198f, 0.775355f );
		posey->skeleton->GetNamedBone( "r leg 2" )->ori = Quaternion::FromRVec( 0.728848f, -0.12879f, -0.798952f );
		posey->skeleton->GetNamedBone( "r heel" )->ori = Quaternion::FromRVec( 0.386118f, -0.00473954f, 0.0593444f );
		posey->skeleton->GetNamedBone( "r toe" )->ori = Quaternion::FromRVec( -0.24557f, 0.151427f, 0.0580851f );*/

		Quaternion p, t1, t2;
		imp->GetDesiredTorsoOris(this, p, t1, t2);

		posey->skeleton->GetNamedBone( "pelvis"  )->ori = p;
		posey->skeleton->GetNamedBone( "torso 1" )->ori = t1 * Quaternion::Reverse(p);
		posey->skeleton->GetNamedBone( "torso 2" )->ori = t2 * Quaternion::Reverse(t1);

		posey->skeleton->GetNamedBone( "l leg 1" )->ori = Quaternion::FromRVec( -0.02f,  0.03f,  0.01f );
		posey->skeleton->GetNamedBone( "l leg 2" )->ori = Quaternion::FromRVec(  0.04f,  0,      0     );
		posey->skeleton->GetNamedBone( "l heel"  )->ori = Quaternion::FromRVec( -0.02f,  0.03f, -0.01f );

		posey->skeleton->GetNamedBone( "r leg 1" )->ori = Quaternion::FromRVec( -0.02f, -0.03f, -0.01f );
		posey->skeleton->GetNamedBone( "r leg 2" )->ori = Quaternion::FromRVec(  0.04f,  0,      0     );
		posey->skeleton->GetNamedBone( "r heel"  )->ori = Quaternion::FromRVec( -0.02f, -0.03f,  0.01f );


#if 0
		Quaternion qlist[6] =
		{
			Quaternion( 0.98935f,   0.058987f,    0.124063f,    -0.0481096f   ),
			Quaternion( 1.0f,      -0.0001091f,   0.000762187f,  0.000103048f ),
			Quaternion( 0.985989f, -0.0697347f,   0.148507f,     0.0301456f   ),
			Quaternion( 0.995083f, -0.017937f,   -0.0915855f,   -0.033182f    ),
			Quaternion( 0.999651f,  0.022753f,   -0.0133616f,   -0.00111608f  ),
			Quaternion( 0.996213f, -0.00356901f,  0.0807469f,    0.0320568f   ),
		};
		posey->skeleton->GetNamedBone( "l leg 1" )->ori = qlist[0];
		posey->skeleton->GetNamedBone( "l leg 2" )->ori = qlist[1];
		posey->skeleton->GetNamedBone( "l heel"  )->ori = qlist[2];
		posey->skeleton->GetNamedBone( "r leg 1" )->ori = qlist[3];
		posey->skeleton->GetNamedBone( "r leg 2" )->ori = qlist[4];
		posey->skeleton->GetNamedBone( "r heel"  )->ori = qlist[5];
#endif

		PreparePAG(TimingInfo(), t2);

		//for(unsigned int i = 0; i < posey->skeleton->bones.size(); ++i)
		//	posey->skeleton->bones[i]->ori = Quaternion::Identity();
	}

	void Soldier::InitBoneHelpers()
	{
		imp->pelvis    = GetBone( "pelvis"     );
		imp->torso1    = GetBone( "torso 1"    );
		imp->torso2    = GetBone( "torso 2"    );
		imp->head      = GetBone( "head"       );
		imp->lshoulder = GetBone( "l shoulder" );
		imp->luarm     = GetBone( "l arm 1"    );
		imp->llarm     = GetBone( "l arm 2"    );
		imp->lhand     = GetBone( "l hand"     );
		imp->rshoulder = GetBone( "r shoulder" );
		imp->ruarm     = GetBone( "r arm 1"    );
		imp->rlarm     = GetBone( "r arm 2"    );
		imp->rhand     = GetBone( "r hand"     );
		imp->luleg     = GetBone( "l leg 1"    );
		imp->llleg     = GetBone( "l leg 2"    );
		imp->lheel     = GetBone( "l heel"     );
		imp->ltoe      = GetBone( "l toe"      );
		imp->ruleg     = GetBone( "r leg 1"    );
		imp->rlleg     = GetBone( "r leg 2"    );
		imp->rheel     = GetBone( "r heel"     );
		imp->rtoe      = GetBone( "r toe"      );
	}

	void Soldier::InitJointHelpers()
	{
		imp->spine2 = GetJoint( "torso 2"    );
		imp->spine1 = GetJoint( "torso 1"    );
		imp->neck   = GetJoint( "head"       );
		imp->lsja   = GetJoint( "l shoulder" );
		imp->lsjb   = GetJoint( "l arm 1"    );
		imp->lelbow = GetJoint( "l arm 2"    );
		imp->lwrist = GetJoint( "l hand"     );
		imp->rsja   = GetJoint( "r shoulder" );
		imp->rsjb   = GetJoint( "r arm 1"    );
		imp->relbow = GetJoint( "r arm 2"    );
		imp->rwrist = GetJoint( "r hand"     );
		imp->lhip   = GetJoint( "l leg 1"    );
		imp->lknee  = GetJoint( "l leg 2"    );
		imp->lankle = GetJoint( "l heel"     );
		imp->lht    = GetJoint( "l toe"      );
		imp->rhip   = GetJoint( "r leg 1"    );
		imp->rknee  = GetJoint( "r leg 2"    );
		imp->rankle = GetJoint( "r heel"     );
		imp->rht    = GetJoint( "r toe"      );
	}

	void Soldier::InitJetpackNozzles()
	{
		Dood::InitJetpackNozzles();

		Vec3 upward(0, 1, 0);
		float jp_angle = 1.0f;
		float jp_force = 200.0f;//150.0f;		// 98kg * 15m/s^2 accel / 10 nozzles ~= 150N per nozzle

		RegisterSymmetricJetpackNozzles( imp->lshoulder, imp->rshoulder, Vec3( 0.442619f, 1.576419f, -0.349652f ), upward, jp_angle, jp_force );
		RegisterSymmetricJetpackNozzles( imp->lshoulder, imp->rshoulder, Vec3( 0.359399f, 1.523561f, -0.366495f ), upward, jp_angle, jp_force );
		RegisterSymmetricJetpackNozzles( imp->lshoulder, imp->rshoulder, Vec3( 0.277547f, 1.480827f, -0.385142f ), upward, jp_angle, jp_force );
		RegisterSymmetricJetpackNozzles( imp->ltoe,      imp->rtoe,      Vec3( 0.237806f, 0.061778f,  0.038247f ), upward, jp_angle, jp_force );
		RegisterSymmetricJetpackNozzles( imp->lheel,     imp->rheel,     Vec3( 0.238084f, 0.063522f, -0.06296f  ), upward, jp_angle, jp_force );
	}

	void Soldier::PreparePAG(const TimingInfo& time, const Quaternion& t2ori)
	{
		p_ag->yaw   = yaw;
		p_ag->pitch = pitch;
		p_ag->torso2_ori = t2ori;
		p_ag->UpdatePose(time);

		for(unordered_map<unsigned int, BoneInfluence>::iterator iter = p_ag->bones.begin(); iter != p_ag->bones.end(); ++iter)
			posey->skeleton->GetNamedBone(iter->first)->ori = Quaternion::FromRVec(iter->second.ori);

		posey->skeleton->InvalidateCachedBoneXforms();
	}

	bool Soldier::GetRBScriptingName(RigidBody* rb, string& name)
	{
		if(rb != NULL && rb == imp->gun_rb)
		{
			name = "gun";
			return true;
		}

		return false;
	}

	Vec3 Soldier::GetDesiredJetpackAccel() { return imp->desired_jp_accel; }
	int Soldier::GetTickAge() { return imp->tick_age; }

	void Soldier::PreCPHFT(float timestep) { Dood::PreCPHFT(timestep); }
	void Soldier::PostCPHFT(float timestep) { Dood::PostCPHFT(timestep); old_contact_points.clear(); new_contact_points.clear(); }




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
