#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"
#include "PlacedFootConstraint.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#define DIE_AFTER_ONE_SECOND   0

#define ENABLE_PELVIS_ANCHOR   0

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

	static const float torso2_yaw_offset  = 0.5f;




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

		struct CBone
		{
			string name;
			RigidBody* rb;
			Bone* posey;

			Vec3 desired_torque;
			Vec3 applied_torque;

			CBone() { }
			CBone(const Soldier* dood, const string& name) : name(name), rb(dood->RigidBodyForNamedBone(name)), posey(dood->posey->skeleton->GetNamedBone(name)) { }

			void Reset() { desired_torque = applied_torque = Vec3(); }

			void ComputeDesiredTorque(const Quaternion& desired_ori, const Mat3& use_moi, float inv_timestep)
			{
				Quaternion ori          = rb->GetOrientation();
				Vec3 rot                = rb->GetAngularVelocity();

				Vec3 desired_rot    = (desired_ori * Quaternion::Reverse(ori)).ToRVec() * -inv_timestep;
				Vec3 desired_aaccel = (desired_rot - rot) * inv_timestep;

				desired_torque = use_moi * desired_aaccel;
			}

			void ComputeDesiredTorqueWithDefaultMoI(const Quaternion& desired_ori, float inv_timestep) { ComputeDesiredTorque(desired_ori, Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }
			void ComputeDesiredTorqueWithPosey(const Mat3& use_moi, float inv_timestep)                { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), use_moi, inv_timestep); }
			void ComputeDesiredTorqueWithDefaultMoIAndPosey(float inv_timestep)                        { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }
		};

		CBone pelvis,    torso1, torso2, head;
		CBone lshoulder, luarm,  llarm,  lhand;
		CBone rshoulder, ruarm,  rlarm,  rhand;
		CBone luleg,     llleg,  lfoot;
		CBone ruleg,     rlleg,  rfoot;

		struct CJoint
		{
			SkeletalJointConstraint* sjc;
			CBone *a, *b;

			Vec3 actual;

			Mat3 oriented_axes;			// gets recomputed every time Reset() is called

			CJoint() { }
			CJoint(const Dood* dood, CBone& bone_a, CBone& bone_b, float max_torque)
			{
				RigidBody *arb = bone_a.rb, *brb = bone_b.rb;
				for(unsigned int i = 0; i < dood->constraints.size(); ++i)
				{
					SkeletalJointConstraint* j = (SkeletalJointConstraint*)dood->constraints[i];
					if(j->obj_a == arb && j->obj_b == brb)
					{
						a   = &bone_a;
						b   = &bone_b;
						sjc = j;

						sjc->min_torque = Vec3(-max_torque, -max_torque, -max_torque);
						sjc->max_torque = Vec3( max_torque,  max_torque,  max_torque);

						return;
					}
				}

				// joint not found?
				a = b = NULL;
				sjc = NULL;
			}

			void Reset()
			{
				sjc->apply_torque = actual = Vec3();
				oriented_axes = sjc->axes * Quaternion::Reverse(sjc->obj_a->GetOrientation()).ToMat3();
			}

			bool SetWorldTorque(const Vec3& torque)
			{
				Vec3 local_torque = oriented_axes * torque;

				const Vec3 &mint = sjc->min_torque, &maxt = sjc->max_torque;

				Vec3 old = oriented_axes.TransposedMultiply(sjc->apply_torque);

				sjc->apply_torque.x = max(mint.x, min(maxt.x, local_torque.x));
				sjc->apply_torque.y = max(mint.y, min(maxt.y, local_torque.y));
				sjc->apply_torque.z = max(mint.z, min(maxt.z, local_torque.z));

				Vec3 dif = sjc->apply_torque - local_torque;
				bool result = (dif.x != 0 || dif.y != 0 || dif.z != 0);

				if(result)
					actual = oriented_axes.TransposedMultiply(sjc->apply_torque);
				else
					actual = torque;

				Vec3 delta = actual - old;

				b->applied_torque -= delta;
				a->applied_torque += delta;

				return result;
			}

			bool SetTorqueToSatisfyA() { return SetWorldTorque(a->desired_torque - (a->applied_torque - actual)); }
			bool SetTorqueToSatisfyB() { return SetWorldTorque((b->applied_torque + actual) - b->desired_torque); }
		};

		CJoint spine1, spine2, neck;
		CJoint lsja,   lsjb,   lelbow, lwrist;
		CJoint rsja,   rsjb,   relbow, rwrist;
		CJoint lhip,   lknee,  lankle;
		CJoint rhip,   rknee,  rankle;

		float inv_timestep;

		Imp() :
			init(false),
			inv_timestep(0)
		{
		}

		~Imp() { }

		void Init(Soldier* dood)
		{
			Debug("Soldier::Imp::Init()\n");

			//dood->collision_group->SetInternalCollisionsEnabled(true);		// TODO: resolve problems arising from torso2-arm1 collisions

			pelvis    = CBone( dood, "pelvis"     );
			torso1    = CBone( dood, "torso 1"    );
			torso2    = CBone( dood, "torso 2"    );
			head      = CBone( dood, "head"       );
			lshoulder = CBone( dood, "l shoulder" );
			luarm     = CBone( dood, "l arm 1"    );
			llarm     = CBone( dood, "l arm 2"    );
			lhand     = CBone( dood, "l hand"     );
			rshoulder = CBone( dood, "r shoulder" );
			ruarm     = CBone( dood, "r arm 1"    );
			rlarm     = CBone( dood, "r arm 2"    );
			rhand     = CBone( dood, "r hand"     );
			luleg     = CBone( dood, "l leg 1"    );
			llleg     = CBone( dood, "l leg 2"    );
			lfoot     = CBone( dood, "l foot"     );
			ruleg     = CBone( dood, "r leg 1"    );
			rlleg     = CBone( dood, "r leg 2"    );
			rfoot     = CBone( dood, "r foot"     );

			float SP = 1500, N = 150, W = 200, E = 350, SB = 600, SA = 700, H = 1400, K = 800, A = 500;
			spine1 = CJoint( dood, pelvis,    torso1,    SP );
			spine2 = CJoint( dood, torso1,    torso2,    SP );
			neck   = CJoint( dood, torso2,    head,      N  );
			lsja   = CJoint( dood, torso2,    lshoulder, SA );
			lsjb   = CJoint( dood, lshoulder, luarm,     SB );
			lelbow = CJoint( dood, luarm,     llarm,     E  );
			lwrist = CJoint( dood, llarm,     lhand,     W  );
			rsja   = CJoint( dood, torso2,    rshoulder, SA );
			rsjb   = CJoint( dood, rshoulder, ruarm,     SB );
			relbow = CJoint( dood, ruarm,     rlarm,     E  );
			rwrist = CJoint( dood, rlarm,     rhand,     W  );
			lhip   = CJoint( dood, pelvis,    luleg,     H  );
			lknee  = CJoint( dood, luleg,     llleg,     K  );
			lankle = CJoint( dood, llleg,     lfoot,     A  );
			rhip   = CJoint( dood, pelvis,    ruleg,     H  );
			rknee  = CJoint( dood, ruleg,     rlleg,     K  );
			rankle = CJoint( dood, rlleg,     rfoot,     A  );
		}

		void GetDesiredTorsoOris(Soldier* dood, Quaternion& p, Quaternion& t1, Quaternion& t2)
		{
			float pfrac = dood->pitch * (2.0f / float(M_PI)), pfsq = pfrac * pfrac;

			float t1_yaw   = dood->yaw + torso2_yaw_offset;
			float t1_pitch = dood->pitch * 0.4f + pfsq * pfrac * 0.95f;
			float t1_yaw2  = pfsq * 0.7f;

			t2 = Quaternion::FromRVec(0, -t1_yaw, 0) * Quaternion::FromRVec(t1_pitch, 0, 0) * Quaternion::FromRVec(0, -t1_yaw2, 0);

			float t2_yaw   = dood->yaw + torso2_yaw_offset * 0.5f;
			float t2_pitch = pfrac * 0.05f + pfrac * pfsq * 0.3f;
			float t2_yaw2  = pfsq * 0.15f;

			p = Quaternion::FromRVec(0, -t2_yaw, 0) * Quaternion::FromRVec(t2_pitch, 0, 0) * Quaternion::FromRVec(0, -t2_yaw2, 0);

			Quaternion twist_ori = p * Quaternion::Reverse(t2);
			t1 = Quaternion::FromRVec(twist_ori.ToRVec() * -0.5f) * t2;
		}

		void DoAnchorStuff(Soldier* dood, const TimingInfo& time, const Quaternion& p)
		{
			static const float helper_frac = 1.0f, rest_frac = 1.0f - helper_frac;

			Quaternion current_ori = pelvis.rb->GetOrientation();
			Quaternion ctd         = current_ori * Quaternion::Reverse(p);
			Quaternion use_ori     = Quaternion::FromRVec(ctd.ToRVec() * rest_frac) * p;

			Vec3 local_com = pelvis.rb->GetMassInfo().com;

			pelvis.rb->SetPosition(Vec3(0, 1, 0) + local_com - use_ori * local_com);
			pelvis.rb->SetOrientation(use_ori);
			pelvis.rb->SetLinearVelocity(Vec3());
			pelvis.rb->SetAngularVelocity(Vec3());
		}

		void DoHeadOri(Soldier* dood, const TimingInfo& time)
		{
			Quaternion desired_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);

			head.ComputeDesiredTorqueWithDefaultMoI(desired_ori, inv_timestep);
			neck.SetTorqueToSatisfyB();
		}

		void DoArmsAimingGun(Soldier* dood, const TimingInfo& time, const Quaternion& t2ori)
		{
			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				// compute desired per-bone net torques
				MassInfo hng_mass_infos[] = { lhand.rb->GetTransformedMassInfo(), rhand.rb->GetTransformedMassInfo(), gun->rigid_body->GetTransformedMassInfo() };
				Mat3 hng_moi = Mat3(MassInfo::Sum(hng_mass_infos, 3).moi);

				dood->PreparePAG(time, t2ori);

				lhand    .ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				llarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				luarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				lshoulder.ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				rhand    .ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				rlarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				ruarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				rshoulder.ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				// compute applied joint torques to achieve the per-bone applied torques we just came up with
				lwrist.SetWorldTorque(-lhand.desired_torque * 0.75f);
				rwrist.SetWorldTorque(-lwrist.actual - rhand.desired_torque);
				
				lelbow.SetTorqueToSatisfyB();
				lsjb  .SetTorqueToSatisfyB();
				lsja  .SetTorqueToSatisfyB();

				relbow.SetTorqueToSatisfyB();
				rsjb  .SetTorqueToSatisfyB();
				rsja  .SetTorqueToSatisfyB();
			}
		}

		void DoTorsoOris(Soldier* dood, const TimingInfo& time, const Quaternion& t1, const Quaternion& t2)
		{
			// figure out what net torques we want to apply to each bone
			torso2.ComputeDesiredTorqueWithDefaultMoI(t2, inv_timestep);
			torso1.ComputeDesiredTorqueWithDefaultMoI(t1, inv_timestep);

			// come up with joint torques to accomplish those net torques
			spine2.SetTorqueToSatisfyB();
			spine1.SetTorqueToSatisfyB();
		}

		void DoKneeCompromise(CJoint& knee)
		{
			knee.SetTorqueToSatisfyB();
			Vec3 temp = knee.actual;
			knee.SetTorqueToSatisfyA();
			knee.SetWorldTorque((temp + knee.actual) * 0.5f);
		}

		void DoLegStuff(Soldier* dood, const TimingInfo& time, const Quaternion& p)
		{
			return;

			// TODO: come up with a pose that satisfies IK constraints for placed feet, and the pelvis pos/ori, while somehow being achievable with joint torques
			Quaternion sdo = Quaternion::FromRVec(0, -dood->yaw, 0);
			Quaternion desired_oris[7] = { p, sdo, sdo, sdo, sdo, sdo, sdo };

			pelvis.ComputeDesiredTorqueWithDefaultMoI(desired_oris[0], inv_timestep);
			luleg.ComputeDesiredTorqueWithDefaultMoI (desired_oris[1], inv_timestep);
			ruleg.ComputeDesiredTorqueWithDefaultMoI (desired_oris[2], inv_timestep);
			llleg.ComputeDesiredTorqueWithDefaultMoI (desired_oris[3], inv_timestep);
			rlleg.ComputeDesiredTorqueWithDefaultMoI (desired_oris[4], inv_timestep);
			lfoot.ComputeDesiredTorqueWithDefaultMoI (desired_oris[5], inv_timestep);
			rfoot.ComputeDesiredTorqueWithDefaultMoI (desired_oris[6], inv_timestep);

			lhip  .SetTorqueToSatisfyB();
			rhip  .SetTorqueToSatisfyB();
			lknee .SetTorqueToSatisfyB();
			rknee .SetTorqueToSatisfyB();
			lankle.SetTorqueToSatisfyB();
			rankle.SetTorqueToSatisfyB();

			
			// magic brain nonsense
			Vec3 translation = -pelvis.rb->GetCenterOfMass();
			Mat3 rotation = Mat3::FromAxisAngle(0, 1, 0, dood->yaw);

			struct AddRBToVector
			{
				// one rb = 6 Vec3 values
				AddRBToVector(RigidBody* rb, const Vec3& translation, const Mat3& rotation, vector<Vec3>& v)
				{
					Mat3 rb_rot = rotation * rb->GetOrientation().ToMat3();

					v.push_back(rotation * (rb->GetPosition() + translation));		// pos
					v.push_back(Vec3(rb_rot[0], rb_rot[1], rb_rot[2]));				// ori (rm)
					v.push_back(Vec3(rb_rot[3], rb_rot[4], rb_rot[6]));
					v.push_back(Vec3(rb_rot[6], rb_rot[7], rb_rot[8]));
					v.push_back(rotation * rb->GetLinearVelocity());				// vel
					v.push_back(rotation * rb->GetAngularVelocity());				// rot
				}
			};

			vector<Vec3> inputs;

			// 18 bones + gun = 19 rbs = 114 vec3s
			for(unsigned int i = 0; i < dood->rigid_bodies.size(); ++i)
				AddRBToVector(dood->rigid_bodies[i], translation, rotation, inputs);
			AddRBToVector(((Gun*)dood->equipped_weapon)->rigid_body, translation, rotation, inputs);

			// TODO: consult previous tick's desired values (if there was a previous tick) to score how well this brain achieved its objectives

			// TODO: compute actual desired values for these things
			// left foot
			inputs.push_back(Vec3());		// linear
			inputs.push_back(Vec3());		// angular
			// right foot
			inputs.push_back(Vec3());		// linear
			inputs.push_back(Vec3());		// angular
			// pelvis
			inputs.push_back(Vec3());		// linear
			inputs.push_back(Vec3());		// angular

			// TODO: magic brain thing do stuff here now
		}

		void Update(Soldier* dood, const TimingInfo& time)
		{
			if(!init)
			{
				Init(dood);
				init = true;
			}

			inv_timestep = 1.0f / time.elapsed;

			// reset joints
			spine1.Reset();       spine2.Reset();    neck.Reset();
			lsja.Reset();         lsjb.Reset();      lelbow.Reset();    lwrist.Reset();
			rsja.Reset();         rsjb.Reset();      relbow.Reset();    rwrist.Reset();
			lhip.Reset();         lknee.Reset();     lankle.Reset();
			rhip.Reset();         rknee.Reset();     rankle.Reset();

			// reset bones
			pelvis.Reset();       torso1.Reset();    torso2.Reset();    head.Reset();
			lshoulder.Reset();    luarm.Reset();     llarm.Reset();     lhand.Reset();
			rshoulder.Reset();    ruarm.Reset();     rlarm.Reset();     rhand.Reset();
			luleg.Reset();        llleg.Reset();     lfoot.Reset();
			ruleg.Reset();        rlleg.Reset();     rfoot.Reset();


			// do actual C/PHFT stuff
			Quaternion p, t1, t2;
			GetDesiredTorsoOris(dood, p, t1, t2);

#if ENABLE_PELVIS_ANCHOR
			DoAnchorStuff  ( dood, time, p         );
#endif

			DoHeadOri      ( dood, time            );
			DoArmsAimingGun( dood, time,        t2 );
			DoTorsoOris    ( dood, time,    t1, t2 );
			DoLegStuff     ( dood, time, p         );
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

		p_ag->torso2_ori = Quaternion::FromRVec(0, -(yaw + torso2_yaw_offset), 0);

		unsigned int lshoulder_name = Bone::string_table["l shoulder"], rshoulder_name = Bone::string_table["r shoulder"];
		for(unsigned int i = 0; i < character->skeleton->bones.size(); ++i)
			if(character->skeleton->bones[i]->name == lshoulder_name || character->skeleton->bones[i]->name == rshoulder_name)
				jet_bones.push_back(bone_to_rbody[i]);
	}

	void Soldier::DoInitialPose()
	{
		Dood::DoInitialPose();

#if ENABLE_PELVIS_ANCHOR
		posey->skeleton->GetNamedBone("pelvis")->pos += Vec3(0, 1, 0);
#endif

		PreparePAG(TimingInfo(0, 0), Quaternion::FromRVec(0, -(yaw + torso2_yaw_offset), 0));
	}

	void Soldier::PreparePAG(const TimingInfo& time, const Quaternion& t2ori)
	{
		p_ag->yaw = yaw;
		p_ag->pitch = pitch;
		p_ag->torso2_ori = posey->skeleton->bones[0]->ori = t2ori;
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
