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

			Vec3 hip, knee, ankle;
			float hk_dist, ka_dist, hksq, kasq;

			float min_hasq, max_hasq;
			float hksq_m_kasq;
			// TODO: add members here as needed

			ModelPhysics::JointPhysics* joint_protos[3];
			Bone* bones[3];

			Vec3 joint_rots[3];

			SoldierFoot(unsigned int posey_id, const Vec3& ee_pos);
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
		//posey->active_poses.push_back(p_ag);

		magic_box_coeffs.clear();
		test_done = false;
		magic_box_score = total_score_weight = 0.0f;
		lifetime = 0.0f;
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

	void Soldier::DoCheatyPose(float timestep, const Vec3& net_vel)
	{
		Dood::DoCheatyPose(timestep, net_vel);

		if(Gun* gun = dynamic_cast<Gun*>(equipped_weapon))
		{
			Mat4 gun_hand_xform = posey->skeleton->GetNamedBone("r hand")->GetTransformationMatrix();
			Mat4 gun_xform = gun_hand_xform * Mat4::FromPositionAndOrientation(Vec3(-0.959f,  1.098f,  0.077f), Quaternion::FromRVec(-Vec3(-1.27667f, 0.336123f, 0.64284f))) * Mat4::Translation(-Vec3(0.000f, -0.063f, -0.152f));
		
			Vec3 pos;
			Quaternion ori;
			gun_xform.Decompose(pos, ori);

			float move_coeff = 1.0f / timestep;

			RigidBody* rb = gun->rigid_body;

			rb->SetLinearVelocity((gun_xform.TransformVec3_1(rb->GetMassInfo().com) - rb->GetCenterOfMass()) * move_coeff);
			rb->SetAngularVelocity((rb->GetOrientation() * Quaternion::Reverse(ori)).ToRVec() * move_coeff);
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
	}


	void Soldier::DoInitialPose()
	{
		Dood::DoInitialPose();

		DoIKStuff(TimingInfo(0, 0));
	}

	void Soldier::DoIKStuff(const TimingInfo& time)
	{
		lifetime += time.elapsed;

		static const string bone_names[7] = { "pelvis", "l leg 1", "l leg 2", "l foot", "r leg 1", "r leg 2", "r foot" };
		
		Bone* bones[7];
		for(unsigned int i = 0; i < 7; ++i)
			bones[i] = posey->skeleton->GetNamedBone(bone_names[i]);

		// special stuff to only do when called by DoInitialPose
		if(lifetime == 0)
		{
			bones[0]->pos.y += magic_box_coeffs[0];
			bones[0]->ori *= Quaternion::FromRVec(magic_box_coeffs[1], 0, 0);
		}

		// stuff that depends on the rigid bodies having been initialized, and thus can't be done for DoInitialPose
		if(lifetime > 0)
		{
			SoldierFoot* lfoot = ((SoldierFoot*)feet[0]);
			SoldierFoot* rfoot = ((SoldierFoot*)feet[1]);

			RigidBody* rbs[7] = {
				RigidBodyForNamedBone(bone_names[0]),		// pelvis
				RigidBodyForNamedBone(bone_names[1]),		// right leg
				RigidBodyForNamedBone(bone_names[2]),
				RigidBodyForNamedBone(bone_names[3]),
				RigidBodyForNamedBone(bone_names[4]),		// right leg
				RigidBodyForNamedBone(bone_names[5]),
				RigidBodyForNamedBone(bone_names[6])
			};

			Quaternion rb_oris[7];
			Vec3 rb_rots[7];
			for(unsigned int i = 0; i < 7; ++i)
			{
				rb_oris[i] = rbs[i]->GetOrientation();
				rb_rots[i] = rbs[i]->GetAngularVelocity();
			}

			Quaternion joint_oris[6];
			Vec3 joint_rots[6];
			Vec3 joint_vals[6];
			Vec3 joint_disps[6];
			for(unsigned int i = 0; i < 6; ++i)
			{
				unsigned int a = i == 3 ? 0 : i;
				unsigned int b = i + 1;

				SoldierFoot* foot = i > 3 ? rfoot : lfoot;

				Quaternion inv_parent = Quaternion::Reverse(rb_oris[a]);
				joint_oris[i] = inv_parent * rb_oris[b];
				joint_rots[i] = inv_parent * (rb_rots[b] - rb_rots[a]);
				
				joint_vals[i] = foot->joint_protos[i % 3]->axes * joint_oris[i].ToRVec();

				joint_disps[i] = inv_parent * (rbs[b]->GetTransformationMatrix() - rbs[a]->GetTransformationMatrix()).TransformVec3_1(foot->joint_protos[i % 3]->pos);
			}

			// orient right foot to make its down vector match the surface's normal vector
			Quaternion foot_ori = rb_oris[6];
			static const Vec3 surface_normal = Vec3(0, 1, 0);
			static const Vec3 dirs[2] = { Vec3(0, 0, 1), Vec3(1, 0, 0) };
			for(unsigned int i = 0; i < 2; ++i)
			{
				Vec3 dir   = foot_ori * dirs[i];
				Vec3 level = dir - surface_normal * Vec3::Dot(dir, surface_normal);
				Vec3 cross = Vec3::Cross(dir, level);

				float level_mag = level.ComputeMagnitude();
				float cross_mag = cross.ComputeMagnitude();

				if(level_mag != 0 && cross_mag != 0 && fabs(cross_mag) <= fabs(level_mag))
					foot_ori = Quaternion::FromRVec(cross * (asinf(cross_mag / level_mag) / cross_mag)) * foot_ori;
			}

			Vec3 com, avgvel;
			float total_mass = 0.0f;
			for(set<RigidBody*>::iterator iter = velocity_change_bodies.begin(); iter != velocity_change_bodies.end(); ++iter)
			{
				RigidBody* rb = *iter;
				float mass = rb->GetMass();
				com    += rb->GetCenterOfMass()   * mass;
				avgvel += rb->GetLinearVelocity() * mass;
				total_mass += mass;
			}
			com    /= total_mass;
			avgvel /= total_mass;

			static const unsigned int num_inputs = 0, num_outputs = 20, num_coeffs = (num_inputs + 1) * num_outputs;
//			float outputs[num_outputs];

			// apply values output by magic black box
			float illegal_ori_penalty = 0.0f;
			float inv_timestep = 1.0f / time.elapsed;
			for(unsigned int i = 0; i < 6; ++i)
			{
				SoldierFoot* foot = i < 3 ? lfoot : rfoot;
				const ModelPhysics::JointPhysics* jp = foot->joint_protos[i % 3];

				Vec3 outputs_vec = Vec3(magic_box_coeffs[i * 3 + 2], magic_box_coeffs[i * 3 + 3], magic_box_coeffs[i * 3 + 4]);
				//Vec3 rot = joint_rots[i] + outputs_vec;
				
				Vec3 lcoords     = outputs_vec;
				//Vec3 lcoords     = jp->axes * bones[i + 4]->ori.ToRVec();
				//Vec3 lcoords     = joint_vals[i] + rot * time.elapsed;
				//Vec3 new_lcoords = jp->GetClampedAngles(outputs_vec);
				Vec3 new_lcoords = jp->GetClampedAngles(lcoords);

				bones[i + 1]->ori = Quaternion::FromRVec(jp->axes.TransposedMultiply(new_lcoords));
				//foot->joint_rots[i] = (new_lcoords - lcoords) * inv_timestep;

				illegal_ori_penalty += (lcoords - new_lcoords).ComputeMagnitudeSquared();
			}

			// magic black box; curly braces for scope
			{
				Vec3 lfoot_lcom = rbs[3]->GetMassInfo().com;
				Vec3 rfoot_lcom = rbs[6]->GetMassInfo().com;
				Mat4 foot_xform = rbs[6]->GetTransformationMatrix();

				Vec3 pelvis_pos = rbs[0]->GetCenterOfMass();

				Mat4 lfoot_xform = rbs[3]->GetTransformationMatrix();
				float lya = lfoot_xform.TransformVec3_1( 0.23f, 0, -0.10f).y;
				float lyb = lfoot_xform.TransformVec3_1( 0.27f, 0,  0.26f).y;
				float lyc = lfoot_xform.TransformVec3_1( 0.21f, 0,  0.26f).y;
				float lfoot_badness = Vec3::MagnitudeSquared(lya, lyb, lyc);

				Mat4 rfoot_xform = rbs[6]->GetTransformationMatrix();
				float rya = rfoot_xform.TransformVec3_1(-0.23f, 0, -0.10f).y;
				float ryb = rfoot_xform.TransformVec3_1(-0.27f, 0,  0.26f).y;
				float ryc = rfoot_xform.TransformVec3_1(-0.21f, 0,  0.26f).y;
				float rfoot_badness = Vec3::MagnitudeSquared(rya, ryb, ryc);

				float speed_penalty_coeff = min(1.0f, lifetime * 8.0f);
				float pelvis_speed  = rbs[0]->GetLinearVelocity().ComputeMagnitudeSquared();
				float pelvis_aspeed = rbs[0]->GetLinearVelocity().ComputeMagnitudeSquared();
				float avg_speed     = speed_penalty_coeff * avgvel.ComputeMagnitudeSquared();
				float poffy         = pelvis_pos.y - 0.85f;
				float poffysq       = poffy * poffy;

				if(illegal_ori_penalty > 0 || lfoot_badness > 0.05f || rfoot_badness > 0.05f || poffysq > 0.05f || avg_speed > 0.1f || lifetime > 4.0f)
				{
					if(!test_done)
					{
						//Debug(((stringstream&)(stringstream() << "t = " << lifetime << "; foot badness = " << foot_badness << "; poffy = " << poffy << "; avg speed = " << avg_speed << "; lfoot standing = " << lfoot->standing << endl)).str());

						test_done = true;
						if(lifetime == 0 || total_score_weight == 0)
							magic_box_score = 9001;
						else
							magic_box_score = sqrtf(magic_box_score / total_score_weight) / lifetime;
					}
				}
				else
				{
					float weight = max(0.0f, min(1.0f, (lifetime - 0.1f) * 10.0f));

					magic_box_score    += weight * ((lfoot_badness + rfoot_badness) * 1000000 + pelvis_speed + pelvis_aspeed + avg_speed * 400 + poffysq);
					total_score_weight += weight;
				}
			}
		}

		// only update the gun pose for DoInitialPose		
		if(lifetime == 0)					// TODO: get rid of the conditional once the balancing ability is fairly robust
		{
			p_ag->yaw = yaw;
			p_ag->pitch = pitch;
			p_ag->pelvis_ori = bones[0]->ori;
			p_ag->UpdatePose(time);

			for(boost::unordered_map<unsigned int, BoneInfluence>::iterator iter = p_ag->bones.begin(); iter != p_ag->bones.end(); ++iter)
				posey->skeleton->GetNamedBone(iter->first)->ori = Quaternion::FromRVec(iter->second.ori);
		}
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
}
