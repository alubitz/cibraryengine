#include "StdAfx.h"
#include "Soldier.h"

#include "WeaponEquip.h"

namespace Test
{
	/*
	 * Soldier constants
	 */
	float fly_accel_up = 15.0f;

	float jump_to_fly_delay = 0.3f;
	float jump_speed = 4.0f;

	float fuel_spend_rate = 0.5f, fuel_refill_rate = 0.4f;
	float flying_accel = 8.0f;




	/*
	 * Soldier::SoldierIKPose private implementation struct
	 */
	struct Soldier::SoldierIKPose : public Pose
	{
		Soldier* dood;

		struct Leg
		{
			RigidBody	*pelvis_rb,		*upper_rb,		*lower_rb,		*foot_rb;
			Bone		*pelvis_bone,	*upper_bone,	*lower_bone,	*foot_bone;

			JointConstraint		*hip_joint,		*knee_joint,	*ankle_joint;
			Dood::FootState		*foot_state;

			float upper_len, lower_len;
			float upper_sq, usqmlsq, max_hta;

			Leg(Dood* dood, unsigned int pelvis_id, unsigned int upper_id, unsigned int lower_id, unsigned int foot_id) :
				pelvis_rb(NULL),
				upper_rb(NULL),
				lower_rb(NULL),
				foot_rb(NULL),
				pelvis_bone(NULL),
				upper_bone(NULL),
				lower_bone(NULL),
				foot_bone(NULL),
				hip_joint(NULL),
				knee_joint(NULL),
				ankle_joint(NULL),
				foot_state(NULL),
				upper_len(0.0f),
				lower_len(0.0f),
				upper_sq(0.0f),
				usqmlsq(0.0f),
				max_hta(0.0f)
			{
				// find rigid bodies and posey bones
				for(unsigned int i = 0, num_bodies = dood->rigid_bodies.size(); i < num_bodies; ++i)
				{
					RigidBody* rb = dood->rigid_bodies[i];
					if(Bone* bone = dood->rbody_to_posey[i])
						if(bone->name == pelvis_id)
						{
							pelvis_rb = rb;
							pelvis_bone = bone;
						}
						else if(bone->name == upper_id)
						{
							upper_rb = rb;
							upper_bone = bone;
						}
						else if(bone->name == lower_id)
						{
							lower_rb = rb;
							lower_bone = bone;
						}
						else if(bone->name == foot_id)
						{
							foot_rb = rb;
							foot_bone = bone;
						}
				}

				// find joints
				for(vector<PhysicsConstraint*>::iterator iter = dood->constraints.begin(); iter != dood->constraints.end(); ++iter)
				{
					JointConstraint* jc = (JointConstraint*)(*iter);
					if(jc->obj_a == pelvis_rb && jc->obj_b == upper_rb)
						hip_joint = jc;
					else if(jc->obj_a == upper_rb && jc->obj_b == lower_rb)
						knee_joint = jc;
					else if(jc->obj_a == lower_rb && jc->obj_b == foot_rb)
						ankle_joint = jc;
				}

				// find foot states
				for(vector<Dood::FootState*>::iterator iter = dood->feet.begin(); iter != dood->feet.end(); ++iter)
					if((*iter)->posey_id == foot_id)
						foot_state = *iter;

				// compute bone lengths
				upper_len = (knee_joint->pos - hip_joint->pos).ComputeMagnitude();
				lower_len = (ankle_joint->pos - knee_joint->pos).ComputeMagnitude();

				upper_sq = upper_len * upper_len;
				usqmlsq = upper_sq - lower_len * lower_len;
				max_hta = upper_len + lower_len;
			}

			void PoseLeg(const Mat4& pelvis_xform, const TimingInfo& time)
			{
				Vec3 hip_pos = pelvis_xform.TransformVec3_1(hip_joint->pos);

				Vec3 cur_ankle_pos = foot_rb->GetTransformationMatrix().TransformVec3_1(ankle_joint->pos);
				Vec3 cur_knee_pos = upper_rb->GetTransformationMatrix().TransformVec3_1(knee_joint->pos);

				// TODO: figure out where we want the foot and ankle to be for real
				Vec3 ankle_pos = cur_ankle_pos;

				Vec3 hip_to_ankle = ankle_pos - hip_pos;
				float hta_dist = hip_to_ankle.ComputeMagnitude();
				if(hta_dist > max_hta)
				{
					hip_to_ankle *= max_hta / hta_dist;
					ankle_pos = hip_pos + hip_to_ankle;

					hta_dist = max_hta;
				}

				// figure out where the knee can possibly go...
				float inv_hta = 1.0f / hta_dist;
				float knee_dist = 0.5f * (usqmlsq + hta_dist * hta_dist) * inv_hta;
				float knee_radius = sqrtf(upper_sq - knee_dist * knee_dist);

				Vec3 u_hta = hip_to_ankle * inv_hta;

				// TODO: decide where we want the knee to go for real
				Vec3 knee_pos = cur_knee_pos + pelvis_xform.TransformVec3_0(0, 0, 1);
				// flatten the desired knee position onto the plane of the circle, and normalize it to the circle's radius
				knee_pos -= u_hta * Vec3::Dot(u_hta, knee_pos);
				knee_pos = hip_pos + u_hta * knee_dist + Vec3::Normalize(knee_pos, knee_radius);

				// TODO: select orientations for the two leg bones
				// set posey bone orientations based on this
			}
		} left_leg, right_leg;

		RigidBody* pelvis;

		SoldierIKPose(Soldier* dood) :
			dood(dood),
			left_leg(dood,	Bone::string_table["pelvis"], Bone::string_table["l leg 1"], Bone::string_table["l leg 2"], Bone::string_table["l foot"]),
			right_leg(dood,	Bone::string_table["pelvis"], Bone::string_table["r leg 1"], Bone::string_table["r leg 2"], Bone::string_table["r foot"]),
			pelvis(dood->root_rigid_body)
		{
		}

		~SoldierIKPose() { }

		void UpdatePose(TimingInfo time)
		{
			// TODO: select desired xform for the pelvis
			Mat4 pelvis_xform = pelvis->GetTransformationMatrix();

			// pose each leg
			left_leg.PoseLeg(pelvis_xform, time);
			right_leg.PoseLeg(pelvis_xform, time);

			// TODO: pose arms, head, etc.
		}
	};




	/*
	 * Soldier methods
	 */
	Soldier::Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		ik_pose(NULL),
		gun_hand_bone(NULL),
		jet_fuel(1.0f),
		jet_start_sound(NULL),
		jet_loop_sound(NULL),
		jet_loop(NULL)
	{
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
			ik_pose = new SoldierIKPose(this);
			posey->active_poses.push_back(ik_pose);
		}
	}
}
