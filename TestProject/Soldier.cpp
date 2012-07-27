#include "StdAfx.h"
#include "Soldier.h"

#include "PoseAimingGun.h"
#include "IKAnimationPose.h"

#include "DoodOrientationConstraint.h"

#include "WeaponEquip.h"

#include "ConverterWhiz.h"
#include "TestGame.h"

namespace Test
{
	/*
	 * Soldier constants
	 */ 
	float jump_pack_accel = 15.0f;

	float jump_to_fly_delay = 0.3f;
	float jump_speed = 4.0f;

	float jump_fuel_spend_rate = 0.5f, jump_fuel_refill_rate = 0.4f;
	float flying_accel = 8.0f;



	
	IKAnimation* MakeTurnLeft()
	{
		IKAnimation* result = new IKAnimation();

		result->bones.push_back("pelvis");
		result->bones.push_back("l foot");
		result->bones.push_back("r foot");

		result->chains.push_back(IKAnimation::Chain(0, 1));
		result->chains.push_back(IKAnimation::Chain(0, 2));

		float frame_duration = 0.6f;

		{	// frame 0
			IKAnimation::Keyframe kf = IKAnimation::Keyframe(frame_duration, 1);
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(0, Vec3(0.05f,	0.1f,	-0.1f),		Quaternion::FromPYR(0,		-0.5f,		0),		1.0f));
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(1, Vec3(-0.1f,	0.25f,	0.1f),		Quaternion::FromPYR(0,		-0.5f,		0.25f),	0.0f));
			result->frames.push_back(kf);
		}

		{	// frame 1
			IKAnimation::Keyframe kf = IKAnimation::Keyframe(frame_duration, 2);
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(0, Vec3(0.05f,	0.1f,	-0.2f),		Quaternion::FromPYR(0,		-1.0f,		0),	0.5f));
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(1, Vec3(-0.2f,	0.1f,	0.15f),		Quaternion::FromPYR(0,		-1.0f,		0),	0.5f));
			result->frames.push_back(kf);
		}

		{	// frame 2
			IKAnimation::Keyframe kf = IKAnimation::Keyframe(frame_duration, -1);
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(0, Vec3(),							Quaternion::Identity(),						0.5f));
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(1, Vec3(),							Quaternion::Identity(),						0.5f));
			result->frames.push_back(kf);
		}



		/*
		{	// frame 0
			IKAnimation::Keyframe kf = IKAnimation::Keyframe(frame_duration, 1);
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(0, Vec3(0.05f,	0.25f,	-0.1f),		Quaternion::FromPYR(0,		-0.5f,		0),	0.0f));
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(1, Vec3(0,		0.05f,	0.05f),		Quaternion::FromPYR(0,		0.5f,		0),	1.0f));
			result->frames.push_back(kf);
		}

		{	// frame 1
			IKAnimation::Keyframe kf = IKAnimation::Keyframe(frame_duration, 2);
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(0, Vec3(0.1f,	0,		-0.2f),		Quaternion::FromPYR(0,		-1,			0),	0.5f));
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(1, Vec3(-0.1f,	0,		0.05f),		Quaternion::FromPYR(0,		1,			0),	0.5f));
			result->frames.push_back(kf);
		}

		{	// frame 2
			IKAnimation::Keyframe kf = IKAnimation::Keyframe(frame_duration, 3);
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(0, Vec3(0.05f,	0.05f,	-0.05f),	Quaternion::FromPYR(0,		-0.5f,		0),	1.0f));
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(1, Vec3(-0.05f,	0.25f,	0.05f),		Quaternion::FromPYR(0,		0.5f,		0),	0.0f));
			result->frames.push_back(kf);
		}

		{	// frame 3
			IKAnimation::Keyframe kf = IKAnimation::Keyframe(frame_duration, -1);
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(0, Vec3(),							Quaternion::Identity(),						0.5f));
			kf.chain_states.push_back(IKAnimation::Keyframe::ChainState(1, Vec3(),							Quaternion::Identity(),						0.5f));
			result->frames.push_back(kf);
		}
		*/

		return result;
	}

	IKAnimation* MakeTurnRight()
	{
		IKAnimation* result = new IKAnimation();

		// TODO: implement this

		return result;
	}




	/*
	 * Soldier methods
	 */
	Soldier::Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		gun_hand_bone(NULL),
		p_ag(NULL),
		ik_pose(NULL),
		turn_left(NULL),
		turn_right(NULL),
		jump_fuel(1.0f),
		jet_start_sound(NULL),
		jet_loop_sound(NULL),
		jet_loop(NULL)
	{
		p_ag = new PoseAimingGun();
		posey->active_poses.push_back(p_ag);

		gun_hand_bone = character->skeleton->GetNamedBone("r grip");

		Cache<SoundBuffer>* sound_cache = game_state->content->GetCache<SoundBuffer>();
		jet_start_sound = sound_cache->Load("jet_start");
		jet_loop_sound = sound_cache->Load("jet_loop");

		turn_left = MakeTurnLeft();
		turn_right = MakeTurnRight();
	}

	void Soldier::InnerDispose()
	{
		delete turn_left;
		delete turn_right;

		Dood::InnerDispose();
	}

	void Soldier::DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward)
	{
		float timestep = time.elapsed;

		bool can_recharge = true;
		bool jetted = false;
		if (control_state->GetBoolControl("jump"))
		{
			if (standing > 0)
			{
				//jump off the ground
				for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
					(*iter)->ApplyCentralImpulse(Vec3(0, jump_speed * (*iter)->GetMassInfo().mass, 0));
				jump_start_timer = time.total + jump_to_fly_delay;
			}
			else
			{
				can_recharge = false;

				if (jump_fuel > 0)
				{
					// jetpacking
					if (time.total > jump_start_timer)
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

						jump_fuel -= timestep * (jump_fuel_spend_rate);

						Vec3 jump_accel_vec = Vec3(0, jump_pack_accel, 0);
						Vec3 lateral_accel = forward * max(-1.0f, min(1.0f, control_state->GetFloatControl("forward"))) + rightward * max(-1.0f, min(1.0f, control_state->GetFloatControl("sidestep")));
						jump_accel_vec += lateral_accel * (flying_accel);

						for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
							(*iter)->ApplyCentralForce(jump_accel_vec * (*iter)->GetMassInfo().mass);
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

		if (can_recharge)
			jump_fuel = min(jump_fuel + jump_fuel_refill_rate * timestep, 1.0f);
	}

	void Soldier::DoMovementControls(TimingInfo time, Vec3 forward, Vec3 rightward)
	{
		Dood::DoMovementControls(time, forward, rightward);
	}

	void Soldier::DoWeaponControls(TimingInfo time)
	{
		p_ag->yaw = yaw;
		p_ag->pitch = pitch;

		Dood::DoWeaponControls(time);
	}

	void Soldier::PreUpdatePoses(TimingInfo time)
	{
		return;

		// turning in place
		Vec3 yaw_fwd = Vec3(-sinf(yaw), 0, cosf(yaw));
		Vec3 yaw_left = Vec3(yaw_fwd.z, 0, -yaw_fwd.x);

		Mat4 pelvis_xform = Mat4::Translation(pos) * character->skeleton->GetNamedBone("pelvis")->GetTransformationMatrix();

		Vec3 pelvis_fwd = pelvis_xform.TransformVec3_0(0, 0, 1);
		Vec3 pelvis_left = pelvis_xform.TransformVec3_0(1, 0, 0);
		pelvis_fwd.y = 0;
		pelvis_left.y = 0;
		pelvis_fwd /= pelvis_fwd.ComputeMagnitude();				// HEADS UP! this is unstable when the forward vector is nearly vertical
		pelvis_left /= pelvis_left.ComputeMagnitude();


		float fwd_dot = Vec3::Dot(yaw_fwd, pelvis_fwd);
		float side_dot = Vec3::Dot(yaw_left, pelvis_fwd);

		float angle = asinf(side_dot);
		((TestGame*)game_state)->debug_text = ((stringstream&)(stringstream() << "angle = " << angle)).str();

		const float max_torso_twist = 1.0f;
		const float step_duration = 0.5f;
		const float between_steps = 0.0f;

		float now = time.total, finish = time.total + step_duration;

		if(ik_pose == NULL || ik_pose->dead)
		{
			if(ik_pose)
			{
				ik_pose->SetActive(false);			// this will ensure it gets deleted
				ik_pose = NULL;
			}

			//if(time.total > 0.5f)
			//	ik_pose = new IKAnimationPose(turn_left, this);

			if(angle < -max_torso_twist && turn_left)
				ik_pose = new IKAnimationPose(turn_left, this);
			//else if(angle > max_torso_twist && turn_right)
			//	ik_pose = new IKAnimationPose(turn_right, this);

			if(ik_pose)
				posey->active_poses.push_back(ik_pose);
		}

		//p_ag->yaw = angle;
	}

	void Soldier::PostUpdatePoses(TimingInfo time)
	{
		if(equipped_weapon != NULL && gun_hand_bone != NULL)
		{
			equipped_weapon->gun_xform = Mat4::Translation(pos) * gun_hand_bone->GetTransformationMatrix() * Mat4::Translation(gun_hand_bone->rest_pos);
			equipped_weapon->sound_pos = equipped_weapon->pos = equipped_weapon->gun_xform.TransformVec3_1(0, 0, 0);
			equipped_weapon->sound_vel = equipped_weapon->vel = vel;
		}
	}

	void Soldier::Spawned()
	{
		Dood::Spawned();

		unsigned int torso_1 = Bone::string_table["torso 1"];
		unsigned int torso_2 = Bone::string_table["torso 2"];

		RigidBody* body_1 = NULL;
		RigidBody* body_2 = NULL;

		for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
		{
			if(rbody_to_posey[i]->name == torso_1)
				body_1 = rigid_bodies[i];
			else if(rbody_to_posey[i]->name == torso_2)
				body_2 = rigid_bodies[i];

			if(body_1 && body_2)
			{
				for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
				{
					if(JointConstraint* jc = dynamic_cast<JointConstraint*>(*iter))
					{
						if(jc->obj_a == body_1 && jc->obj_b == body_2 || jc->obj_a == body_2 && jc->obj_b == body_1)
						{
							jc->orient_absolute = true;
							break;
						}
					}
				}

				break;
			}
		}
	}

	void Soldier::DeSpawned()
	{
		Dood::DeSpawned();
	}




	void Soldier::GetBoneEntries(vector<BoneEntry>& bone_entries)
	{
		bone_entries.push_back(BoneEntry("pelvis",		"",				Vec3(	0,		1.12f,	-0.09f	)));
		bone_entries.push_back(BoneEntry("torso 1",		"pelvis",		Vec3(	0,		1.34f,	-0.2f	)));
		bone_entries.push_back(BoneEntry("torso 2",		"torso 1",		Vec3(	0,		1.57f,	-0.2f	)));
		bone_entries.push_back(BoneEntry("head",		"torso 2",		Vec3(	0,		1.73f,	0.0f	)));

		bone_entries.push_back(BoneEntry("l shoulder",	"torso 2",		Vec3(	0.27f,	1.69f,	0.0f	)));
		bone_entries.push_back(BoneEntry("l arm 1",		"l shoulder",	Vec3(	0.28f,	1.52f,	-0.05f	)));
		bone_entries.push_back(BoneEntry("l arm 2",		"l arm 1",		Vec3(	0.53f,	1.4f,	-0.06f	)));
		bone_entries.push_back(BoneEntry("l hand",		"l arm 2",		Vec3(	0.82f,	1.25f,	0.01f	)));
		bone_entries.push_back(BoneEntry("l leg 1",		"pelvis",		Vec3(	0.15f,	1.04f,	-0.02f	)));
		bone_entries.push_back(BoneEntry("l leg 2",		"l leg 1",		Vec3(	0.19f,	0.64f,	0.01f	)));
		bone_entries.push_back(BoneEntry("l foot",		"l leg 2",		Vec3(	0.27f,	0.14f,	-0.11f	)));

		bone_entries.push_back(BoneEntry("r shoulder",	"torso 2",		Vec3(	-0.27f,	1.69f,	0.0f	)));
		bone_entries.push_back(BoneEntry("r arm 1",		"r shoulder",	Vec3(	-0.28f,	1.52f,	-0.05f	)));
		bone_entries.push_back(BoneEntry("r arm 2",		"r arm 1",		Vec3(	-0.53f,	1.4f,	-0.06f	)));
		bone_entries.push_back(BoneEntry("r hand",		"r arm 2",		Vec3(	-0.82f,	1.25f,	0.01f	)));
		bone_entries.push_back(BoneEntry("r leg 1",		"pelvis",		Vec3(	-0.15f,	1.04f,	-0.02f	)));
		bone_entries.push_back(BoneEntry("r leg 2",		"r leg 1",		Vec3(	-0.19f,	0.64f,	0.01f	)));
		bone_entries.push_back(BoneEntry("r foot",		"r leg 2",		Vec3(	-0.27f,	0.14f,	-0.11f	)));

		for(vector<BoneEntry>::iterator iter = bone_entries.begin(); iter != bone_entries.end(); ++iter)
			iter->mass = 98.0f / bone_entries.size();
	}
}
