#include "StdAfx.h"
#include "Soldier.h"

#include "PoseAimingGun.h"
#include "IKAnimationPose.h"

#include "DoodOrientationConstraint.h"

#include "WeaponEquip.h"

#include "ConverterWhiz.h"

#include "../CibraryEngine/NeuralNet.h"

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




	/*
	 * Soldier methods
	 */
	Soldier::Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		gun_hand_bone(NULL),
		p_ag(NULL),
		ik_pose(NULL),
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
	}

	void Soldier::InnerDispose() { Dood::InnerDispose(); }

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

						float total_mass = 0.0f;

						for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
							total_mass += (*iter)->GetMassInfo().mass;

						Vec3 apply_force = jump_accel_vec * total_mass;
						
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

		if (can_recharge)
			jump_fuel = min(jump_fuel + jump_fuel_refill_rate * timestep, 1.0f);
	}

	void Soldier::DoMovementControls(TimingInfo time, Vec3 forward, Vec3 rightward) { Dood::DoMovementControls(time, forward, rightward); }

	void Soldier::DoWeaponControls(TimingInfo time)
	{
		p_ag->yaw = yaw;
		p_ag->pitch = pitch;

		Dood::DoWeaponControls(time);
	}

	void Soldier::PreUpdatePoses(TimingInfo time) { }

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

	void Soldier::Spawned()
	{
		Dood::Spawned();

		set<RigidBody*> joint_bodies;
		joint_bodies.insert(RigidBodyForNamedBone("pelvis"));
		joint_bodies.insert(RigidBodyForNamedBone("l leg 1"));
		joint_bodies.insert(RigidBodyForNamedBone("l leg 2"));
		joint_bodies.insert(RigidBodyForNamedBone("l foot"));
		joint_bodies.insert(RigidBodyForNamedBone("r leg 1"));
		joint_bodies.insert(RigidBodyForNamedBone("r leg 2"));
		joint_bodies.insert(RigidBodyForNamedBone("r foot"));


		vector<JointConstraint*> use_joints;
		vector<JointConstraint*> all_joints;
		for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
			if(JointConstraint* jc = dynamic_cast<JointConstraint*>(*iter))
			{
				all_joints.push_back(jc);
				//jc->enable_motor = false;

				if(joint_bodies.find(jc->obj_a) != joint_bodies.end() || joint_bodies.find(jc->obj_b) != joint_bodies.end())
					use_joints.push_back(jc);
			}

		ik_pose = new IKWalkPose(physics, rigid_bodies, all_joints, use_joints, rbody_to_posey);
		ik_pose->AddEndEffector(RigidBodyForNamedBone("l foot"));
		ik_pose->AddEndEffector(RigidBodyForNamedBone("r foot"));
		posey->active_poses.push_back(ik_pose);
	}

	void Soldier::Die(Damage cause)
	{
		Dood::Die(cause);

		for(list<Pose*>::iterator iter = posey->active_poses.begin(); iter != posey->active_poses.end(); ++iter)
			if(*iter == ik_pose)
			{
				posey->active_poses.erase(iter);

				delete ik_pose;
				ik_pose = NULL;

				break;
			}
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
