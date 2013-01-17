#include "StdAfx.h"
#include "CrabBug.h"

#include "WalkPose.h"

#include "Limb.h"

#include "ConverterWhiz.h"

#define DIE_AFTER_ONE_SECOND 0

namespace Test
{
	using namespace std;

	/*
	 * CrabBug constants
	 */
	float bug_leap_duration = 0.5f;
	float bug_leap_speed = 8.0f;




	static void GenerateHardCodedWalkAnimation(KeyframeAnimation* ka)
	{
		{
			Keyframe kf(0.5f);
			kf.next = 1;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	1,	0.3f,	0	), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	0,	-1,		0	), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	1,	0.5f,	0	), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	0,	0.3f,	0	), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	1,	-1,		0	), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	0,	0.5f,	0	), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 2;
			
			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	0,	0.3f,	0	), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	1,	-1,		0	), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	0,	0.5f,	0	), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	1,	0.3f,	0	), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	0,	-1,		0	), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	1,	0.5f,	0	), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	0,	-0.3f,	0	), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	1,	1,		0	), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	0,	-0.5f,	0	), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	1,	-0.3f,	0	), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	0,	1,		0	), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	1,	-0.5f,	0	), Vec3());
			
			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3(	1,	-0.3f,	0	), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3(	0,	1,		0	), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3(	1,	-0.5f,	0	), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3(	0,	-0.3f,	0	), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3(	1,	1,		0	), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3(	0,	-0.5f,	0	), Vec3());
			
			ka->frames.push_back(kf);
		}
	}

	static void GenerateRestPose(KeyframeAnimation* ka)
	{
		ka->frames.clear();
		ka->name = "crabby rest pose";

		Keyframe kf;
		kf.duration = 2.0f;
		kf.next = 0;
		kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence();
		kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence();
		kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence();
		kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence();
		kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence();
		kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence();

		ka->frames.push_back(kf);
	}




	/*
	 * CrabBug methods
	 */
	CrabBug::CrabBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		crab_heading(new CrabHeading()),
		limbs()
	{
		hp *= 0.5f;

		// character animation stuff
		posey->active_poses.push_back(crab_heading);

		KeyframeAnimation kw, kr;
		GenerateHardCodedWalkAnimation(&kw);
		GenerateRestPose(&kr);

		posey->active_poses.push_back(new WalkPose(this, &kr, &kw, &kw, &kw, &kw, NULL, NULL));

		foot_bones[Bone::string_table["l leg a 3"]] = NULL;
		foot_bones[Bone::string_table["r leg a 3"]] = NULL;
		foot_bones[Bone::string_table["l leg b 3"]] = NULL;
		foot_bones[Bone::string_table["r leg b 3"]] = NULL;
		foot_bones[Bone::string_table["l leg c 3"]] = NULL;
		foot_bones[Bone::string_table["r leg c 3"]] = NULL;
	}

	void CrabBug::InnerDispose()
	{
		Dood::InnerDispose();

		for(vector<Limb*>::iterator iter = limbs.begin(); iter != limbs.end(); ++iter)
			delete *iter;
		limbs.clear();
	}

	void CrabBug::DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward)
	{
		if(standing_callback.standing > 0 && control_state->GetBoolControl("leap") && time.total > jump_start_timer)
		{
			// crab bug leaps forward
			float leap_angle = 0.4f;
			Vec3 leap_vector = (forward * (cosf(leap_angle)) + Vec3(0, sinf(leap_angle), 0));

			standing_callback.ApplyVelocityChange(leap_vector * bug_leap_speed);

			jump_start_timer = time.total + bug_leap_duration;
		}
	}

	void CrabBug::PreUpdatePoses(TimingInfo time) { crab_heading->yaw = yaw; }

	void CrabBug::Update(TimingInfo time)
	{
#if DIE_AFTER_ONE_SECOND
		if(time.total > 1.0f)
			Die(Damage());
#endif

		Dood::Update(time);
	}

	void CrabBug::PoseToPhysics(float timestep)
	{
		if(alive)
		{
			for(vector<Limb*>::iterator iter = limbs.begin(); iter != limbs.end(); ++iter)
				(*iter)->Update(timestep);
		}
		else
		{
			for(unsigned int i = 0; i < constraints.size(); ++i)
			{
				JointConstraint* jc = (JointConstraint*)constraints[i];

				jc->enable_motor = false;
				jc->motor_torque = Vec3();
			}
		}
	}

	void CrabBug::Spawned()
	{
		Dood::Spawned();

		JointConstraint* use_joints[3] = { NULL, NULL, NULL };
		RigidBody* use_rigid_bodies[4] = { NULL, NULL, NULL, NULL };

		const string limb_prefixes[] =
		{
			"l leg a ",
			"r leg a ",
			"l leg b ",
			"r leg b ",
			"l leg c ",
			"r leg c "
		};

		use_rigid_bodies[0] = RigidBodyForNamedBone("carapace");			// this value is shared for all six limbs

		unsigned int num_bones = character->skeleton->bones.size();
		unsigned int num_constraints = constraints.size();

		for(unsigned int i = 0; i < 6; ++i)
		{
			for(unsigned int j = 1; j <= 3; ++j)
			{
				unsigned int id = Bone::string_table[((stringstream&)(stringstream() << limb_prefixes[i] << j)).str()];

				use_rigid_bodies[j] = NULL;
				use_joints[j - 1] = NULL;

				for(unsigned int k = 0; k < num_bones; ++k)
					if(character->skeleton->bones[k]->name == id)
					{
						use_rigid_bodies[j] = bone_to_rbody[k];
						break;
					}
				assert(use_rigid_bodies[j] != NULL);
				
				for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
				{
					JointConstraint* jc = (JointConstraint*)*iter;
					if(jc->obj_a == use_rigid_bodies[j] && jc->obj_b == use_rigid_bodies[j - 1])
					{
						use_joints[j - 1] = jc;
						break;
					}
				}
				assert(use_joints[j - 1] != NULL);
			}

			Limb* limb = new Limb(use_joints, use_rigid_bodies, 3);
			limbs.push_back(limb);
		}
	}
}
