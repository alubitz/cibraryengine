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


	struct MyNeuralNet
	{
		struct Data
		{
			unsigned int num_bones;
			unsigned int num_end_effectors;
			unsigned int num_joints;

			NeuralNet ee_net;
			NeuralNet bodies_and_ee;
			NeuralNet hidden_layer_a;
			NeuralNet hidden_layer_b;

			Data(unsigned int num_bones, unsigned int num_end_effectors, unsigned int num_joints) : 
				num_bones(num_bones),
				num_end_effectors(num_end_effectors),
				num_joints(num_joints),
				ee_net			(1 + 8,												8),
				bodies_and_ee	(1 + num_end_effectors * 8 + num_bones * 13 + 4,	num_joints * 3),
				hidden_layer_a	(1 + num_joints * 3,								num_joints * 3),
				hidden_layer_b	(1 + num_joints * 3,								num_joints * 3)
			{
			}

			Data(const Data& other) :
				num_bones(other.num_bones),
				num_end_effectors(other.num_end_effectors),
				num_joints(other.num_joints),
				ee_net(other.ee_net),
				bodies_and_ee(other.bodies_and_ee),
				hidden_layer_a(other.hidden_layer_a),
				hidden_layer_b(other.hidden_layer_b)
			{

			}

			void Mutate(int count)
			{
				// each float across all 4 NNs has the same probability of being mutated
				unsigned int ee = ee_net.num_inputs * ee_net.num_outputs;
				unsigned int bodies = ee + bodies_and_ee.num_inputs * bodies_and_ee.num_outputs;
				unsigned int ha = bodies + hidden_layer_a.num_inputs * hidden_layer_a.num_outputs;
				unsigned int hb = ha + hidden_layer_a.num_inputs * hidden_layer_a.num_outputs;

				if(count < 0)
					count = Random3D::RandInt(1, 5);

				for(int i = 0; i < count; ++i)
				{
					unsigned int muta = Random3D::RandInt(hb);

					if(muta < ee)
						ee_net.matrix[muta]						+= Random3D::Rand(-0.1f, 0.1f);
					else if(muta < bodies)
						bodies_and_ee.matrix[muta - ee]			+= Random3D::Rand(-0.1f, 0.1f);
					else if(muta < ha)
						hidden_layer_a.matrix[muta - bodies]	+= Random3D::Rand(-0.1f, 0.1f);
					else
						hidden_layer_b.matrix[muta - ha]		+= Random3D::Rand(-0.1f, 0.1f);
				}
			}
		};
		Data* data;

		vector<RigidBody*> rigid_bodies;

		struct EndEffector
		{
			Vec3 desired_pos;
			Quaternion desired_ori;
			float arrival_time;

			EndEffector() : desired_pos(), desired_ori(Quaternion::Identity()), arrival_time(-1) { }
		};
		vector<EndEffector> ee_goals;

		float arrival_time;
		Vec3 carry_vel;

		MyNeuralNet(unsigned int num_bones, unsigned int num_end_effectors, unsigned int num_joints, const vector<RigidBody*>& rigid_bodies) :
			data(new Data(num_bones, num_end_effectors, num_joints)),
			rigid_bodies(rigid_bodies),
			ee_goals(),
			arrival_time(-1)
		{
			for(unsigned int i = 0; i < num_end_effectors; ++i)
				ee_goals.push_back(EndEffector());
		}

		void Solve(float now, const Vec3& pos_ref, float* outputs)
		{
			float* bodies_and_ee_inputs = new float[data->bodies_and_ee.num_inputs];
			bodies_and_ee_inputs[0] = 1.0f;

			float* ee_inputs = new float[data->ee_net.num_inputs];

			unsigned int i = 0;
			for(vector<EndEffector>::iterator iter = ee_goals.begin(); iter != ee_goals.end() && i < data->num_end_effectors; ++iter, ++i)
			{
				float* input_ptr = ee_inputs;

				*(input_ptr++) = 1.0f;
				*(input_ptr++) = iter->arrival_time - now;

				Vec3 desired_pos = iter->desired_pos - pos_ref;
				*(input_ptr++) = desired_pos.x;
				*(input_ptr++) = desired_pos.y;
				*(input_ptr++) = desired_pos.z;

				*(input_ptr++) = iter->desired_ori.w;
				*(input_ptr++) = iter->desired_ori.x;
				*(input_ptr++) = iter->desired_ori.y;
				*(input_ptr++) = iter->desired_ori.z;

				data->ee_net.Multiply(ee_inputs, &bodies_and_ee_inputs[1 + 8 * i]);
			}

			delete[] ee_inputs;

			float* bodies_ptr = &bodies_and_ee_inputs[1 + 8 * data->num_end_effectors];
			i = 0;
			for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end() && i < data->num_bones; ++iter, ++i)
			{
				RigidBody* body = *iter;

				Vec3 pos = body->GetPosition() - pos_ref;
				*(bodies_ptr++) = pos.x;
				*(bodies_ptr++) = pos.y;
				*(bodies_ptr++) = pos.z;

				Vec3 vel = body->GetLinearVelocity();
				*(bodies_ptr++) = vel.x;
				*(bodies_ptr++) = vel.y;
				*(bodies_ptr++) = vel.z;

				Quaternion ori = body->GetOrientation();
				*(bodies_ptr++) = ori.w;
				*(bodies_ptr++) = ori.x;
				*(bodies_ptr++) = ori.y;
				*(bodies_ptr++) = ori.z;

				Vec3 rot = body->GetAngularVelocity();
				*(bodies_ptr++) = rot.x;
				*(bodies_ptr++) = rot.y;
				*(bodies_ptr++) = rot.z;
			}

			*(bodies_ptr++) = arrival_time - now;
			*(bodies_ptr++) = carry_vel.x;
			*(bodies_ptr++) = carry_vel.y;
			*(bodies_ptr++) = carry_vel.z;

			float* hidden_layer_inputs = new float[data->hidden_layer_a.num_inputs];
			hidden_layer_inputs[0] = 1.0f;

			data->bodies_and_ee.Multiply(bodies_and_ee_inputs, &hidden_layer_inputs[1]);

			delete[] bodies_and_ee_inputs;

			data->hidden_layer_a.Multiply(hidden_layer_inputs, &hidden_layer_inputs[1]);
			data->hidden_layer_b.Multiply(hidden_layer_inputs, outputs);

			delete[] hidden_layer_inputs;
		}
	};

	MyNeuralNet* my_neural_net = NULL;

	vector<MyNeuralNet::Data*> gene_pool;


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

		ik_pose = new IKWalkPose(posey->skeleton, character->skeleton, Bone::string_table["pelvis"], mphys);
		ik_pose->AddEndEffector(Bone::string_table["l foot"]);
		ik_pose->AddEndEffector(Bone::string_table["r foot"]);
		posey->active_poses.push_back(ik_pose);

		gun_hand_bone = character->skeleton->GetNamedBone("r grip");

		Cache<SoundBuffer>* sound_cache = game_state->content->GetCache<SoundBuffer>();
		jet_start_sound = sound_cache->Load("jet_start");
		jet_loop_sound = sound_cache->Load("jet_loop");
	}

	void Soldier::InnerDispose()
	{
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
	}

	void Soldier::PostUpdatePoses(TimingInfo time)
	{
		// if he's ever upside-down, he fails
		Bone* head = character->skeleton->GetNamedBone("head");
		Vec3 head_pos = head->GetTransformationMatrix().TransformVec3_1(head->rest_pos);

		Vec3 actual_com;
		float mass = 0.0f;

		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
		{
			RigidBody* body = *iter;
			MassInfo mass_info = body->GetTransformedMassInfo();

			mass += mass_info.mass;
			actual_com += mass_info.com * mass_info.mass;
		}

		actual_com /= mass;

		if(head_pos.y < actual_com.y + 0.15f)
		{
			delete my_neural_net;
			my_neural_net = NULL;
		}


		// if neural net hasn't been disqualified, have it process inputs and produce outputs
		if(my_neural_net != NULL)
		{
			float* outputs = new float[rigid_bodies.size() * 3];

			my_neural_net->Solve(time.total, pos, outputs);

			float* ptr = outputs;
			for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
			{
				float x = *(ptr++), y = *(ptr++), z = *(ptr++);

				if(JointConstraint* jc = dynamic_cast<JointConstraint*>(*iter))
					jc->SetDesiredOrientation(Quaternion::FromPYR(x, y, z));
			}

			delete[] outputs;
		}

		// position and orient the gun; DON'T DELETE THIS!
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



		my_neural_net = new MyNeuralNet(rigid_bodies.size(), 2, constraints.size(), rigid_bodies);
		my_neural_net->ee_goals[0].desired_ori = Quaternion::FromPYR(0, 1, 0);
		my_neural_net->ee_goals[1].desired_ori = Quaternion::FromPYR(0, 1, 0);
		if(!gene_pool.empty())
		{
			delete my_neural_net->data;

			// select a species randomly from the gene pool
			my_neural_net->data = new MyNeuralNet::Data(*gene_pool[Random3D::RandInt(gene_pool.size())]);
		}
		my_neural_net->data->Mutate(1000);
	}

	void Soldier::DeSpawned()
	{
		if(my_neural_net != NULL)
		{
			// find how badly this neural net scored
			float fail = 0.0f;

			Vec3 lfoot, rfoot;
			
			for(unsigned int i = 0; i < character->skeleton->bones.size(); ++i)
				if(character->skeleton->bones[i]->name == Bone::string_table["l foot"])
				{
					RigidBody* rbody = bone_to_rbody[i];
					lfoot = rbody->GetPosition();
					
					fail += (Quaternion::Reverse(my_neural_net->ee_goals[0].desired_ori) * rbody->GetOrientation()).NormSquared();
					fail += (lfoot - my_neural_net->ee_goals[0].desired_pos).ComputeMagnitudeSquared();
				}
				else if(character->skeleton->bones[i]->name == Bone::string_table["r foot"])
				{
					RigidBody* rbody = bone_to_rbody[i];
					rfoot = rbody->GetPosition();
					
					fail += (Quaternion::Reverse(my_neural_net->ee_goals[1].desired_ori) * rbody->GetOrientation()).NormSquared();
					fail += (rfoot - my_neural_net->ee_goals[1].desired_pos).ComputeMagnitudeSquared();
				}

			Vec3 desired_com = (lfoot + rfoot) * 0.5f;
			desired_com.y += 1.5f;

			Vec3 actual_com, actual_vel;
			float mass = 0.0f;

			for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			{
				RigidBody* body = *iter;
				MassInfo mass_info = body->GetTransformedMassInfo();

				mass += mass_info.mass;
				actual_com += mass_info.com * mass_info.mass;
				actual_vel += body->GetLinearVelocity() * mass_info.mass;
			}

			actual_com /= mass;
			actual_vel /= mass;

			float com_fail = (actual_com - desired_com).ComputeMagnitudeSquared();
			fail += com_fail * com_fail;

			fail += (actual_vel - my_neural_net->carry_vel).ComputeMagnitudeSquared();



			// maybe add a few instances of this species to the gene pool
			float score = min(10.0f, 5.0f / (fail + 0.02f));

			int n = (int)score;
			for(int i = 0; i < n; ++i)
			{
				MyNeuralNet::Data* data = new MyNeuralNet::Data(*my_neural_net->data);
				gene_pool.push_back(data);
			}

			vector<MyNeuralNet::Data*> nu_gene_pool;
			while(gene_pool.size() > 200)
			{
				unsigned int remove = Random3D::RandInt(gene_pool.size());
				nu_gene_pool.clear();
				for(unsigned int i = 0; i < gene_pool.size(); ++i)
					if(i != remove)
						nu_gene_pool.push_back(gene_pool[i]);
					else
						delete gene_pool[i];
				gene_pool.assign(nu_gene_pool.begin(), nu_gene_pool.end());
			}

			delete my_neural_net->data;
			delete my_neural_net;
			my_neural_net = NULL;
		}

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
