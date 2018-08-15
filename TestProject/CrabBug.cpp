#include "StdAfx.h"
#include "CrabBug.h"

#include "WalkPose.h"

#include "CBone.h"
#include "CJoint.h"

#include "GAExperiment.h"

#define DIE_AFTER_ONE_SECOND			0

#define ENABLE_WALK_ANIMATIONS			0

#define DEBUG_GRADIENT_SEARCH_PROGRESS	0

#define MAX_TICK_AGE					30

namespace Test
{
	using namespace std;

	/*
	 * CrabBug constants
	 */
	static const float bug_leap_duration  = 0.5f;
	static const float bug_leap_speed     = 8.0f;



	/*
	 * CrabBug animations and related functions
	 */
#if ENABLE_WALK_ANIMATIONS
	static void GenerateHardCodedWalkAnimation(KeyframeAnimation* ka)
	{
		{
			Keyframe kf(0.5f);
			kf.next = 1;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3( 1,  0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3( 0, -1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3( 1,  0.5f, 0 ), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3( 0,  0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3( 1, -1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3( 0,	 0.5f, 0 ), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 2;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3( 0,  0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3( 1,	-1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3( 0,  0.5f, 0 ), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3( 1,  0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3( 0, -1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3( 1,  0.5f, 0 ), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3( 0, -0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3( 1,  1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3( 0, -0.5f, 0 ), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3( 1, -0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3( 0,  1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3( 1, -0.5f, 0 ), Vec3());

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values[Bone::string_table["l leg a 1"]] = BoneInfluence(Vec3( 1, -0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg b 1"]] = BoneInfluence(Vec3( 0,  1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["l leg c 1"]] = BoneInfluence(Vec3( 1, -0.5f, 0 ), Vec3());

			kf.values[Bone::string_table["r leg a 1"]] = BoneInfluence(Vec3( 0, -0.3f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg b 1"]] = BoneInfluence(Vec3( 1,  1.0f, 0 ), Vec3());
			kf.values[Bone::string_table["r leg c 1"]] = BoneInfluence(Vec3( 0, -0.5f, 0 ), Vec3());

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
#endif




	struct CrabLeg
	{
		CBone bones[3];
		CJoint joints[3];
	};

	/*
	* CrabBug private implementation struct
	*/
	struct CrabBug::Imp
	{
		bool init;
		bool clean_init;
		bool experiment_done;

		float score;
		vector<float> score_parts;

		GATrialToken ga_token;
		GAExperiment* experiment;

		Vec3 initial_pos;

		CBone carapace, head, tail;
		CJoint neck, tailj;

		CrabLeg llegs[3];
		CrabLeg rlegs[3];

		vector<Vec3> initial_ee;
		vector<Vec3> prev_ee;

		float timestep, inv_timestep;

		unsigned int tick_age, max_tick_age;

		struct NodeProcessor
		{
			float strict_inputs[91];
			float scratch_memory[256];
			float old_persistent[16];		// may use different limits for "brain" vs typical node
			float new_persistent[16];
			float strict_outputs[5];

			CBone* bone;
			CJoint* joint;
			NodeProcessor* parent;
			vector<NodeProcessor*> children;
			NodeProcessor* brain_connection;

			NodeProcessor() : bone(nullptr), joint(nullptr), parent(nullptr), children(), brain_connection(nullptr)
			{
				memset(strict_inputs, 0, sizeof(strict_inputs));
				memset(scratch_memory, 0, sizeof(scratch_memory));
				memset(old_persistent, 0, sizeof(old_persistent));
				memset(new_persistent, 0, sizeof(new_persistent));
				memset(strict_outputs, 0, sizeof(strict_outputs));
			}

			void Execute(const GPOp& op, const float* constants_table)
			{
				float arg1 = GetOperand(op.arg1, constants_table);
				float arg2 = GPOp::IsUnary(op.opcode) ? 0.0f : GetOperand(op.arg2, constants_table);

				float value;
				switch(op.opcode)
				{
					case 0: value = arg1 + arg2; break;
					case 1: value = arg1 - arg2; break;
					case 2: value = arg1 * arg2; break;
					case 3: value = arg1 / arg2; break;

					case 4: value = isnan(arg1) ? 0.0f : tanhf(arg1); break;

					case 5: value = (arg1 + arg2) * 0.5f; break;
					case 6: value = arg1 > arg2 ? 1.0f : 0.0f;//arg1 > arg2 ? 1.0f : arg1 < arg2 ? -1.0f : 0.0f; break;		// fixes nan

					case 7: value = sqrtf(arg1); if(!isfinite(value)) { value = 0.0f; } break;
					case 8: value = arg1 * arg1; break;
					case 9: value = powf(arg1, arg2); if(!isfinite(value)) { value = 0.0f; } break;

					default: value = 0.0f; break;
				}

				switch(op.dst_class)
				{
					case 0: scratch_memory[op.dst_index] = value; break;
					case 1: new_persistent[op.dst_index] = value; break;
					case 2: strict_outputs[op.dst_index] = value; break;
				}
			}

			float GetOperand(const GPOperand& op, const float* constants_table) const
			{
				switch(op.src)
				{
					case 0:		// integer constant
						return (float)((signed)op.index);

					case 1:		// strict inputs
						return strict_inputs[op.index];
						
					case 2:		// scatch memory
						return scratch_memory[op.index];

					case 3:		// persistent memory
						return old_persistent[op.index];

					case 4:		// parent's scratch memory
						if(parent == nullptr)
							return 0.0f;
						return parent->scratch_memory[op.index];

					case 5:		// sum of children's scratch memory			// NOTE: order of execution is arbitrary
					{
						float tot = 0.0f;
						for(unsigned int i = 0; i < children.size(); ++i)
							tot += children[i]->scratch_memory[op.index];
						return tot;
					}

					case 6:		// brain connection
					{
						if(brain_connection == nullptr)
							return 0.0f;
						return brain_connection->scratch_memory[op.index];
					}

					case 7:		// constants table
						return constants_table[op.index];

					default:
						return 0.0f;
				}
			}
		};

		vector<NodeProcessor> node_processors;

		Imp() : init(false), experiment_done(false), experiment(nullptr), timestep(0), inv_timestep(0), tick_age(0), max_tick_age(MAX_TICK_AGE) { }
		~Imp() { }

		void Init(CrabBug* dood)
		{
			dood->collision_group->SetInternalCollisionsEnabled(true);

			initial_pos = dood->pos;

			no_touchy.dood = dood;
			for(set<RigidBody*>::iterator iter = dood->velocity_change_bodies.begin(); iter != dood->velocity_change_bodies.end(); ++iter)
			{
				bool any = false;
				for(unsigned int i = 0; i < dood->feet.size(); ++i)
					if(dood->feet[i]->body == *iter)
					{
						any = true;
						break;
					}

				if(!any)
					(*iter)->SetContactCallback(&no_touchy);
			}

			SharedInit(dood);
		}

		void ReInit(CrabBug* dood)
		{
			float saved_yaw = dood->yaw;
			float saved_pitch = dood->pitch;

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

			SharedInit(dood);

			dood->yaw = saved_yaw;
			dood->pitch = saved_pitch;
		}

		void SharedInit(CrabBug* dood)
		{
			clean_init = false;
			experiment_done = false;
			tick_age = 0;

			score = 0.0f;
			score_parts.clear();

			if(experiment != nullptr && ga_token.candidate == nullptr)
				ga_token = experiment->GetNextTrial();

			for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
				dood->all_joints[i]->SetOrientedTorque(Vec3());

			initial_ee.clear();

			node_processors.clear();
			node_processors.resize(dood->all_bones.size() + 1);
			node_processors[0].brain_connection = &node_processors[1];
			node_processors[1].brain_connection = &node_processors[0];

			map<CBone*, NodeProcessor*> bone_node_mapping;
			for(unsigned int i = 1; i < node_processors.size(); ++i)
			{
				CBone* bone = dood->all_bones[i - 1];
				bone_node_mapping[bone] = &node_processors[i];
				node_processors[i].bone = bone;
			}

			for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
			{
				CJoint* joint = dood->all_joints[i];
				NodeProcessor* a = bone_node_mapping[joint->a];
				NodeProcessor* b = bone_node_mapping[joint->b];
				a->children.push_back(b);
				b->parent = a;
				b->joint = joint;
			}

			//for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
			//	dood->all_bones[i]->rb->SetLinearVelocity(Vec3(0, 0, 7));
		}

		void Update(CrabBug* dood, const TimingInfo& time)
		{
			if(!init)
			{
				Init(dood);
				init = true;
				return;
			}
			else if(experiment_done || experiment != nullptr && ga_token.candidate == nullptr)
			{
				ReInit(dood);
				return;
			}

			if(!clean_init)
			{
				ReInit(dood);

				if(Random3D::RandInt() % 2 == 0)		// try to prevent everything from running exactly in sync
					return;
				else
					clean_init = true;
			}

			timestep     = time.elapsed;
			inv_timestep = 1.0f / timestep;

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

			float total_mass = 0.0f;
			for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
				total_mass += dood->all_bones[i]->rb->GetMass();

			for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
				dood->all_bones[i]->ComputeDesiredTorqueWithDefaultMoI(dood->all_bones[i]->initial_ori, inv_timestep);

			//dood->DoScriptedMotorControl("Files/Scripts/crab_motor_control.lua");

			//stringstream ss;
			//ss << "age = " << tick_age << endl;
			//for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
			//{
			//	const CJoint& j = *dood->all_joints[i];
			//	Vec3 f = j.sjc->net_impulse_linear * inv_timestep;
			//	Vec3 t = j.sjc->net_impulse_angular * inv_timestep;
			//	ss << "\t" << j.b->name << ": F = (" << f.x << ", " << f.y << ", " << f.z << "); T = (" << t.x << ", " << t.y << ", " << t.z << ")" << endl;
			//}
			//Debug(ss.str());

			if(experiment != nullptr && ga_token.candidate != nullptr && !ga_token.candidate->aborting)
			{
				Vec3 carapace_com = carapace.rb->GetPosition() + carapace.rb->GetOrientation() * carapace.rb->GetLocalCoM();

				vector<Vec3> ee_goals;
				vector<Vec3> ee_delta;
				vector<float> ee_error;
				Vec3 ee_goal_offset = tick_age + 16 <= MAX_TICK_AGE ? ga_token.subtest.x1 : ga_token.subtest.x2;

				if(tick_age == 0)
				{
					prev_ee.clear();
					prev_ee.resize(dood->feet.size());
				}

				for(unsigned int i = 0; i < dood->feet.size(); ++i)
				{
					const Dood::FootState& fs = *dood->feet[i];
					const RigidBody& rb = *fs.body;
					Vec3 ee_pos = rb.GetPosition() + rb.GetOrientation() * fs.ee_pos;

					Vec3 origin = (i % 2 == 0 ? llegs : rlegs)[i / 2].joints[0].sjc->ComputeAveragePosition();

					if(tick_age == 0)
						initial_ee.push_back(ee_pos - origin);
					
					ee_goals.push_back(origin + initial_ee[i] + (i == 2 ? ee_goal_offset : Vec3()));
					Vec3 ee_error_vec = ee_pos - ee_goals[i];
					for(unsigned int j = 0; j < 3; ++j)
					{
						float x = i == 2 ? ((float*)&ee_error_vec)[j] : 0.0f;
						ee_error.push_back(x * x);
					}

					ee_delta.push_back(tick_age == 0 ? Vec3() : ee_error_vec - prev_ee[i]);

					prev_ee[i] = ee_error_vec;
				}

				

				// organize the articulated axes a bit more conveniently
				struct ArticulatedAxis
				{
					CJoint* j;
					unsigned char axis;

					ArticulatedAxis(CJoint* j, unsigned char axis) : j(j), axis(axis) { }
				};
				vector<ArticulatedAxis> articulated_axes;
				for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
				{
					CJoint* joint = dood->all_joints[i];
					SkeletalJointConstraint* sjc = joint->sjc;
					const float* mint = (float*)&sjc->min_torque;
					const float* maxt = (float*)&sjc->max_torque;
					for(unsigned int j = 0; j < 3; ++j, ++mint, ++maxt)
						if(*mint != *maxt)
						{
							articulated_axes.push_back(ArticulatedAxis(joint, j));
						//	Debug(((stringstream&)(stringstream() << "axis " << articulated_axes.size() - 1 << ": \"" << joint->b->name << "\" axis " << (char)(j + 'x') << endl)).str());
						}
				}

				// set strict inputs
				Mat3 unrotate = Mat3::Identity();
				for(unsigned int i = 0; i < node_processors.size(); ++i)
				{
					NodeProcessor& processor = node_processors[i];

					if(tick_age != 0)
					{
						memcpy(processor.old_persistent, processor.new_persistent, sizeof(processor.old_persistent));
						memset(processor.scratch_memory, 0, sizeof(processor.scratch_memory));
					}

					float* iptr = processor.strict_inputs;
					if(processor.bone == nullptr)
					{
						// set inputs for brain
						if(i != 0)
							DEBUG();

						*(iptr++) = carapace_com.y;
						*(iptr++) = (float)tick_age;
						*(iptr++) = timestep;
						*(iptr++) = inv_timestep;
					}
					else
					{
						const CBone* bone = processor.bone;

						*(iptr++) = carapace_com.y;
						*(iptr++) = (float)tick_age;
#if 1
						*(iptr++) = timestep;
						*(iptr++) = inv_timestep;
#endif

						// set rb inputs
#if 1
						const string& name = bone->name;
						*(iptr++) = (bone == &head) ? 1.0f : 0.0f;
						*(iptr++) = (bone == &carapace) ? 1.0f : 0.0f;
						*(iptr++) = (bone == &tail) ? 1.0f : 0.0f;
						*(iptr++) = (name[0] == 'l' ? 1.0f : name[0] == 'r') ? -1.0f : 0.0f;
						*(iptr++) = (name.size() == 9 && name[6] == 'a') ? 1.0f : 0.0f;
						*(iptr++) = (name.size() == 9 && name[6] == 'b') ? 1.0f : 0.0f;
						*(iptr++) = (name.size() == 9 && name[6] == 'c') ? 1.0f : 0.0f;
						*(iptr++) = (name.size() == 9 && name[8] == '1') ? 1.0f : 0.0f;
						*(iptr++) = (name.size() == 9 && name[8] == '2') ? 1.0f : 0.0f;
						*(iptr++) = (name.size() == 9 && name[8] == '3') ? 1.0f : 0.0f;
#endif

						const RigidBody& rb = *bone->rb;
						Mat3 rm = rb.GetOrientation().ToMat3();
						Mat3 urm = unrotate * rm;
						//Mat3 goalrm = unrotate * (rb.GetOrientation() * Quaternion::Reverse(bone->initial_ori)).ToMat3();
						//Mat3 goalrm = unrotate * (rb.GetOrientation() * Quaternion::Reverse(bone == &carapace ? tick_age + 16 < MAX_TICK_AGE ? ga_token.subtest.ori1 : ga_token.subtest.ori2 : bone->initial_ori)).ToMat3();
						PushVec3(iptr, rb.GetLocalCoM());
						PushVec3(iptr, unrotate * (rb.GetPosition() - carapace_com + rm * rb.GetLocalCoM()));
						PushMat3(iptr, urm);
#if 1
						//Vec3 goal_error_rvec = (rb.GetOrientation() * Quaternion::Reverse(bone == &carapace ? tick_age + 16 < MAX_TICK_AGE ? ga_token.subtest.ori1 : ga_token.subtest.ori2 : bone->initial_ori)).ToRVec();
						//PushVec3(iptr, goal_error_rvec);
						//PushMat3(iptr, goalrm);
#endif
						PushVec3(iptr, unrotate * rb.GetLinearVelocity());
						PushVec3(iptr, unrotate * rb.GetAngularVelocity());
						*(iptr++) = rb.GetMass();

						if(processor.joint != nullptr)
						{
							// set joint inputs
							const SkeletalJointConstraint& sjc = *processor.joint->sjc;
							PushVec3(iptr, processor.joint->GetRVec());
							PushVec3(iptr, unrotate * (sjc.ComputeAveragePosition() - carapace_com));
							PushVec3(iptr, tick_age == 0 ? Vec3() : unrotate * sjc.net_impulse_linear);
							PushVec3(iptr, tick_age == 0 ? Vec3() : unrotate * sjc.net_impulse_angular);
							if(sjc.enable_motor)
								iptr += 12;
							else
							{
								PushVec3(iptr, sjc.min_extents);
								PushVec3(iptr, sjc.max_extents);
								PushVec3(iptr, sjc.min_torque);
								PushVec3(iptr, sjc.max_torque);
							}

#if 1
							Mat3 oa = unrotate * processor.joint->oriented_axes;
							PushMat3(iptr, oa);
#endif

							if(bone->name.size() == 9)
							{
								unsigned int fs_index = (bone->name[0] == 'l' ? 0 : 1) + (bone->name[7] == 'b' ? 2 : bone->name[7] == 'c' ? 4 : 0);
								Dood::FootState* fs = dood->feet[fs_index];

								Vec3 current_ee_pos = rb.GetPosition() + rm * fs->ee_pos;

								PushVec3(iptr, unrotate * (ee_goals[fs_index] - current_ee_pos));
								PushVec3(iptr, unrotate * ee_delta[fs_index]);
							}
							else
							{
								PushVec3(iptr, Vec3());
								PushVec3(iptr, Vec3());
							}

							if(bone->name.size() == 9 && bone->name[8] == '3')
							{
								for(unsigned int j = 0; j < dood->feet.size(); ++j)
								{
									if(dood->feet[j]->body == bone->rb)
									{
										// set foot inputs
										Dood::FootState* fs = dood->feet[j];

										*(iptr++) = tick_age == 0 ? 0.0f : fs->contact_points.size();

										Vec3 current_ee_pos = rb.GetPosition() + rm * fs->ee_pos;
										PushVec3(iptr, unrotate * (current_ee_pos - carapace_com));
										PushVec3(iptr, tick_age == 0 ? Vec3() : unrotate * fs->net_force);
										PushVec3(iptr, tick_age == 0 ? Vec3() : unrotate * fs->net_torque);
							
										Vec3 avg_pos, normal;
										if(!fs->contact_points.empty() && tick_age != 0)
										{
											for(unsigned int k = 0; k < fs->contact_points.size(); ++k)
											{
												avg_pos += fs->contact_points[k].pos;
												normal += fs->contact_points[k].normal;
											}
											avg_pos /= (float)fs->contact_points.size();
											normal /= (float)fs->contact_points.size();		// not actually normalizing
										}
										PushVec3(iptr, unrotate * (avg_pos - carapace_com));
										PushVec3(iptr, unrotate * (normal));
							
										static bool printed_inputs = false;
										if(!printed_inputs)
										{
											printed_inputs = true;
											Debug(((stringstream&)(stringstream() << "num inputs = " << (iptr - processor.strict_inputs) << endl)).str());
										}
							
										break;
									}
								}
							}

							//static bool printed_inputs = false;
							//if(!printed_inputs)
							//{
							//	printed_inputs = true;
							//	Debug(((stringstream&)(stringstream() << "num inputs = " << (iptr - processor.strict_inputs) << endl)).str());
							//}
						}
						else if(i != 1)
							DEBUG();
					}

					if(iptr > processor.strict_inputs + (sizeof(processor.strict_inputs) / sizeof(float)))
						DEBUG();
				}

				// evaluate
				for(unsigned int i = 0; i < ga_token.candidate->compiled.size(); ++i)
				{
					const GPOp& op = ga_token.candidate->compiled[i];
					if(op.brain)
						node_processors[0].Execute(op, ga_token.candidate->constants);
					else
						for(unsigned int i = 1; i < node_processors.size(); ++i)			// no guarantee is made of synchronous (buffered) execution
							node_processors[i].Execute(op, ga_token.candidate->constants);
				}

				// apply outputs
				for(unsigned int i = 0; i < node_processors.size(); ++i)
				{
					NodeProcessor& processor = node_processors[i];
					if(processor.joint != nullptr)
					{
#if 0
						if(!processor.joint->sjc->enable_motor)
						{
							//if     (processor.bone == &llegs[0].bones[0] || processor.bone == &rlegs[0].bones[0])
							processor.joint->SetOrientedTorque(Vec3(processor.strict_outputs[0], processor.strict_outputs[1], processor.strict_outputs[2]));
							//else if(processor.bone == &llegs[1].bones[0] || processor.bone == &rlegs[1].bones[0])
							//	processor.joint->SetOrientedTorque(Vec3(processor.strict_outputs[3], processor.strict_outputs[4], processor.strict_outputs[5]));
							//else if(processor.bone == &llegs[2].bones[0] || processor.bone == &rlegs[2].bones[0])
							//	processor.joint->SetOrientedTorque(Vec3(processor.strict_outputs[6], processor.strict_outputs[7], processor.strict_outputs[8]));
						}
#else

						if(processor.joint == &neck)
						{}
						else if(processor.joint == &tailj)
						{}
						else if(processor.parent->bone == &carapace)
							processor.joint->SetOrientedTorque(Vec3(processor.strict_outputs[0], processor.strict_outputs[1], processor.strict_outputs[2]));
						else if(processor.children.empty())
							processor.joint->SetOrientedTorque(Vec3(processor.strict_outputs[3], 0.0f, 0.0f));
						else
							processor.joint->SetOrientedTorque(Vec3(processor.strict_outputs[4], 0.0f, 0.0f));
#endif
					}
				}



				// scoring
				float ori_error = 0.0f;
				float pos_error = 0.0f;
				float vel_error = 0.0f;
				float rot_error = 0.0f;
				float rvec_error = 0.0f;

				float energy_cost = 0.0f;
				for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
					energy_cost += dood->all_joints[i]->actual.ComputeMagnitudeSquared();

				for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
				{
					const CBone& bone = *dood->all_bones[i];

					float bone_weight = 1.0f;//tick_age < 10 ? 0.0f : 1.0f;
					//if(bone.name == "carapace")
					//	bone_weight = 1.0f;
					//else if(bone.name == "tail")
					//	bone_weight = 0.01f;
					//else if(bone.name == "head")
					//	bone_weight = 0.1f;
					//else if(bone.name[1] == ' ' && bone.name[8] == '2')
					//	bone_weight = 0.1f;
					//else
					//	bone_weight = 0.0f;

					const RigidBody& rb = *bone.rb;

					Quaternion ori = rb.GetOrientation();
					
					//if(bone.name != "tail")
					if(&bone == &carapace)// || (bone.name.size() > 4 && bone.name[bone.name.size() - 1] == '1'))
					{
						float oe_part = (Quaternion::Reverse(bone.initial_ori) * ori).ToRVec().ComputeMagnitudeSquared();
						oe_part = max(0.0f, oe_part - 0.04f);
						ori_error += bone_weight * oe_part;
					}

					Vec3 local_com = rb.GetLocalCoM();
					Vec3 initial_pos = bone.initial_pos + bone.initial_ori * local_com;
					Vec3 current_pos = rb.GetPosition() + ori * local_com;
					if(bone.name == "carapace")
						pos_error += bone_weight * (initial_pos - current_pos).ComputeMagnitudeSquared();

					if(bone.name == "carapace")
						vel_error += bone_weight * rb.GetLinearVelocity().ComputeMagnitudeSquared();
					rot_error += bone_weight * rb.GetAngularVelocity().ComputeMagnitudeSquared();
				}

				for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
				{
					CJoint& joint = *dood->all_joints[i];
					Vec3 rvec = joint.GetRVec();
					//if(joint.b != &tail)
					{
						if(tick_age >= 1)
							rvec_error += (rvec - joint.prev_rvec).ComputeMagnitudeSquared();
					}
					joint.prev_rvec = rvec;
				}

				//energy_cost = max(0.0f, energy_cost - 500.0f);

				

				float bad_touch = 0.0f;
				if(tick_age != 0)
				{
					bad_touch = float(dood->new_contact_points.size());
					for(unsigned int i = 0; i < dood->feet.size(); ++i)
						bad_touch += dood->feet[i]->contact_points.size();
				}


				float input_coverage = 0.0f;
				if(tick_age == 0)
					input_coverage = float(88 - GACandidate::GetInputCoverage(ga_token.candidate->compiled));

				float stay_sideways = 0.0f;
				float move_forward = 0.0f;
				if(tick_age + 1 == MAX_TICK_AGE || tick_age + 16 == MAX_TICK_AGE)
				{
					Vec3 lcom = carapace.rb->GetLocalCoM();
					Vec3 cpos = (carapace.rb->GetPosition() + carapace.rb->GetOrientation() * lcom) - (carapace.initial_pos + carapace.initial_ori * lcom);
					stay_sideways = cpos.x * cpos.x;
					move_forward = expf(-cpos.z * 0.1f);
					//carapace_ori = (carapace.rb->GetOrientation() * Quaternion::Reverse(tick_age + 16 == MAX_TICK_AGE ? ga_token.subtest.ori1 : ga_token.subtest.ori2)).ToRVec();
				}

				vector<float> inst_scores;

				//float cxyz[3] = { carapace_ori.x, carapace_ori.y, carapace_ori.z };

				//inst_scores.push_back(stay_sideways *  10.0f);
				//inst_scores.push_back(move_forward  * 100.0f);
				//for(unsigned int i = 0; i < 3; ++i)
				//{
				//	float x = cxyz[i];
				//	x *= x;
				//	x *= 1000.0f;
				//	inst_scores.push_back(x);
				//}



				//inst_scores.push_back(ori_error * 1.0f);


				//inst_scores.push_back(bad_touch * 1.0f);
				//inst_scores.push_back(pos_error * 100.0f);
				//inst_scores.push_back(vel_error * 1.0f);
				//inst_scores.push_back(tick_age + 1 == MAX_TICK_AGE ? ori_error * 5.0f : 0.0f);
				//inst_scores.push_back(rot_error * 0.1f);
				//inst_scores.push_back(energy_cost * 0.0000001f);

				float ee_xyz[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
				if(tick_age + 16 == MAX_TICK_AGE)
					for(unsigned int i = 0; i < ee_error.size(); ++i)
						ee_xyz[i % 3] += ee_error[i];
				else if(tick_age + 1 == MAX_TICK_AGE)
					for(unsigned int i = 0; i < ee_error.size(); ++i)
						ee_xyz[i % 3 + 3] += ee_error[i];

				//inst_scores.push_back(ee_xyz[0] * 100.0f);
				inst_scores.push_back(ee_xyz[1] * 100.0f);
				//inst_scores.push_back(ee_xyz[2] * 100.0f);

				//inst_scores.push_back(ee_xyz[3] * 100.0f);
				inst_scores.push_back(ee_xyz[4] * 100.0f);
				//inst_scores.push_back(ee_xyz[5] * 100.0f);

				//inst_scores.push_back(rvec_error * 1.0f);
				//for(unsigned int i = 0; i < 19; ++i)
				//	inst_scores.push_back(ga_token.subtest.id == i ? rvec_error : 0.0f);
				//for(unsigned int i = 0; i < 19; ++i)
				//	inst_scores.push_back(ga_token.subtest.id == i ? ori_error * 500.0f : 0.0f);

				rvec_error *= 500.0f;
				//inst_scores.push_back(tick_age + 4 == MAX_TICK_AGE ? rvec_error     : 0.0f);
				//inst_scores.push_back(tick_age + 3 == MAX_TICK_AGE ? rvec_error * 2 : 0.0f);
				//inst_scores.push_back(tick_age + 2 == MAX_TICK_AGE ? rvec_error * 4 : 0.0f);
				//inst_scores.push_back(tick_age + 1 == MAX_TICK_AGE ? rvec_error * 8 : 0.0f);

				//inst_scores.push_back(tick_age + 1 == MAX_TICK_AGE ? rvec_error : 0.0f);

				//inst_scores.push_back(input_coverage * 0.01f);

				//inst_scores.push_back(tick_age == 0 && ga_token.candidate->compiled.size() < 10 ? (10 - ga_token.candidate->compiled.size()) * 200.0f : 0.0f);
				
				if(score_parts.empty())
					score_parts.resize(inst_scores.size());
				else
					assert(score_parts.size() == inst_scores.size());

				for(unsigned int i = 0; i < score_parts.size(); ++i)
				{
					score_parts[i] += inst_scores[i];
					score += inst_scores[i];
				}
			}

			++tick_age;

			if(experiment != nullptr && ga_token.candidate != nullptr)
			{
				float fail_threshold = experiment->GetEarlyFailThreshold();
				if(fail_threshold >= 0.0f && score + ga_token.candidate->score >= fail_threshold)
					ga_token.candidate->aborting = true;

				if(tick_age >= max_tick_age || ga_token.candidate->aborting)
				{
					if(!experiment_done)
					{
						experiment->TrialFinished(ga_token, score, score_parts, tick_age);
						experiment_done = true;

						ga_token = experiment->GetNextTrial();
					}
				}
			}
		}

		static void PushVec3(float*& ptr, const Vec3& value)
		{
			memcpy(ptr, &value, 3 * sizeof(float));
			ptr += 3;
		}

		static void PushMat3(float*& ptr, const Mat3& value)
		{
			memcpy(ptr, value.values, 9 * sizeof(float));
			ptr += 9;
		}

		void GradientSearch(unsigned int iterations, vector<float>& f, vector<float>& iv_mins, vector<float>& iv_maxs, const GenericMatrix& A, const vector<float>& b, const vector<float>& W)
		{
			assert(f.size() == A.h);
			assert(f.size() == iv_mins.size());
			assert(f.size() == iv_maxs.size());
			assert(W.size() == A.w);

			vector<float> h(W.size());
			vector<float> gradient(A.h);

#if DEBUG_GRADIENT_SEARCH_PROGRESS
			Debug(((stringstream&)(stringstream() << "age = " << tick_age << "; starting gradient search, max iterations = " << iterations << endl)).str());
#endif

			float old_y1;
			for(unsigned int i = 0; i < iterations; ++i)
			{
				float y1 = ScoreProposedF(A, b, f, W, h);

				if(i != 0 && old_y1 - y1 < 0.0001f)
				{
#if DEBUG_GRADIENT_SEARCH_PROGRESS
					Debug(((stringstream&)(stringstream() << "\ti = " << i << "; y1 = " << y1 << "; early return 1" << endl)).str());
#endif
					return;
				}

				// compute gradient
				float* gend = gradient.data() + A.h;
				const float* aptr = A.data;
				for(const float *uptr = h.data(), *uend = uptr + h.size()/*, *aptr = A.data*/, *wptr = W.data(); uptr != uend; ++uptr, ++wptr)
				{
					float two_w_u = 2.0f * *wptr * *uptr;
					for(float* gptr = gradient.data(); gptr != gend; ++gptr, ++aptr)
						*gptr += two_w_u * *aptr;
				}

				float gr_magsq = 0.0f;
				for(unsigned int j = 0; j < A.h; ++j)
				{
					float& gj = gradient[j];
					gr_magsq += gj * gj;
				}

				float original_grmag = sqrtf(gr_magsq);

				// prevent gradients from exceeding the limits of the independent variables
				// TODO: make gradients satisfy null space constraints
				for(unsigned int j = 0; j < A.h; ++j)
				{
					if(f[j] >= iv_maxs[j] && gradient[j] < 0)
						gradient[j] = 0.0f;
					else if(f[j] <= iv_mins[j] && gradient[j] > 0)
						gradient[j] = 0.0f;
				}

				gr_magsq = 0.0f;
				for(unsigned int j = 0; j < A.h; ++j)
				{
					float& gj = gradient[j];
					gr_magsq += gj * gj;
				}

				if(gr_magsq < 0.000001f)
				{
#if DEBUG_GRADIENT_SEARCH_PROGRESS
					Debug(((stringstream&)(stringstream() << "\ti = " << i << "; y1 = " << y1 << "; gr mag = " << original_grmag << "; gr' mag = " << sqrtf(gr_magsq) << "; early return 2" << endl)).str());
#endif
					return;
				}

				// find the minimum of the quadratic along this ray
				vector<float> best_f = f;
				float* fend = f.data() + f.size();
				for(float *fptr = f.data(), *gptr = gradient.data(); fptr != fend; ++fptr, ++gptr)
					*fptr -= *gptr;
				float y2 = ScoreProposedF(A, b, f, W, h);
				for(float *fptr = f.data(), *gptr = gradient.data(); fptr != fend; ++fptr, ++gptr)
					*fptr -= *gptr;
				float y3 = ScoreProposedF(A, b, f, W, h);

				// solve for the coefficients in ax^2 + bx + c... except we don't actually care about c
				float qa = 0.5f * (y1 + y3) - y2;
				float qb = -1.5f * y1 + 2.0f * y2 - 0.5f * y3;	

				if(qa <= 0.0f)				// if the quadratic is concave down, we have a problem
				{
#ifndef NDEBUG
					//__debugbreak();
#endif
					return;
				}

				// d(ax^2 + bx + c) / dx = 2ax + b; find where this = 0 to find the vertex of the parabola
				float extremum_x = -0.5f * qb / qa;

				for(float *fptr = f.data(), *bptr = best_f.data(), *gptr = gradient.data(); fptr != fend; ++fptr, ++bptr, ++gptr)
					*fptr = *bptr - *gptr * extremum_x;

				// make sure we stay within constraints
				// TODO: do this better?	(see https://github.com/alubitz/cibraryengine/blob/b054e52d69edccd50b140306e8f70c9565554754/TestProject/Soldier.cpp )
				for(unsigned int j = 0; j < f.size(); ++j)
					f[j] = min(iv_maxs[j], max(iv_mins[j], f[j]));

				old_y1 = y1;

#if DEBUG_GRADIENT_SEARCH_PROGRESS
				Debug(((stringstream&)(stringstream() << "\ti = " << i << "; y1 = " << y1 << "; gr mag = " << original_grmag << "; gr' mag = " << sqrtf(gr_magsq) << endl)).str());
#endif
			}
		}

		float ScoreProposedF(const GenericMatrix& A, const vector<float>& b, const vector<float>& f, const vector<float>& W, vector<float>& h) const
		{
			FToErrors(A, f, b, h);
			return ScoreErrors(h, W);
		}

		void FToErrors(const GenericMatrix& A, const vector<float>& f, const vector<float>& b, vector<float>& h) const
		{
			assert(h.size() == b.size() && h.size());

			const float* fend = f.data() + f.size();
			const float* aptr = A.data;
			const float* bptr = b.data();

			for(float *hptr = h.data(), *hend = hptr + h.size(); hptr != hend; ++hptr, ++bptr)
			{
				*hptr = -*bptr;
				for(const float *fptr = f.data(); fptr != fend; ++fptr, ++aptr)
					*hptr += *fptr * *aptr;
			}
		}

		float ScoreErrors(const vector<float>& h, const vector<float> W) const
		{
			assert(h.size() == W.size());

			float tot = 0.0f;
			for(const float *hptr = h.data(), *hend = hptr + h.size(), *wptr = W.data(); hptr != hend; ++hptr, ++wptr)
				tot += *hptr * *hptr * *wptr;
			return tot;
		}



		struct NoTouchy : public ContactCallback
		{
			Dood* dood;
			void OnContact(const ContactPoint& contact) { dood->new_contact_points.push_back(MyContactPoint(contact, dood)); }
			void AfterResolution(const ContactPoint& cp) { dood->old_contact_points.push_back(MyContactPoint(cp, dood)); }
		} no_touchy;
	};



	
	/*
	 * CrabBug methods
	 */
	CrabBug::CrabBug(GameState* game_state, GAExperiment* experiment, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		imp(nullptr),
		crab_heading(new CrabHeading())
	{
		hp *= 0.5f;

		//yaw = Random3D::Rand(float(M_PI) * 2.0f);

		ragdoll_timer = 10.0f;

		// character animation stuff
		posey->active_poses.push_back(crab_heading);

#if ENABLE_WALK_ANIMATIONS
		KeyframeAnimation kw, kr;
		GenerateHardCodedWalkAnimation(&kw);
		GenerateRestPose(&kr);

		posey->active_poses.push_back(new WalkPose(this, &kr, &kw, &kw, &kw, &kw, nullptr, nullptr));
#endif

		standing_callback.angular_coeff = 0.0f;

		imp = new Imp();
		imp->experiment = experiment;
	}

	void CrabBug::InnerDispose()
	{
		Dood::InnerDispose();

		if(imp) { delete imp; imp = nullptr; }
	}

	void CrabBug::DoJumpControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward)
	{
		if(standing_callback.IsStanding() && control_state->GetBoolControl("leap") && time.total > jump_start_timer)
		{
			// crab bug leaps forward
			float leap_angle = 0.4f;
			Vec3 leap_vector = (forward * (cosf(leap_angle)) + Vec3(0, sinf(leap_angle), 0));

			standing_callback.ApplyVelocityChange(leap_vector * bug_leap_speed);

			jump_start_timer = time.total + bug_leap_duration;
		}
	}

	void CrabBug::PreUpdatePoses(const TimingInfo& time)
	{
		crab_heading->yaw = yaw;

		imp->Update(this, time);
	}

	void CrabBug::Update(const TimingInfo& time)
	{
#if DIE_AFTER_ONE_SECOND
		if(time.total > 1.0f)
			Die(Damage());
#endif

		Dood::Update(time);
	}

	void CrabBug::RegisterFeet()
	{
		feet.push_back(new FootState(Bone::string_table["l leg a 3"], Vec3( 0.288f,  0.0f,  1.293f)));
		feet.push_back(new FootState(Bone::string_table["r leg a 3"], Vec3(-0.288f,  0.0f,  1.293f)));
		feet.push_back(new FootState(Bone::string_table["l leg b 3"], Vec3( 2.068f,  0.0f,  0.470f)));
		feet.push_back(new FootState(Bone::string_table["r leg b 3"], Vec3(-2.068f,  0.0f,  0.470f)));
		feet.push_back(new FootState(Bone::string_table["l leg c 3"], Vec3( 0.798f,  0.0f, -1.366f)));
		feet.push_back(new FootState(Bone::string_table["r leg c 3"], Vec3(-0.798f,  0.0f, -1.366f)));
	}

	void CrabBug::MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi)
	{
		cheaty_rot = Vec3();
		Dood::MaybeSinkCheatyVelocity(timestep, cheaty_vel, cheaty_rot, net_mass, net_moi);
	}

	void CrabBug::InitBoneHelpers()
	{
		Dood::InitBoneHelpers();

		RegisterBone( imp->carapace = CBone(this, "carapace") );
		RegisterBone( imp->head     = CBone(this, "head"    ) );
		RegisterBone( imp->tail     = CBone(this, "tail"    ) );

		static const string sides[2] = { "l", "r" };
		static const string legs [3] = { "a", "b", "c" };
		static const string bones[3] = { "1", "2", "3" };

		for(int i = 0; i < 2; ++i)
		{
			CrabLeg* legs_array = i == 0 ? imp->llegs : imp->rlegs;
			for(int j = 0; j < 3; ++j)
				for(int k = 0; k < 3; ++k)
					RegisterBone( legs_array[j].bones[k] = CBone(this, ((stringstream&)(stringstream() << sides[i] << " leg " << legs[j] << " " << bones[k])).str()) );
		}
	}

	void CrabBug::InitJointHelpers()
	{
		Dood::InitJointHelpers();

		float N = 200, T = 150;

		float joint_strengths[3] = { 1400, 1100, 900 };
		float leg_multipliers[3] = { 0.9f, 1.0f, 0.9f };

		RegisterJoint( imp->neck  = CJoint( this, imp->carapace, imp->head, N ));
		RegisterJoint( imp->tailj = CJoint( this, imp->carapace, imp->tail, T ));

		for(int i = 0; i < 2; ++i)
		{
			CrabLeg* legs_array = i == 0 ? imp->llegs : imp->rlegs;
			for(int j = 0; j < 3; ++j)
				for(int k = 0; k < 3; ++k)
				{
					float x = leg_multipliers[j] * joint_strengths[k];
					float yz = k == 0 ? x : 0.0f;							// the 'knees' and 'ankes' can only rotate and torque on one axis

					RegisterJoint( legs_array[j].joints[k] = CJoint( this, k == 0 ? imp->carapace : legs_array[j].bones[k - 1], legs_array[j].bones[k], x, yz, yz) ); 
				}
		}

		for(unsigned int i = 0; i < all_joints.size(); ++i)
			//if(all_joints[i]->a != &imp->carapace || all_joints[i]->b == &imp->head || all_joints[i]->b == &imp->tail)
			if(all_joints[i]->b == &imp->head || all_joints[i]->b == &imp->tail || all_joints[i]->b->name[0] == 'r' || all_joints[i]->b->name[6] != 'b')
				all_joints[i]->sjc->enable_motor = true;
	}

	void CrabBug::DoInitialPose()
	{
		pos.y += 20.0f;

		Dood::DoInitialPose();

		//if(imp->ga_token.candidate != nullptr)
		//{
		//	if(imp->ga_token.subtest.id >= 13)
		//	{
		//		unsigned int id = imp->ga_token.subtest.id - 13;
		//
		//		float scalar = imp->ga_token.subtest.scalar;
		//
		//		CrabLeg* leg = nullptr;
		//		Vec3 axes[3] = {
		//			Vec3( 0.2068f, -0.0443f, -0.0094f ),
		//			Vec3( 0.0336f,  0.0f,    -0.1125f ),
		//			Vec3(-0.2552f,  0.0232f, -0.1112f )
		//		};
		//		Vec3 axis;
		//		bool flip = false;
		//		switch(id)
		//		{
		//			case 0: leg = &imp->llegs[0]; axis = axes[0]; break;
		//			case 1: leg = &imp->llegs[1]; axis = axes[1]; break;
		//			case 2: leg = &imp->llegs[2]; axis = axes[2]; break;
		//			case 3: leg = &imp->rlegs[0]; axis = axes[0]; flip = true; break;
		//			case 4: leg = &imp->rlegs[1]; axis = axes[1]; flip = true; break;
		//			case 5: leg = &imp->rlegs[2]; axis = axes[2]; flip = true; break;
		//		}
		//		if(flip)
		//		{
		//			axis.y = -axis.y;
		//			axis.z = -axis.z;
		//		}
		//
		//		for(unsigned int i = 0; i < 2; ++i)
		//		{
		//			CrabLeg* side = i == 0 ? imp->llegs : imp->rlegs;
		//			for(unsigned int j = 0; j < 3; ++j)
		//				side[j].bones[0].posey->ori = Quaternion::Identity();
		//		}
		//
		//		leg->bones[0].posey->ori = Quaternion::FromRVec(Vec3::Normalize(axis) * (-0.5f * scalar));
		//	}
		//}
	}

	void CrabBug::PostCPHFT(float timestep)
	{
		Dood::PostCPHFT(timestep);
		old_contact_points.clear();
		new_contact_points.clear();
	}
}
