#include "StdAfx.h"
#include "CrabBug.h"

#include "WalkPose.h"

#include "CBone.h"
#include "CJoint.h"

#include "GAExperiment.h"

#define DIE_AFTER_ONE_SECOND			0

#define ENABLE_WALK_ANIMATIONS			0

#define DEBUG_GRADIENT_SEARCH_PROGRESS	0

#define MAX_TICK_AGE					15

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
		CBone* bones[3];
		CJoint* joints[3];
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

		CBone *carapace, *head, *tail;
		CJoint *neck, *tailj;

		CrabLeg llegs[3];
		CrabLeg rlegs[3];

		vector<Vec3> initial_ee;

		float timestep, inv_timestep;

		unsigned int tick_age, max_tick_age;

		Imp() : init(false), experiment_done(false), experiment(nullptr), timestep(0), inv_timestep(0), tick_age(0), max_tick_age(MAX_TICK_AGE) { }
		~Imp() { }

		void Init(CrabBug* dood)
		{
			dood->collision_group->SetInternalCollisionsEnabled(true);

			initial_pos = dood->pos;

			SharedInit(dood);
		}

		void ReInit(CrabBug* dood)
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

			SharedInit(dood);
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

				//if(Random3D::RandInt() % 2 == 0)
				//	clean_init = true;
				//else
				//	return;

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
				Vec3 carapace_com = carapace->rb->GetPosition() + carapace->rb->GetOrientation() * carapace->rb->GetLocalCoM();

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

				// fudge desired foot torques
				for(unsigned int i = 0; i < 2; ++i)
				{
					CrabLeg* side = i == 0 ? llegs : rlegs;
					for(unsigned int j = 0; j < 3; ++j)
					{
						CrabLeg& leg = side[j];
						Vec3& desired_torque = leg.bones[2]->desired_torque;
						if(tick_age != 0)
						{
							Dood::FootState* fs = dood->feet[j * 2 + i];
							const float* foot_matches = ga_token.candidate->foot_t_matching + j * 3;
							desired_torque.x -= fs->net_force.x * foot_matches[0];
							desired_torque.y -= fs->net_force.y * foot_matches[1];
							desired_torque.z -= fs->net_force.z * foot_matches[2];
						}
						const float* foot_absorbs = ga_token.candidate->foot_t_absorbed + j * 3;
						desired_torque.x *= foot_absorbs[0];
						desired_torque.y *= foot_absorbs[1];
						desired_torque.z *= foot_absorbs[2];

						leg.joints[2]->SetTorqueToSatisfyB();
						leg.bones[1]->desired_torque += leg.bones[2]->desired_torque - leg.bones[2]->applied_torque;
						leg.bones[2]->desired_torque = leg.bones[2]->applied_torque;
						leg.joints[1]->SetTorqueToSatisfyB();
						leg.bones[0]->desired_torque += leg.bones[1]->desired_torque - leg.bones[1]->applied_torque;
						leg.bones[1]->applied_torque = leg.bones[1]->applied_torque;
					}
				}

				// compute jacobian matrix
				static const unsigned int num_rows = 36;					// equal to the number of articulated axes
				static const unsigned int num_bone_torques = 21 * 3;
				static const unsigned int num_cols = num_bone_torques;// + 3;	// we care about the torque on every bone, and also the net force on the carapace

				assert(articulated_axes.size() == num_rows);

				vector<float> jdata(num_rows * num_cols);
				GenericMatrix J(num_cols, num_rows, jdata.data());

				vector<float> min_torques(num_rows);
				vector<float> max_torques(num_rows);

				for(unsigned int i = 0; i < num_rows; ++i)
				{
					float* row = J[i];

					ArticulatedAxis aa = articulated_axes[i];
					const CJoint* joint = aa.j;
					Vec3 axis;
					memcpy(&axis, joint->oriented_axes.values + aa.axis * 3, 3 * sizeof(float));

					// compute direct effects of a torque on this axis on the adjacent bones
					for(unsigned int j = 0; j < dood->all_bones.size(); ++j)
						if(dood->all_bones[j] == joint->a)
						{
							memcpy(row + j * 3, &axis,  3 * sizeof(float));
							break;
						}

					Vec3 naxis = -axis;
					if(joint->a == carapace && joint != neck && joint != tailj)		// is it a hip joint?  if so we also affect the other leg bones
					{
						CrabLeg* side_legs = joint->b->name[0] == 'l' ? llegs : rlegs;
						CrabLeg* leg = nullptr;
						float* fixed_xfrac = nullptr;
						for(unsigned int j = 0; j < 3; ++j)
							if(side_legs[j].bones[0] == joint->b)
							{
								leg = &side_legs[j];
								fixed_xfrac = ga_token.candidate->leg_fixed_xfrac + j * 2;
								break;
							}
						assert(leg != nullptr);

						unsigned int bone_indices[3];
						for(unsigned int j = 0; j < dood->all_bones.size(); ++j)
							for(unsigned int k = 0; k < 3; ++k)
								if(dood->all_bones[j] == leg->bones[k])
									bone_indices[k] = j;
						assert(bone_indices[0] != 0 && bone_indices[1] != 0 && bone_indices[2] != 0);

						Vec3 leg_axis = Vec3::Normalize((leg->joints[1]->oriented_axes + leg->joints[2]->oriented_axes) * Vec3(1, 0, 0));
						Vec3 unsupported = naxis - leg_axis * Vec3::Dot(leg_axis, naxis);
						Vec3 uaxis = unsupported * fixed_xfrac[0];
						Vec3 vaxis = unsupported * fixed_xfrac[1];
						naxis -= uaxis;
						naxis -= vaxis;

						//// feet get special treatment
						//unsigned int leg_index = joint.b->name[6] - 'a';
						//vaxis.x *= ga_token.candidate->foot_t_absorbed[leg_index * 3];
						//vaxis.y *= ga_token.candidate->foot_t_absorbed[leg_index * 3 + 1];
						//vaxis.z *= ga_token.candidate->foot_t_absorbed[leg_index * 3 + 2];

						memcpy(row + bone_indices[0] * 3, &naxis, 3 * sizeof(float));
						memcpy(row + bone_indices[1] * 3, &uaxis, 3 * sizeof(float));
						memcpy(row + bone_indices[2] * 3, &vaxis, 3 * sizeof(float));
					}
					else
					{
						if(joint->b->name.size() == 9 && joint->b->name[8] == '3')	// feet get special treatment
						{
							unsigned int leg_index = joint->b->name[6] - 'a';
							naxis.x *= ga_token.candidate->foot_t_absorbed[leg_index * 3];
							naxis.y *= ga_token.candidate->foot_t_absorbed[leg_index * 3 + 1];
							naxis.z *= ga_token.candidate->foot_t_absorbed[leg_index * 3 + 2];
						}

						for(unsigned int j = 0; j < dood->all_bones.size(); ++j)
							if(dood->all_bones[j] == joint->b)
							{
								memcpy(row + j * 3, &naxis, 3 * sizeof(float));
								break;
							}
					}

					// compute linear (force) effects of each joint on the carapace
					//Vec3 carapace_offset = carapace_com - joint.sjc->ComputeAveragePosition();
					//Vec3 carapace_force = Vec3::Cross(axis, carapace_offset);
					//memcpy(row + num_bone_torques, &carapace_force, 3 * sizeof(float));



					min_torques[i] = ((float*)&joint->sjc->min_torque)[aa.axis];
					max_torques[i] = ((float*)&joint->sjc->max_torque)[aa.axis];

					assert(min_torques[i] != max_torques[i]);
				}

#if 0			// debug A matrix
				stringstream ss2;
				for(unsigned int i = 0; i < J.h; ++i)
				{
					ss2 << '\t';
					if(i == 0)
						ss2 << "[[";
					else
						ss2 << " [";
					for(unsigned int j = 0; j < J.w; ++j)
					{
						if(j != 0)
							ss2 << ", ";
						ss2 << J[i][j];
					}
					ss2 << " ]";
					if(i + 1 == J.h)
						ss2 << ']';
					ss2 << endl;
				}
				Debug(ss2.str());
#endif

				// prep for gradient descent
				vector<float> goal_values(num_cols);
				vector<float> weights(num_cols);

				Vec3* goal_ptr = (Vec3*)goal_values.data();
				Vec3* weight_ptr = (Vec3*)weights.data();
				for(unsigned int i = 0; i < dood->all_bones.size(); ++i, ++goal_ptr, ++weight_ptr)
				{
					CBone& bone = *dood->all_bones[i];
					*goal_ptr = bone.desired_torque;

					unsigned int weight_index = i < 12 ? i : i - 9;
					assert(weight_index == i || bone.name[0] == 'r' && bone.name[1] == ' ');	// non-symmetric bones in first three indices, then all L bones, then R

					float weight = ga_token.candidate->bone_t_weights[weight_index];
					*weight_ptr = Vec3(weight, weight, weight);
				}
				//*goal_ptr = Vec3();//Vec3(0, 9.8f * total_mass, 0);
				//*weight_ptr = Vec3();//Vec3(1, 1, 1);					// solver weight for matching target carapace force is always 1


				// gradient descent
				vector<float> action(num_rows);
				for(unsigned int i = 0; i < articulated_axes.size(); ++i)
				{
					ArticulatedAxis aa = articulated_axes[i];
					Vec3 v = aa.j->sjc->apply_torque;
					action[i] = ((float*)&v)[aa.axis];
				}
				GradientSearch(100, action, min_torques, max_torques, J, goal_values, weights);

				// apply selected action
				//stringstream ss;
				//ss << "selected action: { ";
				for(unsigned int i = 0; i < articulated_axes.size(); ++i)
				{
					ArticulatedAxis aa = articulated_axes[i];
					Vec3 v = aa.j->sjc->apply_torque;
					((float*)&v)[aa.axis] = action[i];
					aa.j->SetOrientedTorque(v);
					//if(i != 0)
					//	ss << ", ";
					//ss << action[i];
				}
				//ss << " }" << endl;
				//Debug(ss.str());


				// scoring
				float ori_error = 0.0f;
				float pos_error = 0.0f;
				float vel_error = 0.0f;
				float rot_error = 0.0f;

				float energy_cost = 0.0f;
				for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
					energy_cost += dood->all_joints[i]->actual.ComputeMagnitudeSquared();

				for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
				{
					const CBone& bone = *dood->all_bones[i];

					float bone_weight = 1.0f;
					if(bone.name == "carapace")
						bone_weight = 1.0f;
					else if(bone.name == "tail")
						bone_weight = 0.01f;
					else if(bone.name == "head")
						bone_weight = 0.1f;
					else if(bone.name[1] == ' ' && bone.name[8] == '2')
						bone_weight = 0.1f;
					else
						bone_weight = 0.0f;

					const RigidBody& rb = *bone.rb;

					Quaternion ori = rb.GetOrientation();
					
					ori_error += bone_weight * (Quaternion::Reverse(bone.initial_ori) * ori).ToRVec().ComputeMagnitudeSquared();

					Vec3 local_com = rb.GetLocalCoM();
					Vec3 initial_pos = bone.initial_pos + bone.initial_ori * local_com;
					Vec3 current_pos = rb.GetPosition() + ori * local_com;
					pos_error += bone_weight * (initial_pos - current_pos).ComputeMagnitudeSquared();

					vel_error += bone_weight * rb.GetLinearVelocity().ComputeMagnitudeSquared();
					rot_error += bone_weight * rb.GetAngularVelocity().ComputeMagnitudeSquared();
				}

				//energy_cost = max(0.0f, energy_cost - 500.0f);

				float ee_error = 0.0f;
				for(unsigned int i = 0; i < dood->feet.size(); ++i)
				{
					const Dood::FootState& fs = *dood->feet[i];
					const RigidBody& rb = *fs.body;
					Vec3 ee_pos = rb.GetPosition() + rb.GetOrientation() * fs.ee_pos;

					if(tick_age == 0)
						initial_ee.push_back(ee_pos);
					else
						ee_error += (ee_pos - initial_ee[i]).ComputeMagnitudeSquared();
				}

				vector<float> inst_scores;
				//inst_scores.push_back(pos_error * 1.0f);
				inst_scores.push_back(vel_error * 1.0f);
				//inst_scores.push_back(ori_error * 1.0f);
				//inst_scores.push_back(rot_error * 1.0f);
				//inst_scores.push_back(energy_cost * 0.00001f);
				inst_scores.push_back(ee_error * 1.0f);
				
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
		imp->carapace = GetBone("carapace");
		imp->head     = GetBone("head");
		imp->tail     = GetBone("tail");

		static const string sides[2] = { "l", "r" };
		static const string legs [3] = { "a", "b", "c" };
		static const string bones[3] = { "1", "2", "3" };

		for(int i = 0; i < 2; ++i)
		{
			CrabLeg* legs_array = i == 0 ? imp->llegs : imp->rlegs;
			for(int j = 0; j < 3; ++j)
				for(int k = 0; k < 3; ++k)
					legs_array[j].bones[k] = GetBone(((stringstream&)(stringstream() << sides[i] << " leg " << legs[j] << " " << bones[k])).str());
		}
	}

	void CrabBug::InitJointHelpers()
	{
		imp->neck  = GetJoint("head");
		imp->tailj = GetJoint("tail");

		for(int i = 0; i < 2; ++i)
		{
			CrabLeg* legs_array = i == 0 ? imp->llegs : imp->rlegs;
			for(int j = 0; j < 3; ++j)
				for(int k = 0; k < 3; ++k)
					legs_array[j].joints[k] = GetJoint(legs_array[j].bones[k]->name); 
		}

		//for(unsigned int i = 0; i < all_joints.size(); ++i)
		//	all_joints[i]->sjc->enable_motor = true;
	}
}
