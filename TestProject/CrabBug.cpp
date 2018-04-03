#include "StdAfx.h"
#include "CrabBug.h"

#include "WalkPose.h"

#include "CBone.h"
#include "CJoint.h"

#include "GAExperiment.h"

#define DIE_AFTER_ONE_SECOND	0

#define ENABLE_WALK_ANIMATIONS	0

#define MAX_TICK_AGE			10

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

		GATrialToken ga_token;
		GAExperiment* experiment;

		Vec3 initial_pos;
		Vec3 initial_com;

		CBone carapace, head, tail;
		CJoint neck, tailj;

		CrabLeg llegs[3];
		CrabLeg rlegs[3];

		float pd_target[36];
		float pd_previous[36];
		float pd_integral[36];

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

			memset(pd_previous, 0, sizeof(pd_previous));

			if(experiment != nullptr && ga_token.candidate == nullptr)
				ga_token = experiment->GetNextTrial();

			for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
				dood->all_joints[i]->SetOrientedTorque(Vec3());
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
				// compute output torques
				EvaluateOpString(dood, ga_token.candidate->ops);

				// scoring
				float ori_error = 0.0f;
				float pos_error = 0.0f;
				float vel_error = 0.0f;
				float rot_error = 0.0f;

				for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
				{
					const CBone& bone = *dood->all_bones[i];
					const RigidBody& rb = *bone.rb;

					Quaternion ori = rb.GetOrientation();
					
					ori_error += (Quaternion::Reverse(bone.initial_ori) * ori).ToRVec().ComputeMagnitudeSquared();

					Vec3 local_com = rb.GetLocalCoM();
					Vec3 initial_pos = bone.initial_pos + bone.initial_ori * local_com;
					Vec3 current_pos = rb.GetPosition() + ori * local_com;
					pos_error += (initial_pos - current_pos).ComputeMagnitudeSquared();

					vel_error += rb.GetLinearVelocity().ComputeMagnitudeSquared();
					rot_error += rb.GetAngularVelocity().ComputeMagnitudeSquared();
				}

				score += ori_error * 0.1f;
				score += pos_error * 1.0f;
				score += vel_error * 1.0f;
				score += rot_error * 0.1f;

				/*Vec3 carapace_com = carapace.rb->GetPosition() + carapace.rb->GetOrientation() * carapace.rb->GetLocalCoM();
				if(tick_age == 0)
					initial_com = carapace_com;
				else if(tick_age + 1 == max_tick_age)
				{
					Vec3 delta = carapace_com - initial_com;
					score += Vec2::MagnitudeSquared(delta.x, delta.y) * 1000.0f;
					score += 1000.0f / max(0.1f, delta.z);
				}*/

				score += max(0, (signed)ga_token.candidate->ops.size() - 500) * 0.001f;
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
						experiment->TrialFinished(ga_token, score);
						experiment_done = true;

						ga_token = experiment->GetNextTrial();
					}
				}
			}
		}

		void PushVec3(vector<float>& a, const Vec3& value)
		{
			a.push_back(value.x);
			a.push_back(value.y);
			a.push_back(value.z);
		}

		void PushBone(vector<float>& a, const CBone& bone, const Vec3& com)
		{
			const RigidBody& rb = *bone.rb;

			Mat3 rm = rb.GetOrientation().ToMat3();
			PushVec3(a, rb.GetPosition() - com);
			PushVec3(a, rm * Vec3(1, 0, 0));
			PushVec3(a, rm * Vec3(0, 1, 0));
			PushVec3(a, rm * Vec3(0, 0, 1));
			PushVec3(a, rb.GetLinearVelocity());
			PushVec3(a, rb.GetAngularVelocity());
		}

		void PushJoint(vector<float>& a, const CJoint& joint, const Vec3& com)
		{
			const SkeletalJointConstraint& sjc = *joint.sjc;

			PushVec3(a, joint.GetRVec());
			PushVec3(a, sjc.ComputeAveragePosition() - com);
			PushVec3(a, sjc.net_impulse_angular);
			PushVec3(a, sjc.net_impulse_linear);
			PushVec3(a, sjc.min_extents);
			PushVec3(a, sjc.max_extents);
			PushVec3(a, sjc.min_torque);
			PushVec3(a, sjc.max_torque);
		}

		struct EvalScope
		{
			vector<float> strict_inputs;
			float locals[256];
			vector<float> strict_outputs;

			EvalScope() : strict_inputs(), strict_outputs() { memset(locals, 0, sizeof(locals)); }
		};

		void EvaluateOpString(CrabBug* dood, const string& ops)
		{
			static const unsigned int num_central_outputs = 6;
			static const unsigned int num_leg_outputs = 5;

			// some useful reference points
			Vec3 com = carapace.rb->GetPosition() + carapace.rb->GetOrientation() * carapace.rb->GetLocalCoM();

			// init strict inputs and declare number of strict outputs
			EvalScope leg_scopes[6];
			for(unsigned int i = 0; i < 6; ++i)
			{
				EvalScope& scope = leg_scopes[i];
				const CrabLeg& leg = (i % 2 == 0 ? llegs : rlegs)[i / 2];

				scope.strict_inputs.push_back(i % 2 == 0 ? 1.0f : -1.0f);
				scope.strict_inputs.push_back(i / 2 == 0 ? 1.0f : 0);
				scope.strict_inputs.push_back(i / 2 == 1 ? 1.0f : 0);
				scope.strict_inputs.push_back(i / 2 == 2 ? 1.0f : 0);

				for(unsigned int j = 0; j < 3; ++j)
				{
					PushBone(scope.strict_inputs, leg.bones[j], com);
					PushJoint(scope.strict_inputs, leg.joints[j], com);					
				}

				const Dood::FootState* fs = dood->feet[i];
				PushVec3(scope.strict_inputs, fs->net_force);
				PushVec3(scope.strict_inputs, fs->net_torque);
				//scope.strict_inputs.push_back((float)fs->contact_points.size());
				//Vec3 avg_n;
				//for(unsigned int j = 0; j < fs->contact_points.size(); ++j)
				//	avg_n += fs->contact_points[j].normal;
				//PushVec3(scope.strict_inputs, fs->contact_points.empty() ? Vec3() : avg_n / (float)fs->contact_points.size());

				scope.strict_outputs.resize(num_leg_outputs);
			}

			EvalScope central_scope;
			central_scope.strict_inputs.push_back(tick_age == 0 ? 1.0f : 0.0f);
			PushBone(central_scope.strict_inputs, head, com);
			PushBone(central_scope.strict_inputs, carapace, com);
			PushBone(central_scope.strict_inputs, tail, com);
			PushJoint(central_scope.strict_inputs, neck, com);
			PushJoint(central_scope.strict_inputs, tailj, com);
			central_scope.strict_inputs.push_back(com.y);

			central_scope.strict_outputs.resize(num_central_outputs);


			// evaluate
			for(unsigned int cindex = 0; cindex < ops.size(); ++cindex)
			{
				unsigned char c = ops[cindex++];
				if(cindex >= ops.size())
					break;
				if((c & 0x80) != 0)// && (c & 0x40) != 0)
				{
					bool write_central = (c & 0x20) != 0;
					bool write_output = (c & 0x10) != 0;
					unsigned char opcode = (unsigned char)(c & 0x3F) % 8;

					unsigned char write_index = ops[cindex++];
					if(cindex >= ops.size())
						break;
					if(write_output)
					{
						write_index %= 6;
						if(write_index >= (write_central ? num_central_outputs : num_leg_outputs))
							continue;
					}

					if(write_central)
					{
						float* outptr = write_output ? &central_scope.strict_outputs[write_index] : &central_scope.locals[write_index];
						*outptr = EvalOneOp(opcode, ops, cindex, central_scope, central_scope, leg_scopes);
					}
					else
					{
						unsigned int pindex = cindex;
						float newval[6];
						for(unsigned int i = 0; i < 6; ++i)
						{
							cindex = pindex;
							newval[i] = EvalOneOp(opcode, ops, cindex, leg_scopes[i], central_scope, leg_scopes);
						}
						for(unsigned int i = 0; i < 6; ++i)
						{
							EvalScope& scope = leg_scopes[i];
							float* outptr = write_output ? &scope.strict_outputs[write_index] : &scope.locals[write_index];
							*outptr = newval[i];
						}
					}
				}
			}


			// apply selected outputs
			for(unsigned int i = 0; i < 6; ++i)
			{
				const EvalScope& scope = leg_scopes[i];
				CrabLeg& leg = (i % 2 == 0 ? llegs : rlegs)[i / 2];

				leg.joints[0].SetOrientedTorque(Vec3(scope.strict_outputs[0], scope.strict_outputs[1], scope.strict_outputs[2]));
				leg.joints[1].SetOrientedTorque(Vec3(scope.strict_outputs[3], 0, 0));
				leg.joints[2].SetOrientedTorque(Vec3(scope.strict_outputs[4], 0, 0));
			}

			neck .SetOrientedTorque(Vec3(central_scope.strict_outputs[0], central_scope.strict_outputs[1], central_scope.strict_outputs[2]));
			tailj.SetOrientedTorque(Vec3(central_scope.strict_outputs[3], central_scope.strict_outputs[4], central_scope.strict_outputs[5]));

			//for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
			//	if(dood->all_joints[i]->actual.ComputeMagnitudeSquared() != 0.0f)
			//		Debug(((stringstream&)(stringstream() << "id = " << dood->imp->ga_token.candidate->id << ": " << dood->all_joints[i]->b->name << "; magsq = " << dood->all_joints[i]->actual.ComputeMagnitudeSquared() << endl)).str());
		}

		

		float EvalOneOp(unsigned char opcode, const string& ops, unsigned int& cindex, const EvalScope& local_scope, const EvalScope& central, const EvalScope legs[6])
		{
			if(opcode == 0)		// constant
			{
				unsigned short numerator = (unsigned short)ops[cindex++];
				numerator <<= 8;
				numerator |= (unsigned short)ops[cindex++];

				unsigned short denominator = (unsigned short)ops[cindex++];
				denominator <<= 8;
				denominator |= (unsigned short)ops[cindex++];

				if(cindex > ops.size())
					return 0.0f;
				
				return (float)(signed)numerator / ((float)denominator + 1);
			}

			float a = GetOneReference(ops, cindex, local_scope, central, legs);

			switch(opcode)
			{
				case 1:			// sum
					return a + GetOneReference(ops, cindex, local_scope, central, legs);
				case 2:			// difference
					return a - GetOneReference(ops, cindex, local_scope, central, legs);
				case 3:			// product
					return a * GetOneReference(ops, cindex, local_scope, central, legs);
				case 4:			// quotient
					return a / GetOneReference(ops, cindex, local_scope, central, legs);

				case 5:			// unary negate
					return -a;
				case 6:			// unary invert
					return 1.0f / a;

				case 7:			// sigmoid (and NaN-removal)
					if(a * a >= 0.0f)
						return tanhf(a);
					else
						return 0.0f;

				default:
					return 0.0f;
			}
		}

		float GetOneReference(const string& ops, unsigned int& cindex, const EvalScope& local_scope, const EvalScope& central, const EvalScope legs[6])
		{
			unsigned char reftype = ops[cindex++];

			if(cindex >= ops.size())
				return 0.0f;

			bool ref_central = (reftype & 0x80) != 0;
			bool ref_input = (reftype & 0x40) != 0;

			unsigned char ref_index = ops[cindex++];
			if(cindex >= ops.size())
				return 0.0f;

			if(ref_central)
				if(ref_input)
					return ref_index < central.strict_inputs.size() ? central.strict_inputs[ref_index] : 0.0f;
				else
					return central.locals[ref_index];
			else	
			{
				if(ref_input && ref_index >= legs[0].strict_inputs.size())
					return 0.0f;

				if(&local_scope == &central)		// when a central op references a leg variable, take the sum of all legs' values
				{
					float tot = 0.0f;
					if(ref_input)
						for(unsigned int i = 0; i < 6; ++i)
							tot += legs[i].strict_inputs[ref_index];
					else
						for(unsigned int i = 0; i < 6; ++i)
							tot += legs[i].locals[ref_index];
					return tot;
				}
				else								// but when a leg op references a leg variable, just use the local leg
				{
					if(ref_input)
						return local_scope.strict_inputs[ref_index];
					else
						return local_scope.locals[ref_index];
				}
			}
		}
	};



	
	/*
	 * CrabBug methods
	 */
	CrabBug::CrabBug(GameState* game_state, GAExperiment* experiment, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		imp(NULL),
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

		posey->active_poses.push_back(new WalkPose(this, &kr, &kw, &kw, &kw, &kw, NULL, NULL));
#endif

		standing_callback.angular_coeff = 0.0f;

		imp = new Imp();
		imp->experiment = experiment;
	}

	void CrabBug::InnerDispose()
	{
		Dood::InnerDispose();

		if(imp) { delete imp; imp = NULL; }
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

		float joint_strengths[3] = { 1400, 1200, 900 };
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
					float yz = j == 0 ? x : 0.0f;							// the 'knees' and 'ankes' can only rotate and torque on one axis

					RegisterJoint( legs_array[j].joints[k] = CJoint( this, k == 0 ? imp->carapace : legs_array[j].bones[k - 1], legs_array[j].bones[k], x, yz, yz) ); 
				}
		}

		//for(unsigned int i = 0; i < all_joints.size(); ++i)
		//	all_joints[i]->sjc->enable_motor = true;
	}
}
