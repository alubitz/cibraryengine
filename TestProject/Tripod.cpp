#include "StdAfx.h"
#include "Tripod.h"

#include "WalkPose.h"

#include "CBone.h"
#include "CJoint.h"

#include "GAExperiment.h"


#define MAX_TICK_AGE					60

namespace Test
{
	using namespace std;



	struct TripodLeg
	{
		CBone* bones[2];
		CJoint* joints[2];
	};

	/*
	* Tripod private implementation struct
	*/
	struct Tripod::Imp
	{
		bool init;
		bool clean_init;
		bool experiment_done;

		float score;
		vector<float> score_parts;

		GATrialToken ga_token;
		GAExperiment* experiment;

		Vec3 initial_pos;

		CBone* carapace;
		TripodLeg legs[3];

		vector<Vec3> initial_ee;

		float timestep, inv_timestep;

		unsigned int tick_age, max_tick_age;

		Imp() : init(false), experiment_done(false), experiment(nullptr), timestep(0), inv_timestep(0), tick_age(0), max_tick_age(MAX_TICK_AGE) { }
		~Imp() { }

		void Init(Tripod* dood)
		{
			dood->collision_group->SetInternalCollisionsEnabled(true);

			initial_pos = dood->pos;

			SharedInit(dood);
		}

		void ReInit(Tripod* dood)
		{
			float old_yaw = dood->yaw;
			float old_pitch = dood->pitch;
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

			dood->yaw = old_yaw;
			dood->pitch = old_pitch;
		}

		void SharedInit(Tripod* dood)
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

		void Update(Tripod* dood, const TimingInfo& time)
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
				// TODO: put your control code here


				// scoring
				float ori_error = 0.0f;
				float pos_error = 0.0f;
				float vel_error = 0.0f;
				float rot_error = 0.0f;
				float xyz_error[3] = { 0.0f, 0.0f, 0.0f };

				float energy_cost = 0.0f;
				for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
					energy_cost += dood->all_joints[i]->actual.ComputeMagnitudeSquared();

				for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
				{
					const CBone& bone = *dood->all_bones[i];

					float bone_weight = 1.0f;
					if(bone.name == "carapace")
						bone_weight = 1.0f;
					else
						bone_weight = 0.1f;

					const RigidBody& rb = *bone.rb;

					Quaternion ori = rb.GetOrientation();
					
					ori_error += bone_weight * (Quaternion::Reverse(bone.initial_ori) * ori).ToRVec().ComputeMagnitudeSquared();

					Vec3 local_com = rb.GetLocalCoM();
					Vec3 initial_pos = bone.initial_pos + bone.initial_ori * local_com;
					Vec3 current_pos = rb.GetPosition() + ori * local_com;
					pos_error += bone_weight * (initial_pos - current_pos).ComputeMagnitudeSquared();

					vel_error += bone_weight * rb.GetLinearVelocity().ComputeMagnitudeSquared();
					rot_error += bone_weight * rb.GetAngularVelocity().ComputeMagnitudeSquared();

					if(bone.name == "carapace")
					{
						Vec3 offset = initial_pos - current_pos;
						xyz_error[0] = offset.x * offset.x;
						xyz_error[1] = offset.y * offset.y;
						xyz_error[2] = offset.z * offset.z;
					}
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
				for(unsigned int i = 0; i < 3; ++i)
					inst_scores.push_back(xyz_error[i] * 1.0f);
				//inst_scores.push_back(ori_error * 1.0f);
				inst_scores.push_back(rot_error * 1.0f);
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
	};



	
	/*
	 * Tripod methods
	 */
	Tripod::Tripod(GameState* game_state, GAExperiment* experiment, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		imp(nullptr)
	{
		//yaw = Random3D::Rand(float(M_PI) * 2.0f);

		ragdoll_timer = 10.0f;

		standing_callback.angular_coeff = 0.0f;

		imp = new Imp();
		imp->experiment = experiment;
	}

	void Tripod::InnerDispose()
	{
		Dood::InnerDispose();

		if(imp) { delete imp; imp = nullptr; }
	}

	void Tripod::PreUpdatePoses(const TimingInfo& time) { imp->Update(this, time); }

	void Tripod::Update(const TimingInfo& time) { Dood::Update(time); }

	void Tripod::RegisterFeet()
	{
		// TODO: update this whenever you update tripod_zzp.lua
		static const float radius = 1.0f;

		string legs[3] = { "a", "b", "c" };
		for(unsigned int i = 0; i < 3; ++i)
		{
			float theta = float(i) * float(M_PI) * 2.0f / 3.0f;
			string bname = ((stringstream&)(stringstream() << "leg " << legs[i] << " 2")).str();
			feet.push_back(new FootState(Bone::string_table[bname], Vec3(radius * sinf(theta), 0.0f, radius * cosf(theta))));
		}
	}

	void Tripod::MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi)
	{
		cheaty_rot = Vec3();
		Dood::MaybeSinkCheatyVelocity(timestep, cheaty_vel, cheaty_rot, net_mass, net_moi);
	}

	void Tripod::InitBoneHelpers()
	{
		imp->carapace = GetBone("carapace");

		static const string legs [3] = { "a", "b", "c" };
		static const string bones[3] = { "1", "2", "3" };

		for(int j = 0; j < 3; ++j)
			for(int k = 0; k < 2; ++k)
				imp->legs[j].bones[k] = GetBone(((stringstream&)(stringstream() << "leg " << legs[j] << ' ' << bones[k])).str());
	}

	void Tripod::InitJointHelpers()
	{
		for(int j = 0; j < 3; ++j)
			for(int k = 0; k < 2; ++k)
				imp->legs[j].joints[k] = GetJoint(imp->legs[j].bones[k]->name);

		imp->legs[1].joints[1]->sjc->enable_motor = true;
		imp->legs[2].joints[1]->sjc->enable_motor = true;

		//for(unsigned int i = 0; i < all_joints.size(); ++i)
		//	all_joints[i]->sjc->enable_motor = true;
	}
}
