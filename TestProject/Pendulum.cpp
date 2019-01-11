#include "StdAfx.h"
#include "Pendulum.h"

#include "WalkPose.h"

#include "CBone.h"
#include "CJoint.h"

#include "GAExperiment.h"

#define INITIAL_PUSH_TEST				0

#define MAX_TICK_AGE					300

using namespace Test;
using namespace std;

/**
 * Describes a generic kinematic chain
 */
struct KChain {
	vector<CBone*>  bones;
	vector<CJoint*> joints;
};

/**
 * Describes the implementation of a Pendulum.
 */
class Pendulum::Imp {
public:
	bool init;

	bool experiment_done;

	float score;
	vector<float> score_parts;

	GATrialToken ga_token;
	GAExperiment* experiment;

	Vec3 initial_pos;

	CBone* carapace;
	KChain legs;

	vector<Vec3> initial_ee;

	float timestep;
	float inv_timestep;

	unsigned int tick_age;
	unsigned int max_tick_age;

public:
	Imp() :
		init(false), experiment_done(false), experiment(nullptr), timestep(0), inv_timestep(0), tick_age(0), max_tick_age(MAX_TICK_AGE)
	{};

	~Imp() {};

	void Init(Pendulum* dood);

	void ReInit(Pendulum* dood);

	void PrePhysicsStep(Pendulum* dood, float timestep);
	void Update(Pendulum* dood, const TimingInfo& time);

	Vec3 GetForceCorrection(CJoint& joint, CJoint& j2, float inv_timestep, float fraction);

protected:
	// Init process shared by Init() and ReInit()
	void SharedInit(Pendulum* dood);
};



void Pendulum::Imp::Init(Pendulum* dood)
{
	dood->collision_group->SetInternalCollisionsEnabled(false);

	initial_pos = dood->pos;

	SharedInit(dood);
}

void Pendulum::Imp::ReInit(Pendulum* dood)
{
	float old_yaw = dood->yaw;
	float old_pitch = dood->pitch;
	dood->yaw = dood->pitch = 0.0f;
	dood->pos = initial_pos;

	dood->DoInitialPose();
	dood->posey->skeleton->InvalidateCachedBoneXforms();

	for (unsigned int i = 0; i < dood->all_bones.size(); ++i)
	{
		CBone& cb = *dood->all_bones[i];
		RigidBody& rb = *cb.rb;

		Bone* posey = cb.posey;
		posey->GetTransformationMatrix().Decompose(cb.initial_pos, cb.initial_ori);		// re-computes bones' initial states

		rb.SetPosition(cb.initial_pos);
		rb.SetOrientation(cb.initial_ori);
		rb.SetLinearVelocity(Vec3());
		rb.SetAngularVelocity(Vec3());
	}

	SharedInit(dood);

	dood->yaw = old_yaw;
	dood->pitch = old_pitch;
}

void Pendulum::Imp::SharedInit(Pendulum* dood)
{
	experiment_done = false;
	tick_age = 0;

	score = 0.0f;
	score_parts.clear();

	if (experiment != nullptr && ga_token.candidate == nullptr)
		ga_token = experiment->GetNextTrial();

	for (unsigned int i = 0; i < dood->all_joints.size(); ++i)
		dood->all_joints[i]->SetOrientedTorque(Vec3());

	initial_ee.clear();

#if INITIAL_PUSH_TEST
	dood->all_bones[0]->rb->SetLinearVelocity(Vec3(5.0, 0.0, 0.0));    // Nudge the carapace in one direction
#endif
}

void Pendulum::Imp::PrePhysicsStep(Pendulum* dood, float timestep)
{
	if (!init)
	{
		Init(dood);
		init = true;
	}
	else if (experiment_done || experiment != nullptr && ga_token.candidate == nullptr)
		ReInit(dood);
}

void Pendulum::Imp::Update(Pendulum* dood, const TimingInfo& time)
{
	if(experiment_done || experiment != nullptr && ga_token.candidate == nullptr)
		return;

	timestep = time.elapsed;
	inv_timestep = 1.0f / timestep;

	// reset all the joints and bones
	int ji = 0;
	for (unsigned int i = 0; i < dood->all_joints.size(); ++i)
	{
		CJoint& joint = *dood->all_joints[i];
		joint.Reset();
		if (tick_age == 0)
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
	for (unsigned int i = 0; i < dood->all_bones.size(); ++i)
		total_mass += dood->all_bones[i]->rb->GetMass();

	//for (unsigned int i = 0; i < dood->all_bones.size(); ++i)
	//	dood->all_bones[i]->ComputeDesiredTorqueWithDefaultMoI(dood->all_bones[i]->initial_ori, inv_timestep);

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

	if (experiment != nullptr && ga_token.candidate != nullptr && !ga_token.candidate->aborting)
	{

		stringstream ss;
		ss << "tick age = " << tick_age << endl;
		vector<Vec3> dts;
		for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
		{
			CBone& bone = *dood->all_bones[i];
			if(bone.rb->GetMass() != 0.0f)
			{
				bone.ComputeDesiredTorqueWithDefaultMoIAndPosey(inv_timestep);
				dts.push_back(bone.desired_torque);

				bone.desired_torque = Vec3();		// TODO: remove this garbage
			}
		}

		vector<Vec3> fcs(legs.bones.size());
		for(unsigned int i = 1; i < legs.joints.size(); ++i)
			fcs[i] = GetForceCorrection(*legs.joints[i - 1], *legs.joints[i], inv_timestep, 1.0f);


		for(unsigned int i = 0; i < dood->all_bones.size(); ++i)
		{
			CBone& bone = *dood->all_bones[i];
			if(bone.rb->GetMass() != 0.0f)
			{
				Vec3 rot = bone.rb->GetAngularVelocity();
				Vec3 dt = dts[i];
				Vec3 fc = fcs[i];
				ss << "\tbone " << i << endl;
				ss << "\t\trot = (" << rot.x << ", " << rot.y << ", " << rot.z << ")" << endl;
				ss << "\t\tangular = (" << dt.x << ", " << dt.y << ", " << dt.z << ")" << endl;
				ss << "\t\tlinear  = (" << fc.x << ", " << fc.y << ", " << fc.z << ")" << endl;
			}
		}

		for(unsigned int i = 0; i < legs.joints.size(); ++i)
		{
			CJoint& joint = *legs.joints[i];
			const SkeletalJointConstraint& sjc = *legs.joints[i]->sjc;

#if 1
			Vec3 full_torque = dts[i] + fcs[i];		// TODO: this isn't good enough
#else
			Vec3 full_torque = fcs[i];
#endif

			Vec3 use_torque = joint.oriented_axes * full_torque;
			const float* minp = (float*)&sjc.min_torque;
			const float* maxp = (float*)&sjc.max_torque;
			for(float *tptr = (float*)&use_torque, *tend = tptr + 3; tptr != tend; ++tptr, ++minp, ++maxp)
			{
				if(*tptr > *maxp)
				{
					*tptr = 0.5f * *maxp;	// expect to spend half the time accelerating to the desired speed, and the remaining half decelerating
				}
				else if(*tptr < *minp)
				{
					*tptr = 0.5f * *minp;
				}
			}
			joint.a->desired_torque = joint.oriented_axes.TransposedMultiply(use_torque);
		}

		for(unsigned int i = 0; i < dood->all_joints.size(); ++i)
		{
			dood->all_joints[i]->SetTorqueToSatisfyA();

			Vec3 act = dood->all_joints[i]->actual;
			ss << "\tjoint " << i << " actual = (" << act.x << ", " << act.y << ", " << act.z << ")" << endl;
		}

		Debug(ss.str());


		// scoring
		float ori_error = 0.0f;
		float pos_error = 0.0f;
		float vel_error = 0.0f;
		float rot_error = 0.0f;
		float xyz_error[3] = { 0.0f, 0.0f, 0.0f };

		float energy_cost = 0.0f;
		for (unsigned int i = 0; i < dood->all_joints.size(); ++i)
			energy_cost += dood->all_joints[i]->actual.ComputeMagnitudeSquared();

		for (unsigned int i = 0; i < dood->all_bones.size(); ++i)
		{
			const CBone& bone = *dood->all_bones[i];

			float bone_weight = 1.0f;
			if (bone.name == "carapace")
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

			if (bone.name == "carapace")
			{
				Vec3 offset = initial_pos - current_pos;
				xyz_error[0] = offset.x * offset.x;
				xyz_error[1] = offset.y * offset.y;
				xyz_error[2] = offset.z * offset.z;
			}
		}

		//energy_cost = max(0.0f, energy_cost - 500.0f);

		float ee_error = 0.0f;
		for (unsigned int i = 0; i < dood->feet.size(); ++i)
		{
			const Dood::FootState& fs = *dood->feet[i];
			const RigidBody& rb = *fs.body;
			Vec3 ee_pos = rb.GetPosition() + rb.GetOrientation() * fs.ee_pos;

			if (tick_age == 0)
				initial_ee.push_back(ee_pos);
			else
				ee_error += (ee_pos - initial_ee[i]).ComputeMagnitudeSquared();
		}

		vector<float> inst_scores;
		//inst_scores.push_back(pos_error * 1.0f);
		inst_scores.push_back(vel_error * 1.0f);
		for (unsigned int i = 0; i < 3; ++i)
			inst_scores.push_back(xyz_error[i] * 1.0f);
		//inst_scores.push_back(ori_error * 1.0f);
		inst_scores.push_back(rot_error * 1.0f);
		//inst_scores.push_back(energy_cost * 0.00001f);
		inst_scores.push_back(ee_error * 1.0f);

		if (score_parts.empty())
			score_parts.resize(inst_scores.size());
		else
			assert(score_parts.size() == inst_scores.size());

		for (unsigned int i = 0; i < score_parts.size(); ++i)
		{
			score_parts[i] += inst_scores[i];
			score += inst_scores[i];
		}
	}

	++tick_age;

	if (experiment != nullptr && ga_token.candidate != nullptr)
	{
		float fail_threshold = experiment->GetEarlyFailThreshold();
		if (fail_threshold >= 0.0f && score + ga_token.candidate->score >= fail_threshold)
			ga_token.candidate->aborting = true;

		if (tick_age >= max_tick_age || ga_token.candidate->aborting)
		{
			if (!experiment_done)
			{
				experiment->TrialFinished(ga_token, score, score_parts, tick_age);
				experiment_done = true;

				ga_token = experiment->GetNextTrial();
			}
		}
	}
}

Vec3 Pendulum::Imp::GetForceCorrection(CJoint& joint, CJoint& j2, float inv_timestep, float fraction)
{
	const CBone& a = *joint.a;
	const RigidBody& rb = *a.rb;
	float mass = rb.GetMass();

	CBone& b = *j2.a;
	if(mass != 0.0f && b.rb->GetMass() != 0.0f)
	{
		Vec3 local_com = rb.GetLocalCoM();

		Vec3 current_x = rb.GetOrientation() * local_com + rb.GetPosition();
		Vec3 desired_x = a.posey->GetTransformationMatrix().TransformVec3_1(local_com);

		Vec3 desired_v = (desired_x - current_x) * inv_timestep;
		//float dvmagsq = desired_v.ComputeMagnitudeSquared() * 1.0f;//0625f;
		//if(dvmagsq > 1.0f)
		//	desired_v /= sqrtf(dvmagsq);


		Vec3 current_v = rb.GetLinearVelocity();

		Vec3 desired_f = (desired_v - current_v) * (inv_timestep * mass);
		desired_f += rb.GetGravity() * mass;
		desired_f *= fraction;

		Vec3 radius = current_x - j2.sjc->ComputeAveragePosition();

		Vec3 full_torque = Vec3::Cross(desired_f, radius);
		return full_torque;

		//float torque_approx = (j2.sjc->max_torque - j2.sjc->min_torque).ComputeMagnitude() * 0.28867513459481288225457439025098f;//* 0.5f;
		//
		//float frac = full_torque.ComputeMagnitude() / torque_approx;
		//if(frac <= 1.0f)
		//	return full_torque;
		//else
		//	return full_torque * (0.5f / frac);			// expect to spend half the time accelerating to the desired speed, and the remaining half decelerating
	}
	else
		return Vec3();
}


void Pendulum::InitBoneHelpers()
{
	Dood::InitBoneHelpers();
	
	// Add carapace
	imp->carapace = GetBone("carapace");

	// Add bones in the leg
	int n_bones = all_bones.size() - 1;
	imp->legs.bones.resize(n_bones, nullptr);
	for (int i = 0; i < (n_bones - 1); ++i) {
		imp->legs.bones[i] = GetBone(((stringstream&)(stringstream() << "leg a " << i + 1)).str());
	}

	// Add foot
	imp->legs.bones[n_bones - 1] = GetBone("foot");

	// Check if all valid
	assert(imp->carapace != nullptr);

	for (auto bone : imp->legs.bones) {
		assert(bone != nullptr);
	}
}

void Pendulum::InitJointHelpers()
{
	Dood::InitJointHelpers();

	auto n_bones = imp->legs.bones.size();
	imp->legs.joints.resize(n_bones);	// Able to do this because carapace is seperate from the legs

	for (auto i = 0u; i < n_bones; ++i) {
		imp->legs.joints[i] = GetJoint(imp->legs.bones[i]->name);
	}

	// Check if all valid
	for (auto joint : imp->legs.joints) {
		assert(joint != nullptr);
	}

	//for(unsigned int i = 0; i < all_joints.size(); ++i)
	//	all_joints[i]->sjc->enable_motor = true;
}

void Pendulum::InnerDispose()
{
	Dood::InnerDispose();

	if (imp) {
		delete imp;
		imp = nullptr;
	}
}

void Pendulum::MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi)
{
	cheaty_rot = Vec3();
	Dood::MaybeSinkCheatyVelocity(timestep, cheaty_vel, cheaty_rot, net_mass, net_moi);
}

Pendulum::Pendulum(GameState* game_state, GAExperiment* experiment, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
	Dood(game_state, model, mphys, pos, team),
	imp(nullptr)
{
	//yaw = Random3D::Rand(float(M_PI) * 2.0f);

	ragdoll_timer = 10.0f;

	standing_callback.angular_coeff = 0.0f;

	imp = new Imp();
	imp->experiment = experiment;
}

void Pendulum::PreUpdatePoses(const TimingInfo& time) { imp->Update(this, time); }

void Pendulum::RegisterFeet()
{
	// A pendulum only has one "foot"
	string bname = "foot";
	feet.push_back(new FootState(Bone::string_table[bname], Vec3(0.0f, 0.0f, 0.0f)));
}

void Pendulum::Update(const TimingInfo& time) { Dood::Update(time); }

void Pendulum::PrePhysicsStep(float timestep) { imp->PrePhysicsStep(this, timestep); }

void Pendulum::DoInitialPose()
{
	Dood::DoInitialPose();

#if !INITIAL_PUSH_TEST
	float amount = 0.25f * float(M_PI);

#if 0
	string bone_names[3] = { "leg a 2", "leg a 3", "foot" };
#else
	string bone_names[3] = { "leg a 1", "leg a 2", "leg a 3" };
#endif

	float derp = 0.00001f;//0.0000000001f;//amount * 0.95f;
	
	//posey->skeleton->GetNamedBone(bone_names[0])->ori = Quaternion::FromRVec( derp, 0, amount        );
	//posey->skeleton->GetNamedBone(bone_names[1])->ori = Quaternion::FromRVec(-derp, 0, amount * -2.0f);
	//posey->skeleton->GetNamedBone(bone_names[2])->ori = Quaternion::FromRVec( 0,    0, amount        );

	posey->skeleton->GetNamedBone(bone_names[0])->ori = Quaternion::FromRVec( 0, 0, float(M_PI_2));
	//posey->skeleton->GetNamedBone(bone_names[1])->ori = Quaternion::FromRVec( 0, 0, 0);
	//posey->skeleton->GetNamedBone(bone_names[2])->ori = Quaternion::FromRVec( 0, 0, 0);

	//posey->skeleton->GetNamedBone(bone_names[0])->ori = Quaternion::FromRVec(amount        , 0, 0);
	//posey->skeleton->GetNamedBone(bone_names[1])->ori = Quaternion::FromRVec(amount * -2.0f, 0, 0);
	//posey->skeleton->GetNamedBone(bone_names[2])->ori = Quaternion::FromRVec(amount        , 0, 0);
#endif
}
