#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#define PROFILE_CPHFT					1
#define PROFILE_QP_SOLVER				1
#define PROFILE_SCORE_PROPOSED_F		0

#define PROFILE_ANY_CPHFT				(PROFILE_CPHFT || PROFILE_QP_SOLVER || PROFILE_SCORE_PROPOSED_F)

#define DEBUG_GRADIENT_SEARCH_PROGRESS	0

#define DIE_AFTER_ONE_SECOND			0

#define ENABLE_NEW_JETPACKING			0


#define MAX_TICK_AGE					2


#define NUM_LOWER_BODY_BONES			9
#define NUM_LB_BONE_FLOATS				(NUM_LOWER_BODY_BONES * 6)

#define NUM_LEG_JOINTS					8
#define NUM_JOINT_AXES					(3 * NUM_LEG_JOINTS)

#define GENERATE_SUBTEST_LIST_EVERY		2		// 1 = game state; 2 = generation; 3 = candidate

#define NUM_SUBTESTS					10
#define TRIALS_PER_SUBTEST				1
#define NUM_TRIALS						(NUM_SUBTESTS * TRIALS_PER_SUBTEST)

#define NUM_ELITES						32
#define MUTANTS_PER_ELITE				0
#define CROSSOVERS_PER_PAIR				2

#define MUTATION_SCALE					0.01f
#define MUTATION_COUNT					10000

namespace Test
{
	/*
	 * Soldier constants
	 */
	static const float jump_speed         = 4.0f;

	static const float fly_accel_up       = 15.0f;
	static const float fly_accel_lateral  = 8.0f;

	static const float fuel_spend_rate    = 0.5f;
	static const float fuel_refill_rate   = 0.4f;
	static const float jump_to_fly_delay  = 0.3f;

	static const float torso2_yaw_offset  = 0.5f;


#if PROFILE_ANY_CPHFT
#if PROFILE_CPHFT
	static float timer_init					= 0.0f;
	static float timer_reset				= 0.0f;
	static float timer_massinfo				= 0.0f;
	static float timer_ub_stuff				= 0.0f;
	static float timer_a_matrix				= 0.0f;
	static float timer_solve_qp				= 0.0f;

	static float timer_use_results			= 0.0f;
	static float timer_scoring				= 0.0f;
	static float timer_end_of_test			= 0.0f;
	static float timer_cphft_total			= 0.0f;

	static unsigned int counter_cphft		= 0;
#endif

#if PROFILE_QP_SOLVER
	static float timer_jt_constraints		= 0.0f;
	static float timer_constraint_mags		= 0.0f;
	static float timer_gradient_descent     = 0.0f;
	static float timer_qp_total				= 0.0f;

	static unsigned int counter_qp			= 0;
#endif

#if PROFILE_SCORE_PROPOSED_F
	static float timer_score_proposed_f		= 0.0f;
	static unsigned int counter_score_f		= 0;
#endif

	static void DebugCPHFTProfilingData()
	{
#if PROFILE_QP_SOLVER
			Debug(((stringstream&)(stringstream() << "total for " << counter_qp << " calls to Soldier::Imp::SolveForOptimalF = " << timer_qp_total << endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "jt_constraints =\t\t"		<< timer_jt_constraints		<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "constraint_mags =\t\t"	<< timer_constraint_mags	<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "gradient_descent =\t\t"	<< timer_gradient_descent	<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_jt_constraints + timer_constraint_mags + timer_gradient_descent << endl)).str());
#endif

#if PROFILE_SCORE_PROPOSED_F
			Debug(((stringstream&)(stringstream() << "total for " << counter_score_f << " calls to Soldier::Imp::ScoreProposedF = " << timer_score_proposed_f << endl)).str());
#endif

#if PROFILE_CPHFT
			Debug(((stringstream&)(stringstream() << "total for " << counter_cphft << " calls to Soldier::Imp::Update = " << timer_cphft_total << endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "init =\t\t\t\t\t"			<< timer_init				<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "reset =\t\t\t\t\t"		<< timer_reset				<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "massinfo =\t\t\t\t"		<< timer_massinfo			<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "ub_stuff =\t\t\t\t"		<< timer_ub_stuff			<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "a_matrix =\t\t\t\t"		<< timer_a_matrix			<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "solve_qp =\t\t\t\t"		<< timer_solve_qp			<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "use_results =\t\t\t"		<< timer_use_results		<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "scoring =\t\t\t\t"		<< timer_scoring			<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "end_of_test =\t\t\t"		<< timer_end_of_test		<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<<
				timer_init + timer_reset + timer_massinfo + timer_ub_stuff + timer_a_matrix +
				timer_solve_qp + timer_use_results + timer_scoring + timer_end_of_test << endl)).str());
#endif
	}
#endif

	struct WalkKeyframe
	{
		Vec3 desired_oris[NUM_LOWER_BODY_BONES];
		float duration;

		WalkKeyframe() : duration(0) { }
		WalkKeyframe(float duration) : duration(duration) { }

		static WalkKeyframe Interpolate(const WalkKeyframe& a, const WalkKeyframe& b, float tfrac)
		{
			float bfrac = max(0.0f, min(1.0f, tfrac));
			float afrac = 1.0f - bfrac;

			WalkKeyframe result;
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
				result.desired_oris[i] = a.desired_oris[i] * afrac + b.desired_oris[i] * bfrac;
			result.duration = a.duration;								// no interpolation for this component
			return result;
		}
	};
	static vector<WalkKeyframe> MakeWalkKeyframes()
	{
		vector<WalkKeyframe> result;

		WalkKeyframe blah(30.0f);
		result.push_back(blah);

		/*
		WalkKeyframe rfootbalance(30.0f);
		rfootbalance.desired_oris[0] = Vec3( 0.1f, 0, 0     );
		rfootbalance.desired_oris[1] = Vec3( 0.4f, 0, 0     );
		rfootbalance.desired_oris[2] = Vec3( 0.7f, 0, 0     );
		rfootbalance.desired_oris[3] = Vec3(-1,    0, 0     );

		rfootbalance.desired_oris[5] = Vec3( 0,    0, 0.15f );
		rfootbalance.desired_oris[6] = Vec3( 0,    0, 0.15f );
		


		result.push_back(rfootbalance);
		*/

		/*
		float A = 0.2f;
		float B = 0.5f;

		float dur = 1.0f;

		WalkKeyframe lfootfront(dur);
		lfootfront.desired_oris[2] = Vec3(-A, 0, 0);
		lfootfront.desired_oris[3] = Vec3(-A, 0, 0);
		lfootfront.desired_oris[5] = Vec3( A, 0, 0);
		lfootfront.desired_oris[6] = Vec3( A, 0, 0);
		result.push_back(lfootfront);

		WalkKeyframe rfootfloat(dur);
		rfootfloat.desired_oris[6] = Vec3(-B, 0, 0);
		result.push_back(rfootfloat);

		WalkKeyframe rfootfront(dur);
		rfootfront.desired_oris[2] = Vec3( A, 0, 0);
		rfootfront.desired_oris[3] = Vec3( A, 0, 0);
		rfootfront.desired_oris[5] = Vec3(-A, 0, 0);
		rfootfront.desired_oris[6] = Vec3(-A, 0, 0);
		result.push_back(rfootfront);

		WalkKeyframe lfootfloat(dur);
		lfootfloat.desired_oris[2] = Vec3(-B, 0, 0);
		result.push_back(lfootfloat);
		*/

		return result;
	}
	static vector<WalkKeyframe> walk_keyframes = MakeWalkKeyframes();

	struct FrameParams
	{
		static const unsigned int num_params;
		static const FrameParams kmin;
		static const FrameParams kmax;

		// don't add any members that aren't floats!
		float qp_bone_weights_angular[NUM_LOWER_BODY_BONES];
		float qp_bone_weights_linear[NUM_LOWER_BODY_BONES + 1];

		Vec3 xprod_radii[NUM_LOWER_BODY_BONES];
		float xprod_frac[NUM_LOWER_BODY_BONES * (NUM_LOWER_BODY_BONES + 1)];

		float gravity_coeff[NUM_LOWER_BODY_BONES];
		
		//float foot_torque_coeff[4];
		//float foot_force_coeff[4];
		//float ground_foott_subfrac[4];
		//float ground_netf_subfrac[4];

		float prediction_error_pcoeff[NUM_LOWER_BODY_BONES];		// PID-ish
		float prediction_error_icoeff[NUM_LOWER_BODY_BONES];

		float hmax;

		float&       operator[](unsigned int index)       { assert(index < num_params); return *(qp_bone_weights_angular + index); }
		const float& operator[](unsigned int index) const { assert(index < num_params); return *(qp_bone_weights_angular + index); }

		static FrameParams InitMin()
		{
			FrameParams k;
			memset(k.qp_bone_weights_angular, 0, sizeof(FrameParams));

			Mat3 m;
			for(unsigned int i = 0; i < 9; ++i)
				m[i] = -1.0f;
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
			{
				k.qp_bone_weights_angular[i] = -2.0f;
				k.qp_bone_weights_linear[i]  = -2.0f;
				//k.prediction_error_pcoeff[i] = 0.0f;
				//k.prediction_error_icoeff[i] = 0.0f;

				if(i == 4)
					k.xprod_radii[i] = Vec3(0.05f, 0, 0);
				else if(i <= 1 || i >= 7)
					k.xprod_radii[i] = true ? Vec3(-0.1f, -0.1f, 0) : Vec3();
				else
					k.xprod_radii[i] = true ? Vec3(-0.1f, 0.35f, -0.1f) : Vec3(0, 0.35f, 0);

				k.gravity_coeff[i] = 0.0f;//i <= 1 || i >= 7 || i == 4 ? 1.0f : 0.0f;
			}
			k.qp_bone_weights_linear[NUM_LOWER_BODY_BONES] = 0;//-2.0f;

			k.qp_bone_weights_angular[2] = k.qp_bone_weights_angular[3] = k.qp_bone_weights_angular[5] = k.qp_bone_weights_angular[6] = -200;
			k.qp_bone_weights_angular[4] = 0;

			//k.qp_bone_weights_linear[2] = k.qp_bone_weights_linear[3] = k.qp_bone_weights_linear[5] = k.qp_bone_weights_linear[6] = -200;
			k.qp_bone_weights_linear[4] = -200;

			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES * (NUM_LOWER_BODY_BONES + 1); ++i)
			{
				unsigned int j = i % NUM_LOWER_BODY_BONES;
				//if(j <= 1 || j >= 7)
					k.xprod_frac[i] = -1.0f;
			}

			for(unsigned int i = 0; i < 4; ++i)
			{
				//k.foot_torque_coeff[i] = 0.0f;//0.5f;
				//k.foot_force_coeff[i] = 1;//0.0f;//0.5f;
				//k.ground_foott_subfrac[i] = 0.0f;
				//k.ground_netf_subfrac[i] = 0;//-1.0f;
			}

			//k.hmax = -100.0f;

			return k;
		}
		static FrameParams InitMax()
		{
			FrameParams k;
			memset(k.qp_bone_weights_angular, 0, sizeof(FrameParams));

			Mat3 m;
			for(unsigned int i = 0; i < 9; ++i)
				m[i] = 1.0f;
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
			{
				k.qp_bone_weights_angular[i] = 2.0f;
				k.qp_bone_weights_linear[i]  = 2.0f;
				//k.prediction_error_pcoeff[i] = 1.0f;
				//k.prediction_error_icoeff[i] = 1.0f;

				if(i == 4)
					k.xprod_radii[i] = Vec3(0.2f, 0, 0);
				else if(i <= 1 || i >= 7)
					k.xprod_radii[i] = true ? Vec3(0.1f, 0.1f, 0.25f) : Vec3(0, 0, 0.25f);
				else
					k.xprod_radii[i] = true ? Vec3(0.1f, 0.65f, 0.1f) : Vec3(0, 0.65f, 0);

				k.gravity_coeff[i] = 1.0f;//i <= 1 || i >= 7 || i == 4 ? 1.0f : 0.0f;
			}

			k.qp_bone_weights_angular[2] = k.qp_bone_weights_angular[3] = k.qp_bone_weights_angular[5] = k.qp_bone_weights_angular[6] = -200;

			k.qp_bone_weights_linear[NUM_LOWER_BODY_BONES] = 2.0f;

			//k.qp_bone_weights_linear[2] = k.qp_bone_weights_linear[3] = k.qp_bone_weights_linear[5] = k.qp_bone_weights_linear[6] = -200;
			k.qp_bone_weights_linear[2] = k.qp_bone_weights_linear[3] = k.qp_bone_weights_linear[5] = k.qp_bone_weights_linear[6] = 0.0f;
			k.qp_bone_weights_linear[4] = -200;

			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES * (NUM_LOWER_BODY_BONES + 1); ++i)
			{
				unsigned int j = i % NUM_LOWER_BODY_BONES;
				//if(j <= 1 || j >= 7)
					k.xprod_frac[i] = 1.0f;
			}

			for(unsigned int i = 0; i < 4; ++i)
			{
				//k.foot_torque_coeff[i] = 0;//1.0f;//1.2f;
				//k.foot_force_coeff[i] = 1.0f;//1.2f;
				//k.ground_foott_subfrac[i] = 0;//1.0f;
				//k.ground_netf_subfrac[i] = 0;//1.0f;
			}

			//k.hmax = 2000.0f;

			return k;
		}

		static FrameParams Interpolate(const FrameParams& a, const FrameParams& b, float tfrac)
		{
			float bfrac = max(0.0f, min(1.0f, tfrac));
			float afrac = 1.0f - bfrac;

			FrameParams result;
			for(unsigned int i = 0; i < num_params; ++i)
				result[i] = a[i] * afrac + b[i] * bfrac;
			return result;
		}
	};
	const unsigned int FrameParams::num_params = sizeof(FrameParams) / sizeof(float);

	const FrameParams FrameParams::kmin = FrameParams::InitMin();
	const FrameParams FrameParams::kmax = FrameParams::InitMax();

	struct Scores
	{
		static const unsigned int num_floats;

		float bone_ori[NUM_LOWER_BODY_BONES];
		float dummy1;
		float bone_pos[NUM_LOWER_BODY_BONES];
		float dummy2;
		float head_ori;
		float gun_aimvec;
		float com;
		float energy_cost;
		float prediction_error;
		float helper_force_penalty;

		float total;

		Scores() { memset(Begin(), 0, sizeof(float) * num_floats); }

		float* Begin() { return bone_ori; }
		float* End()   { return bone_ori + num_floats; }
		const float* Begin() const { return bone_ori; }
		const float* End()   const { return bone_ori + num_floats; }

		void operator += (const Scores& other)
		{
			const float *bptr = other.Begin(), *bend = other.End();
			for(float* aptr = Begin(); bptr != bend; ++aptr, ++bptr)
				*aptr += *bptr;
		}
		Scores operator + (const Scores& other) { Scores result = *this; result += other; return result; }

		void operator *= (float f)
		{
			for(float *aptr = Begin(), *aend = End(); aptr != aend; ++aptr)
				*aptr *= f;
		}
		Scores operator * (float f) { Scores result = *this; result *= f; return result; }

		void operator /= (float f)
		{
			for(float *aptr = Begin(), *aend = End(); aptr != aend; ++aptr)
				*aptr /= f;
		}
		Scores operator / (float f) { Scores result = *this; result /= f; return result; }

		// compute the total, assign it to the variable named total, and return its value
		float ComputeTotal()
		{
			total = 0.0f;
			for(float *aptr = Begin(); aptr != &total; ++aptr)
				total += *aptr;
			return total;
		}

		void ApplyScaleAndClamp(const Scores& scale)
		{
			const float *bptr = scale.Begin();
			for(float* aptr = Begin(); aptr != &total; ++aptr, ++bptr)
			{
				//float s = (1.0f - expf(-*aptr * *bptr));
				//if(_isnan(s))
				//	s = 1.0f;
				*aptr = *aptr * *bptr;//s;
			}
		}
	};
	const unsigned int Scores::num_floats = sizeof(Scores) / sizeof(float);

	struct GABrain
	{
		vector<FrameParams> frames;

		Scores scores;

		unsigned int id, pa, pb;			// my id, and my parents' ids

		GABrain(unsigned int id, unsigned int pa, unsigned int pb) : scores(), id(id), pa(pa), pb(pb) { SetZero(); }
		GABrain() : scores() { SetZero(); }
		~GABrain() { }

		void SetZero()
		{ 
			frames.clear();
			for(unsigned int i = 0, imax = walk_keyframes.size(); i < imax; ++i)
			{
				FrameParams f;
				memset(&f, 0, sizeof(FrameParams));
				frames.push_back(f);
			}
		}

		string GetText() const
		{
			stringstream ss;
			ss << "(id " << id << " p " << pa << ", " << pb << ") " << scores.total << " { ";
			for(const float *first = scores.Begin(), *sptr = first; sptr != &scores.total; ++sptr)
				ss << (sptr == first ? "" : ", ") << *sptr;
			ss << " }";

			return ss.str();
		}

		GABrain* CreateClone(unsigned int newid)
		{
			GABrain* result = new GABrain(newid, id, id);
			result->frames.resize(frames.size());
			for(unsigned int i = 0; i < frames.size(); ++i)
				result->frames[i] = frames[i];
			return result;
		}

		void Randomize(float scale, unsigned int count)
		{
#if 0
			for(unsigned int i = 0; i < frames.size(); ++i)
			{
				FrameParams& k = frames[i];

				FrameParams best_delta;
				float best_dsq = -1;
				for(unsigned int jj = 0; jj < 20; ++jj)
				{
					FrameParams delta;
					float dsq = 0.0f;
					for(unsigned int j = 0; j < FrameParams::num_params; ++j)
						if(FrameParams::kmin[j] != FrameParams::kmax[j])
						{
							float d = delta[j] = Random3D::Rand() * 2.0f - 1.0f;
							dsq += d * d;
						}
				
					if(jj == 0 || dsq < best_dsq)
					{
						best_dsq = dsq;
						best_delta = delta;

						if(dsq <= 1.0f)
							break;
					}
				}

				float delta_scale = best_dsq <= 1.0f ? scale : Random3D::Rand(scale) / best_dsq;

				for(unsigned int j = 0; j < FrameParams::num_params; ++j)
				{
					float *ki = &k[j], kmini = FrameParams::kmin[j], kmaxi = FrameParams::kmax[j];
					*ki = max(kmini, min(kmaxi, *ki + delta_scale * (kmaxi - kmini) * best_delta[j]));
				}
			}
#else
			vector<unsigned int> nonequal;
			for(unsigned int i = 0; i < FrameParams::num_params; ++i)
				if(FrameParams::kmin[i] != FrameParams::kmax[i])
					nonequal.push_back(i);

			for(unsigned int i = 0; i < count; ++i)
			{
				FrameParams& frame = frames[Random3D::RandInt() % frames.size()];

				float *k, kmin, kmax;
				do
				{
					int index = nonequal[Random3D::RandInt() % nonequal.size()];//Random3D::RandInt() % FrameParams::num_params;
					k = &frame[index];
					kmin = FrameParams::kmin[index];
					kmax = FrameParams::kmax[index];
				} while(kmin == kmax);

				float kscale = scale * (kmax - kmin);
				*k = max(kmin, min(kmax, *k + Random3D::Rand() * kscale * 2.0f - kscale));
			}
#endif
		}

		GABrain* CreateCrossover(const GABrain& b, unsigned int nextid) const
		{
			GABrain* r = new GABrain(nextid, id, b.id);

			for(unsigned int i = 0; i < r->frames.size(); ++i)
			{
				FrameParams& rk = r->frames[i];
				const FrameParams& ak = frames[i];
				const FrameParams& bk = b.frames[i];
				for(unsigned int j = 0; j < FrameParams::num_params; ++j)
					rk[j] = max(FrameParams::kmin[j], min(FrameParams::kmax[j], CrossoverCoeff(ak[j], bk[j])));
			}

			return r;
		}

		static float CrossoverCoeff(float a, float b) { return a + (b - a) * Random3D::Rand(); }


		
		void Write(ostream& o)
		{
			WriteUInt32(FrameParams::num_params, o);
			WriteUInt32(frames.size(), o);
			for(unsigned int i = 0; i < frames.size(); ++i)
				for(unsigned int j = 0; j < FrameParams::num_params; ++j)
					WriteSingle(frames[i][j], o);
		}

		static unsigned int Read(istream& is, GABrain*& brain, unsigned int id)
		{
			brain = new GABrain(id, 0, 0);
			unsigned int params_count = ReadUInt32(is);
			unsigned int frames_count = ReadUInt32(is);
			brain->frames.clear();
			for(unsigned int i = 0; i < frames_count; ++i)
			{
				FrameParams fp;
				for(unsigned int j = 0; j < params_count; ++j)
				{
					float val = ReadSingle(is);
					if(j < FrameParams::num_params)
						fp[j] = val;
				}
				brain->frames.push_back(fp);
			}
			return 0;
		}
	};

	struct Subtest
	{
		unsigned int initial_frame;
		float initial_pitch;
		float initial_y;
		float pitch_move;
		float yaw_move;
		Vec3 torso_vel;
		Vec3 torso_rot;
		Vec3 desired_accel;

		Subtest() : initial_frame(0), initial_pitch(0), initial_y(0), pitch_move(0), yaw_move(0), torso_vel(), torso_rot(), desired_accel() { }
	};

	struct Experiment
	{
		vector<GABrain*> candidates;
		unsigned int gen_size;

		list<GABrain*> elites;

		GABrain* test;
		unsigned int test_index;

		unsigned int batch, trial;
		unsigned int next_id;

		string saved_debug_text;

		vector<Subtest> subtests;

		Experiment() : candidates(), test(NULL), batch(0), trial(0), next_id(1)
		{
			// load the saved best brain if possible
			ifstream file("Files/Brains/genepool", ios::in | ios::binary);
			if(!!file)
			{
				unsigned int num_genomes = ReadUInt32(file);
				for(unsigned int i = 0; i < num_genomes; ++i)
				{
					GABrain* loadme = NULL;
					if(unsigned int error = GABrain::Read(file, loadme, next_id++))
					{
						Debug(((stringstream&)(stringstream() << "Error " << error << " loading GABrain" << endl)).str());
						break;
					}
					else
					{
						for(unsigned int j = 0; j < loadme->frames.size(); ++j)
							for(unsigned int k = 0; k < FrameParams::num_params; ++k)
								loadme->frames[j][k] = max(FrameParams::kmin[k], min(FrameParams::kmax[k], loadme->frames[j][k]));
						//loadme->Randomize(0.3f, 10);
						candidates.push_back(loadme);
					}
				}

				if(!candidates.empty())
				{
					Debug(((stringstream&)(stringstream() << "Successfully loaded " << candidates.size() << " brains from genepool" << endl)).str());

					for(unsigned int i = 0; i < FrameParams::num_params; ++i)
					{
						float min, max, tot;
						min = max = tot = candidates[0]->frames[0][i];
						for(unsigned int j = 1; j < candidates.size(); ++j)
						{
							float f = candidates[j]->frames[0][i];
							tot += f;
							if(f < min)
								min = f;
							if(f > max)
								max = f;
						}
						Debug(((stringstream&)(stringstream() << "\tcomponent " << i << ": MIN = " << FrameParams::kmin[i] << "; min = " << min << "; max = " << max << "; MAX = " << FrameParams::kmax[i] << "; avg = " << tot / candidates.size() << endl)).str());
					}
				}

				file.close();
			}

			if(candidates.empty())
				MakeFirstGeneration();

			RecordGenSize();
			GenerateSubtestList();

			test = *candidates.rbegin();
			candidates.pop_back();
		}

		~Experiment()
		{
#if PROFILE_ANY_CPHFT
			DebugCPHFTProfilingData();
#endif

			// delete all the brains
			for(unsigned int i = 0; i < candidates.size(); ++i)
				delete candidates[i];
			
			test = NULL;
		}

		void GenerateSubtestList()
		{
			subtests.clear();
			for(unsigned int i = 0; i < NUM_SUBTESTS; ++i)
			{
				Subtest s;
				s.initial_frame = i % 2;//walk_keyframes.size();
				//s.initial_pitch = Random3D::Rand( -0.1f,   0.1f );
				//s.initial_y     = Random3D::Rand( -0.015f, 0.0f );
				//s.yaw_move      = Random3D::Rand( -1.0f,   1.0f );
				//s.pitch_move    = Random3D::Rand( -1.0f,   1.0f );
				//s.torso_vel     = Random3D::RandomNormalizedVector(Random3D::Rand(1.0f));
				//s.torso_rot     = Vec3(0, Random3D::Rand(-1.0f, 1.0f), 0);
				//s.desired_accel = Vec3(0, 0, i * 0.25f / float(NUM_SUBTESTS - 1));

				subtests.push_back(s);
			}
		}

		Subtest& GetSubtest() { return subtests[trial % subtests.size()]; }

		void DebugMat3(stringstream& ss, const Mat3& m)
		{
			ss << "{ ";
			for(unsigned int i = 0; i < 9; ++i)
				if(i != 0)
					ss << ", " << m[i];
				else
					ss << m[i];
			ss << " }";
		}

		void SaveGenepool(const string& filename, bool verbose = false)
		{
			ofstream file(filename, ios::out | ios::binary);
			if(!file)
				Debug("Failed to save brain!\n");
			else
			{
				WriteUInt32(elites.size(), file);
							
				for(list<GABrain*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
					(**iter).Write(file);

				file.close();

				if(verbose)
				{
					Debug(((stringstream&)(stringstream() << "Genepool saved to \"" << filename << "\"" << endl)).str());
					Debug(((stringstream&)(stringstream() << "batch " << batch << " elites:" << endl)).str());

					for(list<GABrain*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
					{
						GABrain* b = *iter;
						stringstream ss;
						ss << '\t' << b->GetText() << endl;

						for(unsigned int f = 0; f < b->frames.size(); ++f)
						{
							ss << "\t\tframe " << f << ":" << endl;
							const FrameParams& frame = b->frames[f];
							for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
							{
								ss << "\t\t\tbone " << i << ": weight A = " << powf(2.0f, frame.qp_bone_weights_angular[i]) << ", L = " << powf(2.0f, frame.qp_bone_weights_linear[i]) << "; L pred PID = ( " << frame.prediction_error_pcoeff[i] << ", " << frame.prediction_error_icoeff[i] << ", 0 )";
								const Vec3& xr = frame.xprod_radii[i];
								ss << " }; xprod radius = (" << xr.x << ", " << xr.y << ", " << xr.z << "); xprod frac = { ";
								for(unsigned int j = 0; j < NUM_LOWER_BODY_BONES + 1; ++j)
								{
									if(j != 0)
										ss << ", ";
									ss << frame.xprod_frac[i * NUM_LOWER_BODY_BONES + j];
								}
								ss << " }; gravity coeff = " << frame.gravity_coeff[i] << endl;
							}
						
							/*
							ss << "\t\t\tfoot torque coeff = { ";
							for(unsigned int i = 0; i < 4; ++i)
							{
								if(i != 0)
									ss << ", ";
								ss << frame.foot_torque_coeff[i];
							}
							ss << " }; foot force coeff = { ";
							for(unsigned int i = 0; i < 4; ++i)
							{
								if(i != 0)
									ss << ", ";
								ss << frame.foot_force_coeff[i];
							}
							ss << " }" << endl;
							ss << "\t\t\tgr foot t sub = { ";
							for(unsigned int i = 0; i < 4; ++i)
							{
								if(i != 0)
									ss << ", ";
								ss << frame.ground_foott_subfrac[i];
							}
							ss << " }; net f sub = { ";
							for(unsigned int i = 0; i < 4; ++i)
							{
								if(i != 0)
									ss << ", ";
								ss << frame.ground_netf_subfrac[i];
							}
							ss << " }" << endl;
							*/
							ss << "\t\t\t";
							ss << "ub weight L = " << pow(2.0f, frame.qp_bone_weights_linear[NUM_LOWER_BODY_BONES]);
							ss << "; hmax = " << frame.hmax << endl;
						}
						Debug(ss.str());
					}
				}
			}
		}

		GABrain* GetBrain() { return test; } const

		void ExperimentDone(const Scores& scores, string& debug_text, int numerator, int denominator, bool shouldFail)
		{
			if(trial == 0)
				test->scores = Scores();
			test->scores += scores;

			debug_text = ((stringstream&)(stringstream() << "batch " << batch << "; genome " << (gen_size - candidates.size()) << " of " << gen_size << "; (id = " << test->id << " p = " << test->pa << ", " << test->pb << "); trial = " << trial << endl << saved_debug_text)).str();

			++trial;
			float quasi_score = test->scores.total + test->scores.helper_force_penalty * (float(NUM_TRIALS) / trial - 1.0f);
			if(trial == NUM_TRIALS || elites.size() >= NUM_ELITES && quasi_score >= (**elites.rbegin()).scores.total * NUM_TRIALS)
			{
				test->scores /= float(trial);

				// find out where to put this score in the elites array; the elites list size may temporarily exceed NUM_ELITES
				list<GABrain*>::iterator insert_where = elites.begin();
				while(insert_where != elites.end() && test->scores.total > (**insert_where).scores.total)
					++insert_where;
				elites.insert(insert_where, test);

				string failstring = ((stringstream&)(stringstream() << " fail (" << numerator + (trial - 1) * denominator << " / " << denominator * NUM_TRIALS << ")")).str();

				// on-screen debug text (rankings list)
				stringstream ss;
				unsigned int i = 0;
				for(list<GABrain*>::iterator iter = elites.begin(); iter != elites.end(); ++iter, ++i)
				{
					if(i < NUM_ELITES || *iter == test)
					{
						if(i >= NUM_ELITES)
							ss << endl;
						ss << '\t' << (**iter).GetText();
						if(*iter == test)
						{
							if(i >= NUM_ELITES)
								ss << "\t<" << failstring;
							else
							{
								ss << "\t< latest";
								if(shouldFail)
									DEBUG();
							}
						}
						ss << endl;
					}
				}
				saved_debug_text = ss.str();

				// debug.txt genome result logging
				stringstream ss2;
				ss2 << test->GetText();
				if(test == *elites.begin())
					ss2 << "; new best!";
				else if(test == *elites.rbegin() && elites.size() > NUM_ELITES)
					ss2 << ";" << failstring;
				ss2 << endl;
				Debug(ss2.str());

				// cut off any extra items in elites
				if(elites.size() > NUM_ELITES)
				{
					delete *elites.rbegin();
					elites.pop_back();
				}

				// generation finished? save elites and build the next generation
				if(candidates.empty())
				{
					time_t raw_time;
					time(&raw_time);
					tm now = *localtime(&raw_time);
					string filename = ((stringstream&)(stringstream() << "Files/Brains/genepool-" << now.tm_year + 1900 << "-" << now.tm_mon + 1 << "-" << now.tm_mday << "-" << now.tm_hour << "-" << now.tm_min << "-" << now.tm_sec)).str();

					SaveGenepool(filename, true);
					SaveGenepool("Files/Brains/genepool");

					MakeNextGeneration();
					++batch;

					elites.clear();		// clear without deleting! they are still used (included in the next generation)
				}

#if GENERATE_SUBTEST_LIST_EVERY == 3
				GenerateSubtestList();
#endif

				test = *candidates.rbegin();
				candidates.pop_back();

				trial = 0;
			}
		}

		void MakeFirstGeneration()
		{
			unsigned int first_gen_size = NUM_ELITES * NUM_ELITES;
			for(unsigned int i = 0; i < first_gen_size; ++i)
			{
				GABrain* b = new GABrain(next_id++, 0, 0);

				for(unsigned int j = 0; j < b->frames.size(); ++j)
				{
					FrameParams& frame = b->frames[j];
					for(unsigned int k = 0; k < FrameParams::num_params; ++k)
#if 0
						frame[k] = 0.5f * (FrameParams::kmin[k] + FrameParams::kmax[k]);

					//frame.qp_bone_weights_angular[2] = frame.qp_bone_weights_angular[3] = frame.qp_bone_weights_angular[5] = frame.qp_bone_weights_angular[6] = -1.0f;

					//frame.gravity_coeff[0] = frame.gravity_coeff[1] = frame.gravity_coeff[7] = frame.gravity_coeff[8] = 0.0f;
					//frame.gravity_coeff[4] = 1.0f;
					
#else
						frame[k] = Random3D::Rand(FrameParams::kmin[k], FrameParams::kmax[k]);
#endif
				}

#if 0
				if(i >= NUM_ELITES)
					b->Randomize(MUTATION_SCALE, MUTATION_COUNT);
#endif

				candidates.push_back(b);
			}
		}
		
		void MakeNextGeneration()
		{
			candidates.clear();

			// push veterans
			for(list<GABrain*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
				candidates.push_back(*iter);

			for(list<GABrain*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
			{
				GABrain* pa = *iter;

				// push mutants
				for(unsigned int j = 0; j < MUTANTS_PER_ELITE; ++j)
				{
					GABrain* mutant = pa->CreateClone(next_id++);
					mutant->Randomize(MUTATION_SCALE, MUTATION_COUNT);
					candidates.push_back(mutant);
				}

				// push crossovers
				for(list<GABrain*>::iterator jter = iter; jter != elites.end(); ++jter)
					if(iter != jter)
					{
						GABrain* pb = *jter;
						for(unsigned int k = 0; k < CROSSOVERS_PER_PAIR; ++k)
						{
							GABrain* crossover = pa->CreateCrossover(*pb, next_id++);
							crossover->Randomize(MUTATION_SCALE, MUTATION_COUNT);
							candidates.push_back(crossover);
						}
					}
			}

			RecordGenSize();

#if GENERATE_SUBTEST_LIST_EVERY == 2
			GenerateSubtestList();
#endif

			elites.clear();
		}

		void RecordGenSize()
		{
			gen_size = candidates.size();
			Debug(((stringstream&)(stringstream() << "next generation will contain " << gen_size << " genomes" << endl)).str());

			if(gen_size > 0)			// else right becomes uint -1, causing nonsense
				for(unsigned int left = 0, right = gen_size - 1; left < right; ++left, --right)
					swap(candidates[left], candidates[right]);
		}
	};
	static Experiment* experiment = NULL;




	/*
	 * Soldier's custom FootState class
	 */
	class SoldierFoot : public Dood::FootState
	{
		public:

			SoldierFoot(unsigned int posey_id, const Vec3& ee_pos);

			Quaternion OrientBottomToSurface(const Vec3& normal) const;
	};




	/*
	 * Soldier private implementation struct
	 */
	struct Soldier::Imp
	{
		bool init;
		bool experiment_done;

		struct CBone
		{
			string name;
			RigidBody* rb;
			Bone* posey;

			Vec3 local_com;

			Vec3 desired_torque;
			Vec3 applied_torque;

			Vec3 desired_force;
			Vec3 old_vel;
			Vec3 predicted_force, prediction_error, perr_integral;

			Vec3 initial_pos;
			Quaternion initial_ori;

			CBone() { }
			CBone(const Soldier* dood, const string& name) : name(name), rb(dood->RigidBodyForNamedBone(name)), posey(dood->posey->skeleton->GetNamedBone(name)), local_com(rb->GetLocalCoM()), predicted_force(), initial_pos(rb->GetPosition()), initial_ori(rb->GetOrientation()) { }

			void Reset() { desired_torque = applied_torque = desired_force = Vec3(); }

			void ComputeDesiredTorque(const Quaternion& desired_ori, const Mat3& use_moi, float inv_timestep)
			{
				Quaternion ori      = rb->GetOrientation();
				Vec3 rot            = rb->GetAngularVelocity();

				Vec3 desired_rot    = (desired_ori * Quaternion::Reverse(ori)).ToRVec() * -inv_timestep;
				Vec3 desired_aaccel = (desired_rot - rot) * inv_timestep;

				desired_torque = use_moi * desired_aaccel;
			}

			void ComputeDesiredTorqueWithDefaultMoI(const Quaternion& desired_ori, float inv_timestep) { ComputeDesiredTorque(desired_ori, Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }
			void ComputeDesiredTorqueWithPosey(const Mat3& use_moi, float inv_timestep)                { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), use_moi, inv_timestep); }
			void ComputeDesiredTorqueWithDefaultMoIAndPosey(float inv_timestep)                        { ComputeDesiredTorque(posey->GetTransformationMatrix().ExtractOrientation(), Mat3(rb->GetTransformedMassInfo().moi), inv_timestep); }
		};

		CBone pelvis,    torso1, torso2, head;
		CBone lshoulder, luarm,  llarm,  lhand;
		CBone rshoulder, ruarm,  rlarm,  rhand;
		CBone luleg,     llleg,  lheel,  ltoe;
		CBone ruleg,     rlleg,  rheel,  rtoe;

		RigidBody* gun_rb;

		struct CJoint
		{
			SkeletalJointConstraint* sjc;
			CBone *a, *b;

			Vec3 actual;				// world-coords torque to be applied by this joint
			Vec3 last;

			Mat3 oriented_axes;			// gets recomputed every time Reset() is called

			Vec3 r1, r2;

			CJoint() : sjc(NULL), a(NULL), b(NULL) { }
			CJoint(const Soldier* dood, CBone& bone_a, CBone& bone_b, float max_torque)
			{
				RigidBody *arb = bone_a.rb, *brb = bone_b.rb;
				for(unsigned int i = 0; i < dood->constraints.size(); ++i)
				{
					SkeletalJointConstraint* j = (SkeletalJointConstraint*)dood->constraints[i];
					if(j->obj_a == arb && j->obj_b == brb)
					{
						a   = &bone_a;
						b   = &bone_b;
						sjc = j;

						sjc->min_torque = Vec3(-max_torque, -max_torque, -max_torque);
						sjc->max_torque = Vec3( max_torque,  max_torque,  max_torque);

						r1 = sjc->pos - a->local_com;
						r2 = sjc->pos - b->local_com;

						return;
					}
				}

				// joint not found?
				a = b = NULL;
				sjc = NULL;
			}

			void Reset()
			{
				last = sjc->apply_torque;

				sjc->apply_torque = actual = Vec3();
				oriented_axes = sjc->axes * Quaternion::Reverse(sjc->obj_a->GetOrientation()).ToMat3();
			}

			// returns true if UNABLE to match the requested value
			bool SetWorldTorque(const Vec3& torque)
			{
				Vec3 local_torque = oriented_axes * torque;

				const Vec3 &mint = sjc->min_torque, &maxt = sjc->max_torque;

				Vec3 old = oriented_axes.TransposedMultiply(sjc->apply_torque);

				sjc->apply_torque.x = max(mint.x, min(maxt.x, local_torque.x));
				sjc->apply_torque.y = max(mint.y, min(maxt.y, local_torque.y));
				sjc->apply_torque.z = max(mint.z, min(maxt.z, local_torque.z));

				Vec3 dif = sjc->apply_torque - local_torque;
				bool result = (dif.x != 0 || dif.y != 0 || dif.z != 0);

				if(result)
					actual = oriented_axes.TransposedMultiply(sjc->apply_torque);
				else
					actual = torque;

				Vec3 delta = actual - old;

				b->applied_torque -= delta;
				a->applied_torque += delta;

				return result;
			}

			bool SetTorqueToSatisfyA() { return SetWorldTorque(a->desired_torque - (a->applied_torque - actual)); }
			bool SetTorqueToSatisfyB() { return SetWorldTorque((b->applied_torque + actual) - b->desired_torque); }

			bool SetOrientedTorque(const Vec3& local_torque)
			{
				Vec3 old = oriented_axes.TransposedMultiply(sjc->apply_torque);

				const Vec3 &mint = sjc->min_torque, &maxt = sjc->max_torque;

				sjc->apply_torque.x = max(mint.x, min(maxt.x, local_torque.x));
				sjc->apply_torque.y = max(mint.y, min(maxt.y, local_torque.y));
				sjc->apply_torque.z = max(mint.z, min(maxt.z, local_torque.z));

				Vec3 dif = sjc->apply_torque - local_torque;
				bool result = (dif.x != 0 || dif.y != 0 || dif.z != 0);

				actual = oriented_axes.TransposedMultiply(sjc->apply_torque);

				Vec3 delta = actual - old;

				b->applied_torque -= delta;
				a->applied_torque += delta;

				return result;
			}
		};

		CJoint spine1, spine2, neck;
		CJoint lsja,   lsjb,   lelbow, lwrist;
		CJoint rsja,   rsjb,   relbow, rwrist;
		CJoint lhip,   lknee,  lankle, lht;
		CJoint rhip,   rknee,  rankle, rht;

		vector<CBone*>  all_bones;
		vector<CJoint*> all_joints;

		CJoint* leg_joints[NUM_LEG_JOINTS];
		CBone* lower_body_bones[NUM_LOWER_BODY_BONES];

		Scores cat_scores;

		Vec3 desired_pelvis_pos;
		Vec3 initial_com, desired_com;

		Quaternion old_p;

		float timestep, inv_timestep;

		unsigned int tick_age, max_tick_age;

		struct JetpackNozzle
		{
			CBone* bone;

			Vec3 pos;
			Vec3 cone_center;
			float cone_cossq;
			float max_force, max_forcesq;

			Vec3 world_force, world_torque;
			Vec3 try_force, try_torque;

			Vec3 world_center;
			Vec3 apply_pos;
			Mat3 force_to_torque;

			JetpackNozzle(CBone& bone, const Vec3& pos, const Vec3& cone_center, float cone_angle, float max_force) : bone(&bone), pos(pos), cone_center(cone_center), cone_cossq(cosf(cone_angle)), max_force(max_force), max_forcesq(max_force * max_force) { cone_cossq *= cone_cossq; }

			void Reset() { world_force = Vec3(); }

			void SolverInit(const Vec3& dood_com, float prop_frac)
			{
				const RigidBody& rb = *bone->rb;
				Mat3 rm = rb.GetOrientation().ToMat3();
				world_center = rm * cone_center;
				apply_pos    = rm * pos + rb.GetPosition();

				// compute force-to-torque Mat3
				Vec3 bone_com = rb.GetCenterOfMass();
				Vec3 r1 = apply_pos - bone_com;
				Mat3 xr1 = Mat3(        0,   r1.z,  -r1.y,
									-r1.z,      0,   r1.x,
									 r1.y,  -r1.x,      0	);
				Vec3 r2 = bone_com - dood_com;
				Mat3 xr2 = Mat3(        0,   r2.z,  -r2.y,
									-r2.z,      0,   r2.x,
									 r2.y,  -r2.x,      0	);
				force_to_torque = xr1 + xr2;			// is this right?


				world_force  = world_center * max_force * prop_frac;
				world_torque = force_to_torque * world_force;

				try_force  = world_force;
				try_torque = world_torque;
			}

			void GetNudgeEffects(const Vec3& nudge, Vec3& nu_force, Vec3& nu_torque)
			{
				nu_force = world_force + nudge;

				/*
				float dot = Vec3::Dot(nu_force, world_center);
				if(dot <= 0.0f)
					nu_force = nu_torque = Vec3();
				else
				{
					Vec3 axial = world_center * dot;
					Vec3 ortho = nu_force - axial;
					float axialsq = axial.ComputeMagnitudeSquared();
					float orthosq = ortho.ComputeMagnitudeSquared();
					if(orthosq > axialsq * cone_cossq)
					{
						ortho *= sqrtf(axialsq * cone_cossq / orthosq);
						nu_force = axial + ortho;
					}*/

					float magsq = nu_force.ComputeMagnitudeSquared();
					if(magsq > max_forcesq)
						nu_force *= sqrtf(max_forcesq / magsq);

					nu_torque = force_to_torque * nu_force;
				//}
			}

			void ApplySelectedForce(float timestep)
			{
				//bone->rb->ApplyWorldForce(world_force, apply_pos);				// TODO: make this work?
				bone->rb->ApplyWorldImpulse(world_force * timestep, apply_pos);
			}
		};

		Vec3 initial_pos;

		vector<JetpackNozzle> jetpack_nozzles;
		bool jetpacking;
		Vec3 desired_jp_accel;

		Vec3 desired_aim;

		Vec3 gun_initial_pos;
		Quaternion gun_initial_ori;

		vector<float> a_data, j_data, zwork_data, z_data;

		GABrain* brain;
		unsigned int frame_index;
		float frame_time;

		Imp() :
			init(false),
			experiment_done(false),
			timestep(0),
			inv_timestep(0),
			tick_age(0),
			max_tick_age(MAX_TICK_AGE)
		{
		}

		~Imp() { }

		void RegisterBone (CBone& bone)   { all_bones.push_back(&bone); }
		void RegisterJoint(CJoint& joint) { all_joints.push_back(&joint); }

		void RegisterSymmetricJetpackNozzles(CBone& lbone, CBone& rbone, const Vec3& lpos, const Vec3& lnorm, float angle, float force)
		{
			jetpack_nozzles.push_back(JetpackNozzle(lbone, lpos,                          lnorm,                            angle, force));
			jetpack_nozzles.push_back(JetpackNozzle(rbone, Vec3(-lpos.x, lpos.y, lpos.z), Vec3(-lnorm.x, lnorm.y, lnorm.z), angle, force));
		}


		// helper functions, called by Soldier::Imp::Init
		void InitBoneHelpers(Soldier* dood)
		{
			RegisterBone( pelvis    = CBone( dood, "pelvis"     ));
			RegisterBone( torso1    = CBone( dood, "torso 1"    ));
			RegisterBone( torso2    = CBone( dood, "torso 2"    ));
			RegisterBone( head      = CBone( dood, "head"       ));
			RegisterBone( lshoulder = CBone( dood, "l shoulder" ));
			RegisterBone( luarm     = CBone( dood, "l arm 1"    ));
			RegisterBone( llarm     = CBone( dood, "l arm 2"    ));
			RegisterBone( lhand     = CBone( dood, "l hand"     ));
			RegisterBone( rshoulder = CBone( dood, "r shoulder" ));
			RegisterBone( ruarm     = CBone( dood, "r arm 1"    ));
			RegisterBone( rlarm     = CBone( dood, "r arm 2"    ));
			RegisterBone( rhand     = CBone( dood, "r hand"     ));
			RegisterBone( luleg     = CBone( dood, "l leg 1"    ));
			RegisterBone( llleg     = CBone( dood, "l leg 2"    ));
			RegisterBone( lheel     = CBone( dood, "l heel"     ));
			RegisterBone( ltoe      = CBone( dood, "l toe"      ));
			RegisterBone( ruleg     = CBone( dood, "r leg 1"    ));
			RegisterBone( rlleg     = CBone( dood, "r leg 2"    ));
			RegisterBone( rheel     = CBone( dood, "r heel"     ));
			RegisterBone( rtoe      = CBone( dood, "r toe"      ));
		}

		void InitJointHelpers(Soldier* dood)
		{
			float SP = 1500, N = 150, W = 200, E = 350, SB = 600, SA = 700, H = 1400, K = 1000, A = 600, HT = 600;
			RegisterJoint( spine1 = CJoint( dood, pelvis,    torso1,    SP ));
			RegisterJoint( spine2 = CJoint( dood, torso1,    torso2,    SP ));
			RegisterJoint( neck   = CJoint( dood, torso2,    head,      N  ));
			RegisterJoint( lsja   = CJoint( dood, torso2,    lshoulder, SA ));
			RegisterJoint( lsjb   = CJoint( dood, lshoulder, luarm,     SB ));
			RegisterJoint( lelbow = CJoint( dood, luarm,     llarm,     E  ));
			RegisterJoint( lwrist = CJoint( dood, llarm,     lhand,     W  ));
			RegisterJoint( rsja   = CJoint( dood, torso2,    rshoulder, SA ));
			RegisterJoint( rsjb   = CJoint( dood, rshoulder, ruarm,     SB ));
			RegisterJoint( relbow = CJoint( dood, ruarm,     rlarm,     E  ));
			RegisterJoint( rwrist = CJoint( dood, rlarm,     rhand,     W  ));
			RegisterJoint( lhip   = CJoint( dood, pelvis,    luleg,     H  ));
			RegisterJoint( lknee  = CJoint( dood, luleg,     llleg,     K  ));
			RegisterJoint( lankle = CJoint( dood, llleg,     lheel,     A  ));
			RegisterJoint( lht    = CJoint( dood, lheel,     ltoe,      HT ));
			RegisterJoint( rhip   = CJoint( dood, pelvis,    ruleg,     H  ));
			RegisterJoint( rknee  = CJoint( dood, ruleg,     rlleg,     K  ));
			RegisterJoint( rankle = CJoint( dood, rlleg,     rheel,     A  ));
			RegisterJoint( rht    = CJoint( dood, rheel,     rtoe,      HT ));

			CJoint* temp_leg_joints[NUM_LEG_JOINTS] = { &lht, &lankle, &lknee, &lhip, &rht, &rankle, &rknee, &rhip };
			for(unsigned int i = 0; i < NUM_LEG_JOINTS; ++i)
				leg_joints[i] = temp_leg_joints[i];

			// knees have special torque limits (smaller on the second and third axes)
			float KS = 0;//K * 0.25f;
			lknee.sjc->min_torque.y = -KS;
			lknee.sjc->min_torque.z = -KS;
			lknee.sjc->max_torque.y =  KS;
			lknee.sjc->max_torque.z =  KS;
			rknee.sjc->min_torque.y = -KS;
			rknee.sjc->min_torque.z = -KS;
			rknee.sjc->max_torque.y =  KS;
			rknee.sjc->max_torque.z =  KS;
		}

		void InitJetpackNozzles()
		{
			Vec3 upward(0, 1, 0);
			float jp_angle = 1.0f;
			float jp_force = 150.0f;		// 98kg * 15m/s^2 accel / 10 nozzles ~= 150N per nozzle

			RegisterSymmetricJetpackNozzles( lshoulder, rshoulder, Vec3( 0.442619f, 1.576419f, -0.349652f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lshoulder, rshoulder, Vec3( 0.359399f, 1.523561f, -0.366495f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lshoulder, rshoulder, Vec3( 0.277547f, 1.480827f, -0.385142f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( ltoe,      rtoe,      Vec3( 0.237806f, 0.061778f,  0.038247f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lheel,     rheel,     Vec3( 0.238084f, 0.063522f, -0.06296f  ), upward, jp_angle, jp_force );
		}

		void Init(Soldier* dood)
		{
			//dood->collision_group->SetInternalCollisionsEnabled(true);		// TODO: resolve problems arising from torso2-arm1 collisions

			// init some component helpers
			all_bones.clear();
			all_joints.clear();
			jetpack_nozzles.clear();

			InitBoneHelpers(dood);
			InitJointHelpers(dood);												// this method also responsible for setting joint torque limits!
			InitJetpackNozzles();

			CBone* temp_lb_bones[NUM_LOWER_BODY_BONES] = { &ltoe, &lheel, &llleg, &luleg, &pelvis, &ruleg, &rlleg, &rheel, &rtoe };
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
				lower_body_bones[i] = temp_lb_bones[i];

			// some misc. initialization
			no_touchy.imp = this;
			for(set<RigidBody*>::iterator iter = dood->velocity_change_bodies.begin(); iter != dood->velocity_change_bodies.end(); ++iter)
				if(*iter != lheel.rb && *iter != rheel.rb && *iter != ltoe.rb && *iter != rtoe.rb)
					(*iter)->SetContactCallback(&no_touchy);

			desired_pelvis_pos = pelvis.rb->GetCenterOfMass();

			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				gun_rb = gun->rigid_body;
				gun_initial_pos = gun_rb->GetPosition();
				gun_initial_ori = gun_rb->GetOrientation();
			}

			brain = experiment->GetBrain();
			cat_scores = Scores();

			frame_index = 0;
			frame_time = 0.0f;

			initial_pos = dood->pos;
		}

		void ReInit(Soldier* dood)
		{
			dood->yaw = dood->pitch = 0.0f;
			dood->pos = initial_pos;

			dood->DoInitialPose();
			dood->posey->skeleton->InvalidateCachedBoneXforms();

			for(unsigned int i = 0; i < all_bones.size(); ++i)
			{
				CBone& cb = *all_bones[i];
				RigidBody& rb = *cb.rb;

				Bone* posey = cb.posey;
				posey->GetTransformationMatrix().Decompose(cb.initial_pos, cb.initial_ori);		// re-computes bones' initial states

				rb.SetPosition   (cb.initial_pos);
				rb.SetOrientation(cb.initial_ori);
				rb.SetLinearVelocity (Vec3());
				rb.SetAngularVelocity(Vec3());
			}

			if(gun_rb != NULL)
			{
				Mat4 xform = ((Gun*)dood->equipped_weapon)->GetInitialXform();

				Vec3 pos = xform.TransformVec3_1(0, 0, 0);
				Vec3 a   = xform.TransformVec3_0(1, 0, 0);
				Vec3 b   = xform.TransformVec3_0(0, 1, 0);
				Vec3 c   = xform.TransformVec3_0(0, 0, 1);

				//gun_rb->SetPosition   (gun_initial_pos);
				//gun_rb->SetOrientation(gun_initial_ori);
				gun_rb->SetPosition(pos);
				gun_rb->SetOrientation(Quaternion::FromRotationMatrix(Mat3(a.x, b.x, c.x, a.y, b.y, c.y, a.z, b.z, c.z)));
				gun_rb->SetLinearVelocity (Vec3());
				gun_rb->SetAngularVelocity(Vec3());
			}

			for(unsigned int i = 0; i < all_bones.size(); ++i)
			{
				CBone& bone = *all_bones[i];
				bone.old_vel = bone.predicted_force = bone.prediction_error = bone.perr_integral = Vec3();
			}
			for(unsigned int i = 0; i < all_joints.size(); ++i)
				all_joints[i]->last = Vec3();

			experiment_done = false;
			tick_age = 0;

			frame_index = 0;
			frame_time = 0.0f;

			brain = experiment->GetBrain();
			cat_scores = Scores();
		}



		void GetDesiredTorsoOris(Soldier* dood, Quaternion& p, Quaternion& t1, Quaternion& t2, const Vec3& fudge_t2 = Vec3(), const Vec3& fudge_p = Vec3())
		{
			float pfrac = dood->pitch * (2.0f / float(M_PI)), pfsq = pfrac * pfrac;

			float t1_yaw   = dood->yaw + torso2_yaw_offset;
			float t1_pitch = dood->pitch * 0.4f + pfsq * pfrac * 0.95f;
			float t1_yaw2  = pfsq * 0.7f;

			t2 = Quaternion::FromRVec(0, -t1_yaw, 0) * Quaternion::FromRVec(t1_pitch, 0, 0) * Quaternion::FromRVec(0, -t1_yaw2, 0);
			t2 = t2 * Quaternion::FromRVec(Quaternion::FromRVec(0, -dood->yaw, 0) * fudge_t2);

			float t2_yaw   = dood->yaw + torso2_yaw_offset * 0.5f;
			float t2_pitch = pfrac * 0.05f + pfrac * pfsq * 0.3f;
			float t2_yaw2  = pfsq * 0.15f;

			p = Quaternion::FromRVec(0, -t2_yaw, 0) * Quaternion::FromRVec(t2_pitch, 0, 0) * Quaternion::FromRVec(0, -t2_yaw2, 0);
			p = p * Quaternion::FromRVec(Quaternion::FromRVec(0, -dood->yaw, 0) * fudge_p);

			Quaternion twist_ori = p * Quaternion::Reverse(t2);
			t1 = Quaternion::FromRVec(twist_ori.ToRVec() * -0.5f) * t2;
		}

		void DoHeadOri(Soldier* dood, const TimingInfo& time)
		{
			Quaternion desired_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);

			head.ComputeDesiredTorqueWithDefaultMoI(desired_ori, inv_timestep);
			neck.SetTorqueToSatisfyB();
		}

		void DoArmsAimingGun(Soldier* dood, const TimingInfo& time, const Quaternion& t2ori)
		{
			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				// compute desired per-bone net torques
				MassInfo hng_mass_infos[] = { lhand.rb->GetTransformedMassInfo(), rhand.rb->GetTransformedMassInfo(), gun->rigid_body->GetTransformedMassInfo() };
				Mat3 hng_moi = Mat3(MassInfo::Sum(hng_mass_infos, 3).moi);

				dood->PreparePAG(time, t2ori);

				lhand    .ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				llarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				luarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				lshoulder.ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				rhand    .ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				rlarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				ruarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				rshoulder.ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				// compute applied joint torques to achieve the per-bone applied torques we just came up with
				lwrist.SetWorldTorque(-lhand.desired_torque * 0.75f);
				rwrist.SetWorldTorque(-lwrist.actual - rhand.desired_torque);
				
				lelbow.SetTorqueToSatisfyB();
				lsjb  .SetTorqueToSatisfyB();
				lsja  .SetTorqueToSatisfyB();

				relbow.SetTorqueToSatisfyB();
				rsjb  .SetTorqueToSatisfyB();
				rsja  .SetTorqueToSatisfyB();
			}
		}

		void ComputeMomentumStuff(const set<RigidBody*>& included_rbs, float& dood_mass, Vec3& dood_com, Vec3& com_vel, Vec3& angular_momentum)
		{
			com_vel = Vec3();
			dood_mass = 0.0f;
			for(set<RigidBody*>::iterator iter = included_rbs.begin(); iter != included_rbs.end(); ++iter)
			{
				RigidBody* rb = *iter;
				float mass = rb->GetMass();
				dood_mass += mass;
				dood_com  += rb->GetCachedCoM() * mass;
				com_vel   += rb->GetLinearVelocity() * mass;
			}
			dood_com /= dood_mass;
			com_vel  /= dood_mass;

			MassInfo mass_info;
			Mat3& moi = *((Mat3*)((void*)mass_info.moi));						// moi.values and mass_info.moi occupy the same space in memory

			for(set<RigidBody*>::iterator iter = included_rbs.begin(); iter != included_rbs.end(); ++iter)
			{
				RigidBody* body = *iter;
				mass_info = body->GetTransformedMassInfo();

				// linear component
				float mass  = mass_info.mass;
				Vec3 vel    = body->GetLinearVelocity() - com_vel;
				Vec3 radius = mass_info.com - dood_com;
				angular_momentum += Vec3::Cross(vel, radius) * mass;

				// angular component
				angular_momentum += moi * body->GetAngularVelocity();
			}
		}

		void ResolveJetpackOutput(Soldier* dood, const TimingInfo& time, float dood_mass, const Vec3& dood_com, const Vec3& desired_jp_accel, const Vec3& desired_jp_torque)
		{
			unsigned int num_nozzles = jetpack_nozzles.size();

			for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
				iter->SolverInit(dood_com, 0.0f);

#if ENABLE_NEW_JETPACKING
			Vec3 desired_jp_force = desired_jp_accel * dood_mass;

			float torque_coeff = 50.0f;

			// search for nozzle forces to match the requested accel & torque
			Vec3 force_error, torque_error;
			float errsq, error;
			for(unsigned int i = 0; i < 500; ++i)
			{
				if(i == 0)
				{
					force_error  = -desired_jp_force;
					torque_error = -desired_jp_torque;
					for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
					{
						force_error  += iter->world_force;
						torque_error += iter->world_torque;
					}
					
					errsq = force_error.ComputeMagnitudeSquared() + torque_error.ComputeMagnitudeSquared() * torque_coeff;
				}
				else
				{
					float mutation_scale = error * 0.25f;
					Vec3 mutant_force  = force_error;
					Vec3 mutant_torque = torque_error;
					for(unsigned char j = 0; j < 3; ++j)
					{
						JetpackNozzle& jpn = jetpack_nozzles[Random3D::RandInt() % num_nozzles];

						jpn.GetNudgeEffects(Random3D::RandomNormalizedVector(Random3D::Rand(mutation_scale)), jpn.try_force, jpn.try_torque);

						mutant_force  += jpn.try_force  - jpn.world_force;
						mutant_torque += jpn.try_torque - jpn.world_torque;
					}

					float mutant_errsq = mutant_force.ComputeMagnitudeSquared() + mutant_torque.ComputeMagnitudeSquared() * torque_coeff;
					if(mutant_errsq < errsq)
					{
						for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
						{
							iter->world_force  = iter->try_force;
							iter->world_torque = iter->try_torque;
						}
						force_error  = mutant_force;
						torque_error = mutant_torque;

						errsq = mutant_errsq;
					}
				}

				error = sqrtf(errsq);
				Debug(((stringstream&)(stringstream() << "i = " << i << "; error squared = " << errsq << "; error = " << error << endl)).str());
				if(error < 1.0f)
					break;
			}
#endif
			
			// apply the nozzle forces we computed
			for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
				iter->ApplySelectedForce(timestep);
		}


		void DoSubtestAimMove(Soldier* dood, const Subtest& subtest)
		{
			float aimfrac = sinf(min(1.0f, (float)tick_age / 120.0f) * float(M_PI));
			float aimmove = aimfrac * aimfrac * timestep * 0.1f;

			dood->yaw   += subtest.yaw_move   * aimmove;
			dood->pitch += subtest.pitch_move * aimmove;
		}
		void DoSubtestInitialVelocity(Soldier* dood, const Subtest& subtest)
		{
			Vec3 vvec = subtest.torso_vel * 0.005f;
			head  .rb->SetLinearVelocity(head.old_vel = vvec);
			torso2.rb->SetLinearVelocity(head.old_vel = vvec);
			torso1.rb->SetLinearVelocity(head.old_vel = vvec * 0.6f);
			pelvis.rb->SetLinearVelocity(head.old_vel = vvec * 0.2f);

			static const float r = 0.00125f;
			Vec3 avvec = subtest.torso_rot * r;
			head  .rb->SetAngularVelocity(avvec);
			torso2.rb->SetAngularVelocity(avvec);
			torso1.rb->SetAngularVelocity(avvec * 0.6f);
			pelvis.rb->SetAngularVelocity(avvec * 0.2f);
		}



		void Update(Soldier* dood, const TimingInfo& time)
		{
#if PROFILE_CPHFT
			ProfilingTimer timer, timer2;
			timer2.Start();
			timer.Start();
#endif

			if(!init)
			{
				Init(dood);
				init = true;
			}
			else if(experiment_done)
				ReInit(dood);

			timestep     = time.elapsed;
			inv_timestep = 1.0f / timestep;

			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
				gun_rb = gun->rigid_body;
			else
				gun_rb = NULL;

			const Subtest& subtest = experiment->GetSubtest();
			DoSubtestAimMove(dood, subtest);
			//if(tick_age == 0)
			//	DoSubtestInitialVelocity(dood, subtest);

			//RigidBody& t2rb = *torso2.rb;
			//t2rb.SetLinearVelocity(t2rb.GetLinearVelocity() + subtest.torso_vel * (10.0f * timestep / t2rb.GetMass()));
			//t2rb.SetAngularVelocity(t2rb.GetAngularVelocity() + t2rb.GetInvMoI() * subtest.torso_rot * (1.0f * timestep));

#if PROFILE_CPHFT
			timer_init += timer.GetAndRestart();
#endif

			// reset all the joints and bones
			for(vector<CJoint*>::iterator iter = all_joints.begin(); iter != all_joints.end(); ++iter)
				(*iter)->Reset();
			for(vector<CBone*>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
				(*iter)->Reset();

#if PROFILE_CPHFT
			timer_reset += timer.GetAndRestart();
#endif

			float dood_mass;
			Vec3 dood_com, com_vel, angular_momentum;

			set<RigidBody*> included_rbs;
			//for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
				//included_rbs.insert(lower_body_bones[i]->rb);
			included_rbs.insert(pelvis.rb);		// TODO: remove this when reverting?
			included_rbs.insert(torso1.rb);
			included_rbs.insert(torso2.rb);
			included_rbs.insert(head.rb);
			included_rbs.insert(lshoulder.rb);
			included_rbs.insert(rshoulder.rb);

			ComputeMomentumStuff(included_rbs, dood_mass, dood_com, com_vel, angular_momentum);

			if(tick_age == 0)
				desired_com = initial_com = dood_com;

#if PROFILE_CPHFT
			timer_massinfo += timer.GetAndRestart();
#endif

#if ENABLE_NEW_JETPACKING
			if(jetpacking)
			{
				Vec3 desired_jp_torque = angular_momentum * (-60.0f);
				ResolveJetpackOutput(dood, time, dood_mass, dood_com, desired_jp_accel, desired_jp_torque);
			}
			else
			{
				// this will be necessary for when rendering for jetpack flames is eventually added
				for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
					iter->Reset();
			}
#endif

			

			// compute desired pose for leg bones
			unsigned int num_walk_keyframes = walk_keyframes.size();
			frame_time += timestep;
			const WalkKeyframe* current_frame = &walk_keyframes[frame_index];
			while(frame_time > current_frame->duration)
			{
				frame_time -= current_frame->duration;
				frame_index = (frame_index + 1) % num_walk_keyframes;
				current_frame = &walk_keyframes[frame_index];
			}
			const WalkKeyframe* next_frame = &walk_keyframes[(frame_index + 1) % num_walk_keyframes];
			float kf_time = tick_age / current_frame->duration;
			float kf_frac = kf_time - (int)kf_time;
			unsigned int kf_index1 = ((unsigned int)kf_time) % num_walk_keyframes;
			unsigned int kf_index2 = (kf_index1 + 1) % num_walk_keyframes;
			WalkKeyframe use_pose = WalkKeyframe::Interpolate(*current_frame, *next_frame, kf_frac);
			FrameParams use_params = FrameParams::Interpolate(brain->frames[kf_index1], brain->frames[kf_index2], kf_frac);

			// upper body stuff; mostly working
			Quaternion p, t1, t2;
			Quaternion yaw_ori = Quaternion::FromAxisAngle(0, 1, 0, -dood->yaw);
			Mat3 yawmat = yaw_ori.ToMat3();
			Mat3 unyaw = yawmat.Transpose();

			{
				Vec3 desired_vel = (tick_age * timestep) * subtest.desired_accel;
				float dvmag = desired_vel.ComputeMagnitude();
				if(dvmag > 5.0f)
					desired_vel *= 5.0f / dvmag;

				desired_com += desired_vel;
			}

			Vec3 momentum = com_vel * dood_mass;
			Vec3 com_error = desired_com - dood_com;
			Vec3 desired_momentum = com_error * (dood_mass * inv_timestep * 0.25f);
			Vec3 desired_force = (desired_momentum - momentum) * (inv_timestep * 0.25f);

//			for(unsigned int i = 0; i < 4; ++i)
//				desired_force -= dood->feet[i]->net_force * use_params.ground_netf_subfrac[i];

			Vec3 unyaw_df = unyaw * desired_force;
			Vec3 unyaw_dc = unyaw * com_error * (dood_mass);

			Vec3 torso2_fudge;
			Vec3 pelvis_fudge;
			GetDesiredTorsoOris(dood, p, t1, t2, torso2_fudge, pelvis_fudge);

			DoHeadOri      ( dood, time     );
			DoArmsAimingGun( dood, time, t2 );

			torso2.ComputeDesiredTorqueWithDefaultMoI(t2, inv_timestep);
			torso1.ComputeDesiredTorqueWithDefaultMoI(t1, inv_timestep);

			// compute desired net torques to get leg bones into their desired orientations
			Quaternion lb_desired_oris[NUM_LOWER_BODY_BONES];
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
			{
				if(i == 4)
					lb_desired_oris[i] = p;
				else
					lb_desired_oris[i] = lower_body_bones[i]->initial_ori;

				lower_body_bones[i]->ComputeDesiredTorqueWithDefaultMoI(lb_desired_oris[i], inv_timestep);
			}

			spine2.SetTorqueToSatisfyB();
			spine1.SetTorqueToSatisfyB();

#if PROFILE_CPHFT
			timer_ub_stuff += timer.GetAndRestart();
#endif

			// more factors affecting computation of bones' desired torques
			float ideal_bone_torques[NUM_LB_BONE_FLOATS + 3];
			memset(ideal_bone_torques, 0, sizeof(float) * (NUM_LB_BONE_FLOATS + 3));
			float* ibt_ptr = ideal_bone_torques;

			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
			{
				CBone* bone = lower_body_bones[i];

				Vec3 ibt, bdesired_force;
				
				ibt = bone->desired_torque - bone->applied_torque;

				//if(i == 4)
				//	bdesired_force = desired_force;// * (pelvis.rb->GetMass() / dood_mass);
				//else
				//{
					Vec3 desired_pos = bone->initial_pos + bone->initial_ori * bone->local_com;
					Vec3 desired_vel = (desired_pos - bone->rb->GetCenterOfMass()) * (inv_timestep * 0.25f);
					bdesired_force = (desired_vel - bone->rb->GetLinearVelocity()) * (inv_timestep * 0.25f * bone->rb->GetMass());
				//}

				*(ibt_ptr++) = ibt.x;
				*(ibt_ptr++) = ibt.y;
				*(ibt_ptr++) = ibt.z;

				bone->desired_force = bdesired_force;
				*(ibt_ptr++) = bdesired_force.x;
				*(ibt_ptr++) = bdesired_force.y;
				*(ibt_ptr++) = bdesired_force.z;
			}

			*(ibt_ptr++) = desired_force.x;
			*(ibt_ptr++) = desired_force.y;
			*(ibt_ptr++) = desired_force.z;

			static const unsigned int foot_bt_indices[4] = { 0, 2, 16, 14 };	// l toe, l heel, r toe, r heel
//			for(unsigned int i = 0; i < 4; ++i)
//				((Vec3*)ideal_bone_torques)[foot_bt_indices[i]]  -= dood->feet[i]->net_torque * use_params.ground_foott_subfrac[i];

			// generate matrix A (in "Af = b"), which maps joint torques to bone torques
			vector<float> base_forces(NUM_LOWER_BODY_BONES * 3 + 3);
			GenericMatrix A = ComputeMatrixA(dood->feet, use_params, a_data, base_forces, com_vel, angular_momentum, dood_mass);

#if 0
			stringstream ss;
			ss << "t = " << tick_age << "; Matrix A:" << endl;
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES * 6; ++i)
			{
				ss << "\t{ ";
				for(unsigned int j = 0; j < NUM_LEG_JOINTS * 3; ++j)
				{
					if(j != 0)
						ss << ", ";
					//if(j == NUM_LEG_JOINTS * 3 / 2)
					//	ss << "          ";
					ss << A[i][j];
				}
				if(i % 6 >= 3)
					ss << " } - " << base_forces[(i / 6) * 3 + i % 3] << endl;
				else
					ss << " }" << endl;
			}
			Debug(ss.str());
#endif

			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
				((Vec3*)ideal_bone_torques)[i * 2 + 1] -= ((Vec3*)base_forces.data())[i];
			((Vec3*)ideal_bone_torques)[NUM_LOWER_BODY_BONES * 2] -= ((Vec3*)base_forces.data())[NUM_LOWER_BODY_BONES];

#if PROFILE_CPHFT
			timer_a_matrix += timer.GetAndRestart();
#endif

			// solve the QP (quadratic program) ... aka find the f which minimizes ||Af-b||^2 with some component-specific weights
			float joint_torques     [NUM_JOINT_AXES];
			float bone_torque_errors[NUM_LB_BONE_FLOATS + 3];
			memset(joint_torques, 0, sizeof(float) * NUM_JOINT_AXES);

			SolveForOptimalF(use_params, A, ideal_bone_torques, joint_torques, bone_torque_errors);


#if PROFILE_CPHFT
			timer_solve_qp += timer.GetAndRestart();
#endif

			// finally, tell the joints to output these torques
			const Vec3 *tptr = (Vec3*)joint_torques;
			for(unsigned int i = 0; i < NUM_LEG_JOINTS; ++i)
				leg_joints[i]->SetOrientedTorque(*(tptr++));

			float mis_prediction = 0.0f;
			for(unsigned int i = 0; i <= NUM_LOWER_BODY_BONES; ++i)
			{
				CBone& bone = i == NUM_LOWER_BODY_BONES ? torso2 : *lower_body_bones[i];
				if(i <= 1 || i >= 7 || i == 4)
					mis_prediction += bone.prediction_error.ComputeMagnitudeSquared() * timestep / bone.rb->GetMass();
				for(unsigned int j = 0; j < 3; ++j)			// ... this is to prevent a floating point overflow
				{
					float& bteij = bone_torque_errors[i == NUM_LOWER_BODY_BONES ? i * 6 + j : i * 6 + 3 + j];
					bteij = max(-2000.0f, min(2000.0f, bteij));
				}
				int index = i == NUM_LOWER_BODY_BONES ? NUM_LOWER_BODY_BONES * 2 : i * 2 + 1;
				bone.predicted_force = ((Vec3*)bone_torque_errors)[index] + ((Vec3*)ideal_bone_torques)[index] + ((Vec3*)base_forces.data())[i];
			}

			// compute helper forces and apply them
			float use_hmax = max(0.0f, use_params.hmax);
			static const unsigned int HF_DIM = NUM_LB_BONE_FLOATS + 3;
			float helper_force[HF_DIM];
			Vec3* hf_vec = (Vec3*)helper_force;

			hf_vec[0] = desired_force - torso2.predicted_force;
			for(unsigned int i = 0; i < 9; ++i)
			{
				hf_vec[i * 2 + 1] = hf_vec[i * 2 + 2] = Vec3();

				const CBone& bone = *lower_body_bones[i];
				if(i != 4)
					hf_vec[i * 2 + 1] = bone.desired_force - bone.predicted_force;
				hf_vec[i * 2 + 2] = bone.desired_torque - bone.applied_torque;
			}
			
			float hfmagsq = 0.0f;
			for(unsigned int i = 0; i < HF_DIM; ++i)
				hfmagsq += helper_force[i] * helper_force[i];
			if(hfmagsq > 0)
			{
				float hfmag = sqrtf(hfmagsq);
				float hfcoeff = min(use_hmax, hfmag) * timestep / hfmag;
				for(unsigned int i = 0; i < HF_DIM; ++i)
					helper_force[i] *= hfcoeff;

				Vec3 ubody_helper_force = hf_vec[0] / dood_mass;
				for(set<RigidBody*>::iterator iter = included_rbs.begin(); iter != included_rbs.end(); ++iter)
				{
					RigidBody& rb = **iter;
					rb.SetLinearVelocity(rb.GetLinearVelocity() + ubody_helper_force);
				}

				for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
				{	
					RigidBody& rb = *lower_body_bones[i]->rb;
					rb.SetLinearVelocity (rb.GetLinearVelocity () + hf_vec[i * 2 + 1] / rb.GetMass());
					rb.SetAngularVelocity(rb.GetAngularVelocity() + rb.GetInvMoI() * hf_vec[i * 2 + 2]);
				}
			}


#if PROFILE_CPHFT
			timer_use_results += timer.GetAndRestart();
#endif

			// scoring
			Scores cat_weights;
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
			{
				//cat_weights.bone_ori[i] = i == 4 ? 10.0f : 1.0f;
				//cat_weights.bone_pos[i] = i == 1 || i == 7 ? 100.0f : 1.0f;//10.0f;

				cat_weights.bone_ori[i] = i == 4 ? 10.0f : i <= 1 || i >= 7 ? 1.0f : 0;//0.1f;
				cat_weights.bone_pos[i] = i <= 1 || i >= 7 ? 100.0f : i != 4 ? 10.0f : 0;
			}

			//cat_weights.head_ori = 100.0f;
			//cat_weights.gun_aimvec = 10.0f;
			cat_weights.com = 1000.0f;
			//cat_weights.energy_cost = 0.00005f;
			cat_weights.prediction_error = 0.05f;//0.005f;
			cat_weights.helper_force_penalty = 100.0f / 60.0f;

			float energy_cost = 0.0f;
			for(unsigned int i = 0; i < NUM_LEG_JOINTS; ++i)
				energy_cost += leg_joints[i]->actual.ComputeMagnitudeSquared();
			energy_cost += spine1.actual.ComputeMagnitudeSquared();
			energy_cost += spine2.actual.ComputeMagnitudeSquared();

			Scores instant_scores;
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
			{
				CBone& bone = *lower_body_bones[i];
				RigidBody* rb = bone.rb;
				instant_scores.bone_ori[i] = (lb_desired_oris[i] * Quaternion::Reverse(rb->GetOrientation())).ToRVec().ComputeMagnitudeSquared();
				instant_scores.bone_pos[i] = ((bone.initial_ori * bone.local_com + bone.initial_pos) - rb->GetCenterOfMass()).ComputeMagnitudeSquared();
			}

			Quaternion head_desired_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);

			instant_scores.head_ori = (head_desired_ori * Quaternion::Reverse(head.rb->GetOrientation())).ToRVec().ComputeMagnitudeSquared();
			instant_scores.gun_aimvec = (Vec3::Normalize(gun_rb->GetOrientation() * Vec3(0, 0, 1)) - Vec3::Normalize(head_desired_ori * Vec3(0, 0, 1))).ComputeMagnitude();
			instant_scores.com = (dood_com - desired_com).ComputeMagnitudeSquared();
			instant_scores.energy_cost = energy_cost;
			instant_scores.prediction_error = mis_prediction;
			instant_scores.helper_force_penalty = use_hmax;

			instant_scores.ApplyScaleAndClamp(cat_weights);
			cat_scores += instant_scores;

#if PROFILE_CPHFT
			timer_scoring += timer.GetAndRestart();
#endif

			// update the timer, and check for when the experiment is over
			++tick_age;
			if(!experiment_done)
			{
				bool early_fail = false;

				if(experiment->elites.size() >= NUM_ELITES)
				{
					Scores quasi_scores;
					if(experiment->trial != 0)
						quasi_scores = experiment->test->scores;
					quasi_scores += cat_scores;
					quasi_scores.helper_force_penalty *= float(max_tick_age) / tick_age * NUM_TRIALS;
					if(quasi_scores.ComputeTotal() >= (**experiment->elites.rbegin()).scores.total * NUM_TRIALS)
					{
						cat_scores.helper_force_penalty *= float(max_tick_age) / tick_age;
						early_fail = true;
					}
				}

				if(tick_age >= max_tick_age || early_fail)
				{
					// end-of-test stuff
					cat_scores.ComputeTotal();

					experiment->ExperimentDone(cat_scores, ((TestGame*)dood->game_state)->debug_text, tick_age, max_tick_age, early_fail);
					experiment_done = true;
				}
			}

#if PROFILE_CPHFT
			timer_end_of_test += timer.GetAndRestart();
			timer_cphft_total += timer2.Stop();

			++counter_cphft;
#endif
		}

		GenericMatrix ComputeMatrixA(const vector<FootState*>& feet, const FrameParams& use_params, vector<float>& a_data, vector<float>& base_forces, const Vec3& ub_vel, const Vec3& angular_momentum, float ub_mass)
		{
			GenericMatrix A(NUM_JOINT_AXES, NUM_LB_BONE_FLOATS + 3, NULL);
			a_data.resize(A.wh);
			A.data = a_data.data();
			memset(A.data, 0, A.wh * sizeof(float));

			base_forces.clear();
			base_forces.resize(NUM_LOWER_BODY_BONES * 3 + 3);
			Vec3* base_forces_v = (Vec3*)base_forces.data();

			static const unsigned int joint_bone_indices[NUM_LEG_JOINTS][2] = { {1, 0}, {2, 1}, {3, 2}, {4, 3}, {7, 8}, {6, 7}, {5, 6}, {4, 5} };

//			float bone_torque_coeffs[NUM_LOWER_BODY_BONES] = { use_params.foot_torque_coeff[1], use_params.foot_torque_coeff[0], 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, use_params.foot_torque_coeff[2], use_params.foot_torque_coeff[3] };
			float bone_torque_coeffs[NUM_LOWER_BODY_BONES] = { 1, 1, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1, 1};

			// matrix to map ground forces (per foot bone) to bone forces ... thus it also maps foot forces to other bone forces
			float grbf_matrix[4][NUM_LOWER_BODY_BONES];
			memset(grbf_matrix, 0, sizeof(grbf_matrix));

			Mat3 bone_lmat[NUM_LOWER_BODY_BONES];
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
			{
				Vec3 v = lower_body_bones[i]->rb->GetOrientation() * use_params.xprod_radii[i];
				bone_lmat[i] = Mat3(	   0,   v.z,  -v.y,
										-v.z,     0,   v.x,
										 v.y,  -v.x,     0	);
			}

			Vec3 bone_angular_momentum[NUM_LOWER_BODY_BONES + 1];
			for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
			{
				const RigidBody* rb = lower_body_bones[i]->rb;
				bone_angular_momentum[i] = Mat3(rb->GetTransformedMassInfo().moi) * rb->GetAngularVelocity();
			}
			bone_angular_momentum[NUM_LOWER_BODY_BONES] = angular_momentum;

			for(unsigned int i = 0; i < NUM_LEG_JOINTS; ++i)
			{
				Vec3 avg_pos = leg_joints[i]->sjc->ComputeAveragePosition();
				Mat3 mat = leg_joints[i]->oriented_axes.Transpose();
				unsigned int k0 = i * 3;
				unsigned int bone_a = joint_bone_indices[i][0];
				unsigned int bone_b = joint_bone_indices[i][1];

				unsigned int a0 = bone_a * 6;
				unsigned int b0 = bone_b * 6;

				// joint torques to bone torques
				float acoeff = bone_torque_coeffs[bone_a];
				float bcoeff = bone_torque_coeffs[bone_b];
				const float* m = mat.values;

				for(unsigned int j = 0; j < 3; ++j)
					for(unsigned int k = 0; k < 3; ++k, ++m)
					{
						A[a0 + j][k0 + k] += *m * acoeff;
						A[b0 + j][k0 + k] -= *m * bcoeff;
					}

				Mat3 lmata = bone_lmat[bone_a] * mat;
				Mat3 lmatb = bone_lmat[bone_b] * mat;
				for(unsigned int ob = 0; ob <= NUM_LOWER_BODY_BONES; ++ob)
				{
					float obcoeff_a = use_params.xprod_frac[ob * NUM_LOWER_BODY_BONES + bone_a];
					float obcoeff_b = use_params.xprod_frac[ob * NUM_LOWER_BODY_BONES + bone_b];

					Mat3 lmat = lmata * obcoeff_a - lmatb * obcoeff_b;
					float* L = lmat.values;

					for(unsigned int j = 0; j < 3; ++j)
						for(unsigned int k = 0; k < 3; ++k, ++L)
						{
							unsigned int index = ob == NUM_LOWER_BODY_BONES ? NUM_LOWER_BODY_BONES * 6 + j : ob * 6 + 3 + j;
							A[index][k0 + k] += *L;
						}

					base_forces_v[ob] += (lmat * mat.Transpose()) * bone_angular_momentum[ob] * (inv_timestep);
				}
			}

			

			// base bone force to expect when joint torques are zero
			for(unsigned int i = 0; i <= NUM_LOWER_BODY_BONES; ++i)
			{
				CBone& bone = i == NUM_LOWER_BODY_BONES ? torso2 : *lower_body_bones[i];

				Vec3 vel = i == NUM_LOWER_BODY_BONES ? ub_vel : bone.rb->GetLinearVelocity();
				Vec3 measured_force = (vel - bone.old_vel) * (inv_timestep * (i == NUM_LOWER_BODY_BONES ? ub_mass : bone.rb->GetMass()));
				bone.old_vel = vel;
				bone.prediction_error = measured_force - bone.predicted_force;
				bone.perr_integral += bone.prediction_error * timestep;

				unsigned int i0 = i * 6 + 3;

				float* bf_begin = base_forces.data() + i * 3;
				Vec3& bf_begin_vec = *(Vec3*)bf_begin;

				bf_begin_vec = bone.prediction_error * use_params.prediction_error_pcoeff[i] + bone.perr_integral * use_params.prediction_error_icoeff[i];
				//bf_begin_vec += measured_force;
				
				for(unsigned int j = 0; j < 4; ++j)
				{
					float grbf_coeff = grbf_matrix[j][i];
					//bf_begin_vec += feet[j]->net_force * grbf_coeff;
				}

				bf_begin_vec += Vec3(0, -9.8f * use_params.gravity_coeff[i == NUM_LOWER_BODY_BONES ? 4 : i] * (i == NUM_LOWER_BODY_BONES ? ub_mass : bone.rb->GetMass()), 0);
			}

			return A;
		}
		
		void ComputeConstraintRowMagnitudes(GenericMatrix& all_constraints, unsigned int cmagsq_index) const
		{
			for(unsigned int i = 0; i < all_constraints.h; ++i)
			{
				float* row = all_constraints[i];
				float magsq = 0.0f;
				for(unsigned int j = 0; j < NUM_JOINT_AXES; ++j)
					magsq += row[j] * row[j];
				row[cmagsq_index] = magsq;
			}
		}

		void SolveForOptimalF(
			const FrameParams& use_params, const GenericMatrix& A, const float b[NUM_LB_BONE_FLOATS + 3], float f[NUM_JOINT_AXES], float force_errors[NUM_LB_BONE_FLOATS + 3]) const
		{
#if PROFILE_QP_SOLVER
			ProfilingTimer timer, timer2;
			timer2.Start();
			timer.Start();
#endif

			// collect all of the constraints into a single array
			static const unsigned int ac_w = NUM_JOINT_AXES + 3, ac_h = NUM_JOINT_AXES + NUM_JOINT_AXES;
			GenericMatrix all_constraints(ac_w, ac_h, NULL);
			float acd[ac_w * ac_h];
			all_constraints.data = acd;
			all_constraints.h = 0;

			float torque_mins[NUM_JOINT_AXES];
			float torque_maxs[NUM_JOINT_AXES];
			for(unsigned int i = 0; i < NUM_LEG_JOINTS; ++i)
			{
				const SkeletalJointConstraint* sjc = leg_joints[i]->sjc;
				memcpy(torque_mins + i * 3, &sjc->min_torque, sizeof(float) * 3);
				memcpy(torque_maxs + i * 3, &sjc->max_torque, sizeof(float) * 3);
			}

			static const unsigned int minbound_index = NUM_JOINT_AXES;
			static const unsigned int maxbound_index = NUM_JOINT_AXES + 1;
			static const unsigned int cmagsq_index   = NUM_JOINT_AXES + 2;

#if PROFILE_QP_SOLVER
			timer_jt_constraints += timer.GetAndRestart();
#endif

			// compute magnitudes of all of those constraint vectors
			ComputeConstraintRowMagnitudes(all_constraints, cmagsq_index);

#if PROFILE_QP_SOLVER
			timer_constraint_mags += timer.GetAndRestart();
#endif

			// initialize the search value to zero
			memset(f, 0, sizeof(float) * NUM_JOINT_AXES);

			float weights[NUM_LB_BONE_FLOATS + 3];
			float* wptr = weights;
			for(unsigned int i = 0; i <= NUM_LOWER_BODY_BONES; ++i)
			{
				if(i != NUM_LOWER_BODY_BONES)
				{
					float angular = powf(2.0f, use_params.qp_bone_weights_angular[i]);
					*(wptr++) = angular;
					*(wptr++) = angular;
					*(wptr++) = angular;
				}

				float linear  = powf(2.0f, use_params.qp_bone_weights_linear[i]);
				*(wptr++) = linear;
				*(wptr++) = linear;
				*(wptr++) = linear;
			}

			memset(f, 0, sizeof(float) * NUM_JOINT_AXES);
			DoGradientSearch<NUM_LB_BONE_FLOATS + 3>(10, torque_mins, torque_maxs, all_constraints, A, b, weights, f, force_errors, 0, 0);

			ScoreProposedF<NUM_LB_BONE_FLOATS + 3>(A, b, f, weights, force_errors);

#if PROFILE_QP_SOLVER
			timer_gradient_descent += timer.Stop();
			timer_qp_total += timer2.Stop();

			++counter_qp;
#endif
		}

		// searches for f to minimize ||Af - b||^2_W, and puts the final Af - b in h
		template<unsigned int B> void DoGradientSearch(unsigned int iterations, const float iv_mins[NUM_JOINT_AXES], const float iv_maxs[NUM_JOINT_AXES], const GenericMatrix& all_constraints, const GenericMatrix& A, const float b[B], const float W[B], float f[NUM_JOINT_AXES], float h[B], unsigned int gradient_constraints_begin, unsigned int gradient_constraints_end) const
		{
#if DEBUG_GRADIENT_SEARCH_PROGRESS
			Debug(((stringstream&)(stringstream() << "age = " << tick_age << "; starting gradient search, max iterations = " << iterations << endl)).str());
#endif

			// TODO: undo this bullshit
			//for(int i = 0; i < NUM_JOINT_AXES; ++i)
			//	f[i] = ((iv_maxs[i] - iv_mins[i]) * experiment->GetSubtest().jt[i] + iv_mins[i]) * 0.25f;
			//iterations = 1;

			float old_y1;
			// do a bunch of iterations of gradient descent
			for(unsigned int i = 0; i < iterations; ++i)
			{
				float y1 = ScoreProposedF<B>(A, b, f, W, h);
				//for(int j = 0; j < B; ++j)
				//	if(_isnan(b[j]))
				//		Debug(((stringstream&)(stringstream() << "b[" << j << "] is NaN" << endl)).str());

				if(i != 0 && old_y1 - y1 < 0.0001f)
				{
#if DEBUG_GRADIENT_SEARCH_PROGRESS
					Debug(((stringstream&)(stringstream() << "\ti = " << i << "; y1 = " << y1 << "; early return 1" << endl)).str());
#endif
					return;
				}

				// compute the gradient, using calculus! we do u-substitution where u = Af-b, aka what's already stored in force_errors
				float gradient[NUM_JOINT_AXES];
				memset(gradient, 0, sizeof(float) * NUM_JOINT_AXES);
				
				float* gend = gradient + NUM_JOINT_AXES;
				for(const float *uptr = h, *uend = uptr + B, *aptr = A.data, *wptr = W; uptr != uend; ++uptr, ++wptr)
				{
					float two_w_u = 2.0f * *wptr * *uptr;
					for(float* gptr = gradient; gptr != gend; ++gptr, ++aptr)
						*gptr += two_w_u * *aptr;
				}

				float gr_magsq = 0.0f;
				for(unsigned int j = 0; j < NUM_JOINT_AXES; ++j)
				{
					float& gj = gradient[j];
					gr_magsq += gj * gj;
				}

				float original_grmag = sqrtf(gr_magsq);

				// make sure the gradient satisfies the null space constraints
				MakeGradientSatisfyNullSpaceConstraints(all_constraints, gradient_constraints_begin, gradient_constraints_end, gradient);

				gr_magsq = 0.0f;
				for(unsigned int j = 0; j < NUM_JOINT_AXES; ++j)
				{
					float& gj = gradient[j];
					if(gj > 0 ? f[j] >= iv_maxs[j] : f[j] <= iv_mins[j])
						gj = 0.0f;
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
				float best_f[NUM_JOINT_AXES];
				memcpy(best_f, f, sizeof(float) * NUM_JOINT_AXES);

				float* fend = f + NUM_JOINT_AXES;
				for(float *fptr = f, *gptr = gradient; fptr != fend; ++fptr, ++gptr)
					*fptr -= *gptr;
				float y2 = ScoreProposedF<B>(A, b, f, W, h);

				for(float *fptr = f, *gptr = gradient; fptr != fend; ++fptr, ++gptr)
					*fptr -= *gptr;
				float y3 = ScoreProposedF<B>(A, b, f, W, h);

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
				
				for(float *fptr = f, *bptr = best_f, *gptr = gradient; fptr != fend; ++fptr, ++bptr, ++gptr)
					*fptr = *bptr - *gptr * extremum_x;

				// make sure we stay within constraints
				unsigned int fails = 0;
				while(fails < 10 && CheckConstraintsTryFixSome(iv_mins, iv_maxs, all_constraints, f) > 0.000001f)
					++fails;

#if DEBUG_GRADIENT_SEARCH_PROGRESS
				Debug(((stringstream&)(stringstream() << "\ti = " << i << "; y1 = " << y1 << "; gr mag = " << original_grmag << "; gr' mag = " << sqrtf(gr_magsq) << "; fails = " << fails << endl)).str());
#endif

				old_y1 = y1;
			}
		}

		// h = Af - b = force errors
		template<unsigned int B> float ScoreProposedF(const GenericMatrix& A, const float b[B], const float f[NUM_JOINT_AXES], const float W[B], float h[B]) const
		{
#if PROFILE_SCORE_PROPOSED_F
			ProfilingTimer timer;
			timer.Start();
#endif
			JointTorquesToBoneTorqueErrors<B>(A, f, b, h);
			float result = ScoreBoneTorqueErrors<B>(h, W);

#if PROFILE_SCORE_PROPOSED_F
			timer_score_proposed_f += timer.Stop();
			++counter_score_f;
#endif

			return result;
		}

		void MakeGradientSatisfyNullSpaceConstraints(const GenericMatrix& constraints, unsigned int from, unsigned int to, float gradient[NUM_JOINT_AXES]) const
		{
			static const unsigned int minbound_index = NUM_JOINT_AXES;
			static const unsigned int maxbound_index = NUM_JOINT_AXES + 1;
			static const unsigned int cmagsq_index   = NUM_JOINT_AXES + 2;

			float errsq;
			unsigned int fails = 0;
			while(fails < 10)
			{
				errsq = 0.0f;
				for(unsigned int i = from; i < to; ++i)
				{
					const float* constraint = constraints[i];

					float dot = 0.0f;
					for(unsigned int j = 0; j < NUM_JOINT_AXES; ++j)
						dot += constraint[j] * gradient[j];

					float cmagsq = constraint[cmagsq_index];

					float frac = dot / cmagsq;
					for(unsigned int j = 0; j < NUM_JOINT_AXES; ++j)
						gradient[j] -= constraint[j] * frac;
		
					errsq += dot * dot / cmagsq;
				}
				if(errsq < 0.000001f)
					break;
				else
					++fails;
			}
		}

		// returns magnitude squared of all the errors that had to be corrected
		float CheckConstraintsTryFixSome(const float iv_mins[NUM_JOINT_AXES], const float iv_maxs[NUM_JOINT_AXES], const GenericMatrix& constraints, float f[NUM_JOINT_AXES]) const
		{
			static const unsigned int minbound_index = NUM_JOINT_AXES;
			static const unsigned int maxbound_index = NUM_JOINT_AXES + 1;
			static const unsigned int cmagsq_index   = NUM_JOINT_AXES + 2;

			float errsq = 0.0f;

			// check ranges of the independent variables (f) against +/- fmax; no need to do dot product for these
			for(unsigned int i = 0; i < NUM_JOINT_AXES; ++i)
			{
				float& fi = f[i];
				if(fi < iv_mins[i])
				{
					errsq += (fi + iv_mins[i]) * (fi - iv_mins[i]);
					fi = iv_mins[i];
				}
				else if(fi > iv_maxs[i])
				{
					errsq += (fi - iv_maxs[i]) * (fi - iv_maxs[i]);
					fi = iv_maxs[i];
				}
			}

			// now check the more complicated constraints which require computing a dot product
			for(unsigned int i = 0; i < constraints.h; ++i)
			{
				const float* constraint = constraints[i];

				float dot = 0.0f;
				for(unsigned int j = 0; j < NUM_JOINT_AXES; ++j)
					dot += constraint[j] * f[j];

				float minb   = constraint[minbound_index];
				float maxb   = constraint[maxbound_index];
				float cmagsq = constraint[cmagsq_index];

				if(dot < minb)
				{
					float dif = minb - dot;
					errsq += dif * dif / cmagsq;
				
					float frac = (minb - dot) / cmagsq;
					for(unsigned int j = 0; j < NUM_JOINT_AXES; ++j)
						f[j] += constraint[j] * frac;
				}
				else if(dot > maxb)
				{
					float dif = maxb - dot;
					errsq += dif * dif / cmagsq;

					float frac = dif / cmagsq;
					for(unsigned int j = 0; j < NUM_JOINT_AXES; ++j)
						f[j] += constraint[j] * frac;
				}
			}

			return errsq;
		}



		// h = Af - b = bone torque errors
		template<unsigned int B> void JointTorquesToBoneTorqueErrors(const GenericMatrix& A, const float f[NUM_JOINT_AXES], const float b[B], float h[B]) const
		{
			const float* fend = f + NUM_JOINT_AXES;
			const float* aptr = A.data;
			const float* bptr = b;

			for(float *hptr = h, *hend = hptr + B; hptr != hend; ++hptr, ++bptr)
			{
				*hptr = -*bptr;
				for(const float *fptr = f; fptr != fend; ++fptr, ++aptr)
					*hptr += *fptr * *aptr;
			}
		}

		template<unsigned int B> float ScoreBoneTorqueErrors(const float h[B], const float W[B]) const
		{
			float tot = 0.0f;
			for(const float *hptr = h, *hend = h + B, *wptr = W; hptr != hend; ++hptr, +wptr)
				tot += *hptr * *hptr * *wptr;
			return tot;
		}

		struct NoTouchy : public ContactCallback
		{
			Imp* imp;
			void OnContact(const ContactPoint& contact) { /*++imp->goal_error_cost;*/ }
			void AfterResolution(const ContactPoint& cp) { }
		} no_touchy;
	};



	/*
	 * Soldier methods
	 */
	Soldier::Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		imp(NULL),
		gun_hand_bone(NULL),
		p_ag(NULL),
		walk_pose(NULL),
		jet_bones(),
		jet_fuel(1.0f),
		jet_start_sound(NULL),
		jet_loop_sound(NULL),
		jet_loop(NULL)
	{
		use_cheaty_ori = false;

		//yaw = Random3D::Rand(float(M_PI) * 2.0f);

		gun_hand_bone = character->skeleton->GetNamedBone("r grip");

		Cache<SoundBuffer>* sound_cache = game_state->content->GetCache<SoundBuffer>();
		jet_start_sound = sound_cache->Load("jet_start");
		jet_loop_sound  = sound_cache->Load("jet_loop" );

		standing_callback.angular_coeff = 1.0f;

		ragdoll_timer = 3600.0f;

		p_ag = new PoseAimingGun();
		//posey->active_poses.push_back(p_ag);

		imp = new Imp();
	}

	void Soldier::InnerDispose()
	{
		Dood::InnerDispose();

		if(imp) { delete imp; imp = NULL; }
	}

	void Soldier::DoJumpControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward)
	{
		float timestep = time.elapsed;

		bool can_recharge = true;
		bool jetted = false;
		if(control_state->GetBoolControl("jump"))
		{
			if(standing_callback.IsStanding() && time.total > jump_start_timer)							// jump off the ground
			{
				standing_callback.ApplyVelocityChange(Vec3(0, jump_speed, 0));

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
						fly_accel_vec += horizontal_accel * fly_accel_lateral;

						imp->desired_jp_accel = fly_accel_vec;

#if !ENABLE_NEW_JETPACKING
						// TODO: remove this once similar functionality is moved to Soldier::Imp::ResolveJetpackOutput
						float total_mass = 0.0f;
						for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
							total_mass += (*iter)->GetMassInfo().mass;
						Vec3 apply_force = fly_accel_vec * total_mass;

						for(vector<RigidBody*>::iterator iter = jet_bones.begin(); iter != jet_bones.end(); ++iter)
							(*iter)->ApplyCentralForce(apply_force / float(jet_bones.size()));
#endif
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

		imp->jetpacking = jetted;

		if(!jetted && jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}

		if(can_recharge)
			jet_fuel = min(jet_fuel + fuel_refill_rate * timestep, 1.0f);
	}

	void Soldier::PreUpdatePoses(const TimingInfo& time) { imp->Update(this, time); }

	void Soldier::PhysicsToCharacter()
	{
		Dood::PhysicsToCharacter();

		// position and orient the gun
		if(equipped_weapon != NULL)
		{
			Gun* gun = dynamic_cast<Gun*>(equipped_weapon);

			if(gun != NULL && gun->rigid_body != NULL)
			{
				RigidBody* gun_rb = gun->rigid_body;
				Mat4 gun_xform = gun_rb->GetTransformationMatrix();

				equipped_weapon->gun_xform = gun_xform;
				equipped_weapon->sound_pos = equipped_weapon->pos = gun_xform.TransformVec3_1(0, 0, 0);
				equipped_weapon->sound_vel = equipped_weapon->vel = vel;
			}
			else if(gun_hand_bone != NULL)
			{
				equipped_weapon->gun_xform = Mat4::Translation(pos) * gun_hand_bone->GetTransformationMatrix() * Mat4::Translation(gun_hand_bone->rest_pos);
				equipped_weapon->sound_pos = equipped_weapon->pos = equipped_weapon->gun_xform.TransformVec3_1(0, 0, 0);
				equipped_weapon->sound_vel = equipped_weapon->vel = vel;
			}
		}
	}

	void Soldier::RegisterFeet()
	{
		feet.push_back(new SoldierFoot(Bone::string_table["l toe" ], Vec3( 0.24f, 0.00f,  0.185f )));
		feet.push_back(new SoldierFoot(Bone::string_table["l heel"], Vec3( 0.23f, 0.00f, -0.07f  )));
		
		feet.push_back(new SoldierFoot(Bone::string_table["r toe" ], Vec3(-0.24f, 0.00f,  0.185f )));
		feet.push_back(new SoldierFoot(Bone::string_table["r heel"], Vec3(-0.23f, 0.00f, -0.07f  )));
	}

	void Soldier::Update(const TimingInfo& time)
	{
#if DIE_AFTER_ONE_SECOND
		if(time.total > 1.0f)
			Die(Damage());
#endif

		Dood::Update(time);
	}

	void Soldier::Die(const Damage& cause)
	{
		if(jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}

		Dood::Die(cause);
	}

	void Soldier::Spawned()
	{
		Dood::Spawned();

		if(!is_valid)
			return;		

		p_ag->torso2_ori = Quaternion::FromRVec(0, -(yaw + torso2_yaw_offset), 0);

		unsigned int lshoulder_name = Bone::string_table["l shoulder"], rshoulder_name = Bone::string_table["r shoulder"];
		for(unsigned int i = 0; i < character->skeleton->bones.size(); ++i)
			if(character->skeleton->bones[i]->name == lshoulder_name || character->skeleton->bones[i]->name == rshoulder_name)
				jet_bones.push_back(bone_to_rbody[i]);
	}

	void Soldier::DeSpawned()
	{
		Dood::DeSpawned();

		if(jet_loop != NULL)
		{
			jet_loop->StopLooping();
			jet_loop->SetLoudness(0.0f);
			jet_loop = NULL;
		}
	}

	void Soldier::DoInitialPose()
	{
		const Subtest& subtest = experiment->GetSubtest();

		Quaternion yaw_ori = Quaternion::FromAxisAngle(0, 1, 0, -yaw);
		//Quaternion lb_oris[NUM_LOWER_BODY_BONES];
		//const WalkKeyframe& frame = walk_keyframes[subtest.initial_frame];
		//for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
		//	lb_oris[i] = yaw_ori * Quaternion::FromRVec(frame.desired_oris[i]);
		pos.y += subtest.initial_y;

		pitch += subtest.initial_pitch;

		Dood::DoInitialPose();

		Quaternion p, t1, t2;
		imp->GetDesiredTorsoOris(this, p, t1, t2);

		posey->skeleton->GetNamedBone( "pelvis"  )->ori = p;
		posey->skeleton->GetNamedBone( "torso 1" )->ori = t1 * Quaternion::Reverse(p);
		posey->skeleton->GetNamedBone( "torso 2" )->ori = t2 * Quaternion::Reverse(t1);

#if 0
		posey->skeleton->GetNamedBone( "l toe"   )->ori = Quaternion::Reverse(lb_oris[1]) * lb_oris[0];
		posey->skeleton->GetNamedBone( "l heel"  )->ori = Quaternion::Reverse(lb_oris[2]) * lb_oris[1];
		posey->skeleton->GetNamedBone( "l leg 2" )->ori = Quaternion::Reverse(lb_oris[3]) * lb_oris[2];
		posey->skeleton->GetNamedBone( "l leg 1" )->ori = Quaternion::Reverse(p) * lb_oris[3];
		posey->skeleton->GetNamedBone( "r toe"   )->ori = Quaternion::Reverse(lb_oris[7]) * lb_oris[8];
		posey->skeleton->GetNamedBone( "r heel"  )->ori = Quaternion::Reverse(lb_oris[6]) * lb_oris[7];
		posey->skeleton->GetNamedBone( "r leg 2" )->ori = Quaternion::Reverse(lb_oris[5]) * lb_oris[6];
		posey->skeleton->GetNamedBone( "r leg 1" )->ori = Quaternion::Reverse(p) * lb_oris[5];
#else
		if(subtest.initial_frame == 0)
		{
			posey->skeleton->GetNamedBone( "l leg 1" )->ori = Quaternion::FromRVec( -0.05f, 0.1f,  0.05f );
			posey->skeleton->GetNamedBone( "l leg 2" )->ori = Quaternion::FromRVec(  0.1f,  0,     0     );
			posey->skeleton->GetNamedBone( "l heel"  )->ori = Quaternion::FromRVec( -0.05f, 0.1f, -0.05f );

			posey->skeleton->GetNamedBone( "r leg 1" )->ori = Quaternion::FromRVec(  0,    -0.1f,  0     );
			posey->skeleton->GetNamedBone( "r heel"  )->ori = Quaternion::FromRVec(  0,    -0.1f,  0     );
		}
		else
		{
			posey->skeleton->GetNamedBone( "l leg 1" )->ori = Quaternion::FromRVec( -0.1f,  0.2f,  0     );
			posey->skeleton->GetNamedBone( "l leg 2" )->ori = Quaternion::FromRVec(  0.2f,  0,     0     );
			posey->skeleton->GetNamedBone( "l heel"  )->ori = Quaternion::FromRVec( -0.1f,  0.2f,  0     );

			posey->skeleton->GetNamedBone( "r leg 1" )->ori = Quaternion::FromRVec(  0,    -0.1f,  0     );
			posey->skeleton->GetNamedBone( "r heel"  )->ori = Quaternion::FromRVec(  0,     0,     0     );
		}
#endif

#if 0
		Quaternion qlist[6] =
		{
			Quaternion( 0.98935f,   0.058987f,    0.124063f,    -0.0481096f   ),
			Quaternion( 1.0f,      -0.0001091f,   0.000762187f,  0.000103048f ),
			Quaternion( 0.985989f, -0.0697347f,   0.148507f,     0.0301456f   ),
			Quaternion( 0.995083f, -0.017937f,   -0.0915855f,   -0.033182f    ),
			Quaternion( 0.999651f,  0.022753f,   -0.0133616f,   -0.00111608f  ),
			Quaternion( 0.996213f, -0.00356901f,  0.0807469f,    0.0320568f   ),
		};
		posey->skeleton->GetNamedBone( "l leg 1" )->ori = qlist[0];
		posey->skeleton->GetNamedBone( "l leg 2" )->ori = qlist[1];
		posey->skeleton->GetNamedBone( "l heel"  )->ori = qlist[2];
		posey->skeleton->GetNamedBone( "r leg 1" )->ori = qlist[3];
		posey->skeleton->GetNamedBone( "r leg 2" )->ori = qlist[4];
		posey->skeleton->GetNamedBone( "r heel"  )->ori = qlist[5];
#endif

		PreparePAG(TimingInfo(), t2);
	}

	void Soldier::PreparePAG(const TimingInfo& time, const Quaternion& t2ori)
	{
		p_ag->yaw   = yaw;
		p_ag->pitch = pitch;
		p_ag->torso2_ori = t2ori;
		p_ag->UpdatePose(time);

		for(boost::unordered_map<unsigned int, BoneInfluence>::iterator iter = p_ag->bones.begin(); iter != p_ag->bones.end(); ++iter)
			posey->skeleton->GetNamedBone(iter->first)->ori = Quaternion::FromRVec(iter->second.ori);

		posey->skeleton->InvalidateCachedBoneXforms();
	}




	bool Soldier::IsExperimentDone() const { return imp->init && imp->experiment_done; }

	void Soldier::LoadExperimentData()     { experiment = new Experiment(); }

	void Soldier::SaveExperimentData()     { if(experiment) { delete experiment; experiment = NULL; } }




	/*
	 * SoldierFoot methods
	 */
	SoldierFoot::SoldierFoot(unsigned int posey_id, const Vec3& ee_pos) : FootState(posey_id, ee_pos) { }

	Quaternion SoldierFoot::OrientBottomToSurface(const Vec3& normal) const
	{
		static const Vec3 dirs[2] = { Vec3(0, 0, 1), Vec3(1, 0, 0) };

		Quaternion foot_ori = body->GetOrientation();
		for(unsigned int i = 0; i < 2; ++i)
		{
			Vec3 dir   = foot_ori * dirs[i];
			Vec3 level = dir - normal * Vec3::Dot(dir, normal);
			Vec3 cross = Vec3::Cross(dir, level);

			float level_mag = level.ComputeMagnitude();
			float cross_mag = cross.ComputeMagnitude();

			if(level_mag != 0 && cross_mag != 0 && fabs(cross_mag) <= fabs(level_mag))
				foot_ori = Quaternion::FromRVec(cross * (asinf(cross_mag / level_mag) / cross_mag)) * foot_ori;
		}

		return foot_ori;
	}
}
