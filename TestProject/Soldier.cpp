#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#include "Particle.h"

#define PROFILE_CPHFT					1
#define PROFILE_ANY_CPHFT				(PROFILE_CPHFT)

#define DIE_AFTER_ONE_SECOND			0

#define ENABLE_NEW_JETPACKING			1


#define MAX_TICK_AGE					60
#define AIMMOVE_MAX_AGE					5

#define NUM_STRICT_INPUTS				78
#define NUM_SIMPLE_FEEDBACKS			5
#define NUM_NODE_COMMS					35
#define NUM_OUTPUTS						(3 + NUM_NODE_COMMS)
#define NUM_INITIAL_VALUES				NUM_SIMPLE_FEEDBACKS
#define MIDDLE_LAYER_SIZE				20
#define OUTPUT_LAYER_SIZE				(NUM_OUTPUTS + NUM_SIMPLE_FEEDBACKS)
#define BATCH_ITERATIONS				8

#define NUM_LOWER_BODY_BONES			9
#define NUM_LB_BONE_FLOATS				(NUM_LOWER_BODY_BONES * 6)

#define NUM_LEG_JOINTS					8

#define GENERATE_SUBTEST_LIST_EVERY		1		// 1 = game state; 2 = generation; 3 = candidate

#define NUM_SUBTESTS					1
#define TRIALS_PER_SUBTEST				20
#define NUM_TRIALS						(NUM_SUBTESTS * TRIALS_PER_SUBTEST)

#define NUM_ELITES						10
#define MUTANTS_PER_ELITE				0
#define CROSSOVERS_PER_PAIR				2

#define MUTATION_COUNT					10000
#define MUTATION_SCALE					0.005f
#define NN_COEFFS_RANGE					4.0f

#define INITIAL_NONZERO_DIAGONAL		1.0f

//#define GENERATION_SIZE				29
//#define LINE_SEARCH_COUNT				19

#define NUM_LAYERS						6		// middle layers and output layer

namespace Test
{
	/*
	 * Soldier constants
	 */
	static const float jump_speed         = 0;//4.0f;

	static const float fly_accel_up       = 15.0f;
	static const float fly_accel_lateral  = 8.0f;

	static const float fuel_spend_rate    = 0.5f;
	static const float fuel_refill_rate   = 0.4f;
	static const float jump_to_fly_delay  = 0;//0.3f;

	static const float torso2_yaw_offset  = 0.5f;


#if PROFILE_ANY_CPHFT
#if PROFILE_CPHFT
	static float timer_init					= 0.0f;
	static float timer_reset				= 0.0f;
	static float timer_massinfo				= 0.0f;
	static float timer_ub_stuff				= 0.0f;

	static float timer_scoring				= 0.0f;
	static float timer_end_of_test			= 0.0f;
	static float timer_cphft_total			= 0.0f;

	static unsigned int counter_cphft		= 0;
#endif

	static void DebugCPHFTProfilingData()
	{
#if PROFILE_CPHFT
			Debug(((stringstream&)(stringstream() << "total for " << counter_cphft << " calls to Soldier::Imp::Update = " << timer_cphft_total << endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "init =\t\t\t\t\t"			<< timer_init				<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "reset =\t\t\t\t\t"		<< timer_reset				<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "massinfo =\t\t\t\t"		<< timer_massinfo			<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "ub_stuff =\t\t\t\t"		<< timer_ub_stuff			<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "scoring =\t\t\t\t"		<< timer_scoring			<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "end_of_test =\t\t\t"		<< timer_end_of_test		<< endl)).str());
			Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<<
				timer_init + timer_reset + timer_massinfo + timer_ub_stuff + timer_scoring + timer_end_of_test << endl)).str());
#endif
	}
#endif


	struct Scores
	{
		static const unsigned int num_floats;

		float first_member;
		float lowness;
		//float ori_error[19];
		float energy_cost;
		//float kinetic_energy;
		//float feet_verror;
		//float feet_rerror;
		//float head_error, t2_error, t1_error, pelvis_error;
		//float gun_error, gun_yerror;
		//float com_errx, com_erry, com_errz;
		//float early_fail_head;
		//float early_fail_feet;

		float total;

		Scores() { memset(Begin(), 0, sizeof(float) * num_floats); }

		float* Begin() { return &first_member; }
		float* End()   { return &first_member + num_floats; }
		const float* Begin() const { return &first_member; }
		const float* End()   const { return &first_member + num_floats; }

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

	struct Subtest;

	struct SparseNet
	{
		vector<vector<float>> nodes;
		unsigned int min_input, max_input;

		SparseNet(unsigned int num_nodes, unsigned int max_input) : nodes(num_nodes), min_input(0), max_input(max_input)
		{
			for(unsigned int i = 0; i < num_nodes; ++i)
				nodes[i] = vector<float>(max_input);
		}

		void Evaluate(vector<float>& data) const
		{
			unsigned int initial_size = data.size();
			data.resize(data.size() + nodes.size());
			for(unsigned int i = 0; i < nodes.size(); ++i)
			{
				const vector<float>& refs = nodes[i];
				float tot = 0.0f;
				for(unsigned int j = min_input; j < refs.size(); ++j)
					tot += data[j] * refs[j];
				data[initial_size + i] = tanhf(tot);
			}
		}

		static unsigned int Read(istream& s, SparseNet*& result, unsigned int override_nodes, unsigned int override_maxinput)
		{
			BinaryChunk chunk;
			chunk.Read(s);

			if(chunk.GetName() != "SPARSENT")
				return 1;

			istringstream nnss(chunk.data);

			static unsigned long zeroes = 0, nonzeroes = 0;

			unsigned int num_nodes = ReadUInt32(nnss);
			result = new SparseNet(num_nodes, 0);
			for(unsigned int i = 0; i < num_nodes; ++i)
			{
				unsigned int num_refs = ReadUInt32(nnss);

				vector<float> refs(num_refs);
				for(unsigned int j = 0; j < num_refs; ++j)
				{
					refs[j] = ReadSingle(nnss);
					if(refs[j] == 0)
						++zeroes;
					else
						++nonzeroes;
				}
				refs.resize(override_maxinput);

				result->nodes[i] = refs;
			}

			result->nodes.resize(override_nodes);
			result->max_input = override_maxinput;

			if(nnss.bad())
				return 2;

			Debug(((stringstream&)(stringstream() << "Zeroes = " << zeroes << "; nonzeroes = " << nonzeroes << endl)).str());

			return 0;
		}

		unsigned int Write(ostream& o)
		{
			stringstream nnss;
			WriteUInt32(nodes.size(), nnss);
			for(const vector<float> *nptr = nodes.data(), *nend = nptr + nodes.size(); nptr != nend; ++nptr)
			{
				unsigned int sz = nptr->size();
				WriteUInt32(sz, nnss);
				for(unsigned int j = 0; j < nptr->size(); ++j)
					WriteSingle((*nptr)[j], nnss);
			}

			BinaryChunk chunk("SPARSENT");
			chunk.data = nnss.str();
			chunk.Write(o);

			return 0;
		}

		void Randomize(unsigned int count, float scale)
		{
			do
			{
				vector<float>& refs = nodes[Random3D::RandInt(nodes.size())];
				Randomize(refs[Random3D::RandInt(min_input, refs.size() - 1)], scale);
				
			} while(Random3D::RandInt() % count != 0);
		}

		static void Randomize(float& value, float scale)
		{
			value += Random3D::Rand(-NN_COEFFS_RANGE * scale, NN_COEFFS_RANGE * scale);
			value = max(-NN_COEFFS_RANGE, min(NN_COEFFS_RANGE, value));
		}
	};

	struct GABrain
	{
		SparseNet* nn[NUM_LAYERS];
		float initial_values[NUM_INITIAL_VALUES];

		Scores scores;

		unsigned int id, pa, pb;			// my id, and my parents' ids

		vector<Subtest> subtests;

		vector<unsigned int> trials_not_started;
		vector<unsigned int> trials_not_finished;
		vector<unsigned int> trials_finished;

		unsigned int tick_tot;

		bool fail_early;

		GABrain(unsigned int id, unsigned int pa, unsigned int pb) : scores(), id(id), pa(pa), pb(pb), fail_early(false) { SetZero(); }
		GABrain() : scores(), fail_early(false) { SetZero(); }

		void SetZero()
		{
			for(unsigned int i = 0; i < NUM_LAYERS; ++i)
				nn[i] = NULL;
			for(unsigned int i = 0; i < NUM_INITIAL_VALUES; ++i)
				initial_values[i] = 0.0f;
		}

		~GABrain()
		{
			for(unsigned int i = 0; i < NUM_LAYERS; ++i)
				if(nn[i] != NULL)
				{
					delete nn[i];
					nn[i] = NULL;
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
			for(unsigned int i = 0; i < NUM_LAYERS; ++i)
				result->nn[i] = new SparseNet(*nn[i]);
			return result;
		}

		void Randomize(unsigned int count, float scale)
		{
			nn[NUM_LAYERS - 1]->min_input = NUM_STRICT_INPUTS;

			unsigned int total_coeffs = 0;
			unsigned int layer_sizes[NUM_LAYERS];
			total_coeffs += NUM_INITIAL_VALUES;
			for(unsigned int i = 0; i < NUM_LAYERS; ++i)
			{
				layer_sizes[i] = nn[i]->nodes.size() * (nn[i]->max_input - nn[i]->min_input);
				total_coeffs += layer_sizes[i];
			}

			static bool printed = false;
			if(!printed)
			{
				Debug(((stringstream&)(stringstream() << "total coeffs = " << total_coeffs << "; rand max = " << RAND_MAX << endl)).str());
				printed = true;
			}

			do
			{
				unsigned int choice;
				do
				{
					choice = Random3D::RandInt();
					unsigned int max_possible = RAND_MAX;
					while(max_possible < total_coeffs)
					{
						max_possible <<= 1;
						choice <<= 1;
						choice |= Random3D::RandInt() % 2;
					}
				} while(choice >= total_coeffs);

				if(choice < NUM_INITIAL_VALUES)
					SparseNet::Randomize(initial_values[choice], scale);
				else
				{
					for(unsigned int i = 0; i < NUM_LAYERS; ++i)
					{
						if(choice >= layer_sizes[i])
							choice -= layer_sizes[i];
						else
						{
							unsigned int mod = nn[i]->max_input - nn[i]->min_input;
							SparseNet::Randomize(nn[i]->nodes[choice / mod][choice % mod + nn[i]->min_input], scale);
							break;
						}
					}
				}

				//int layer = Random3D::RandInt() % NUM_LAYERS;
				//nn[layer]->Randomize(1, scale);
			} while(Random3D::RandInt() % count != 0);
		}

		GABrain* CreateCrossover(const GABrain& b, unsigned int nextid)
		{
			GABrain* r = new GABrain(nextid, id, b.id);

			for(unsigned int i = 0; i < NUM_INITIAL_VALUES; ++i)
				r->initial_values[i] = max(-NN_COEFFS_RANGE, min(NN_COEFFS_RANGE, CrossoverCoeff(initial_values[i], b.initial_values[i])));

			for(unsigned int i = 0; i < NUM_LAYERS; ++i)
			{
				SparseNet* rk = new SparseNet(*nn[i]);
				r->nn[i] = rk;
				const SparseNet* ak = nn[i];
				const SparseNet* bk = b.nn[i];
				for(unsigned int j = 0; j < rk->nodes.size(); ++j)
					for(unsigned int k = 0; k < rk->nodes[j].size(); ++k)
						rk->nodes[j][k] = max(-NN_COEFFS_RANGE, min(NN_COEFFS_RANGE, CrossoverCoeff(ak->nodes[j][k], bk->nodes[j][k])));
			}

			return r;
		}

		static float CrossoverCoeff(float a, float b) { return a + (b - a) * Random3D::Rand(); }


		
		void Write(ostream& o)
		{
			for(unsigned int i = 0; i < NUM_INITIAL_VALUES; ++i)
				WriteSingle(initial_values[i], o);
			for(unsigned int i = 0; i < NUM_LAYERS; ++i)
				nn[i]->Write(o);
		}

		static unsigned int Read(istream& is, GABrain*& brain, unsigned int id)
		{
			brain = new GABrain(id, 0, 0);
			for(unsigned int i = 0; i < NUM_INITIAL_VALUES; ++i)
				brain->initial_values[i] = ReadSingle(is);
			for(unsigned int i = 0; i < NUM_LAYERS; ++i)
				if(int error = SparseNet::Read(is, brain->nn[i], i + 1 == NUM_LAYERS ? OUTPUT_LAYER_SIZE : MIDDLE_LAYER_SIZE, NUM_STRICT_INPUTS + MIDDLE_LAYER_SIZE * i))
				{
					Debug(((stringstream&)(stringstream() << "Error " << error << " occurred loading SparseNet from stream" << endl)).str());
					return error;
				}

			// enforce no direct input-to-output
			for(unsigned int i = 0; i < brain->nn[NUM_LAYERS - 1]->nodes.size(); ++i)
				for(unsigned int j = 0; j < NUM_STRICT_INPUTS; ++j)
					brain->nn[NUM_LAYERS - 1]->nodes[i][j] = 0.0f;
			brain->nn[NUM_LAYERS - 1]->min_input = NUM_STRICT_INPUTS;

			return 0;
		}
	};

	struct Subtest
	{
		unsigned int initial_frame;
		float initial_pitch;
		float initial_y;
		float yaw_move, pitch_move;
		float forward, sidestep;
		//Vec3 torso_vel;
		//Vec3 torso_rot;
		//Vec3 bone_rots[20];
		float gun_weight;
		int goal_delay;
		Vec3 goal_rvecs[19];
		Vec3 initial_bone_torque;
		int ibt_a, ibt_b;

		Subtest() : initial_frame(0), initial_pitch(0), initial_y(0), yaw_move(0), pitch_move(0), forward(0), sidestep(0), gun_weight(0), goal_delay(0), initial_bone_torque(), ibt_a(0), ibt_b(0)
		{
			//for(unsigned int i = 0; i < 20; ++i)
			//	bone_rots[i] = Vec3();
			for(unsigned int i = 0; i < 19; ++i)
				goal_rvecs[i] = Vec3();
		}
	};

	struct Experiment
	{
		mutex mutex;

		vector<GABrain*> candidates;
		unsigned int gen_done;
		unsigned int gen_size;

		list<GABrain*> elites;

		unsigned int batch;
		unsigned int next_id;

		string saved_debug_text;

		vector<Subtest> subtests;

		Experiment() : mutex(), candidates(), batch(0), next_id(1)
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
						candidates.push_back(loadme);
				}

				if(!candidates.empty())
				{
					Debug(((stringstream&)(stringstream() << "Successfully loaded " << candidates.size() << " brains from genepool" << endl)).str());

					// TODO: display stats?
				}

				file.close();
			}

			if(candidates.empty())
				MakeFirstGeneration();

			RecordGenSize();
			GenerateSubtestList();

			for(unsigned int i = 0; i < candidates.size(); ++i)
			{
				candidates[i]->fail_early = false;
				candidates[i]->tick_tot = 0;
				candidates[i]->trials_finished.clear();
				candidates[i]->trials_not_finished.clear();
				candidates[i]->trials_not_started.clear();
				for(unsigned int j = 0; j < NUM_TRIALS; ++j)
					candidates[i]->trials_not_started.push_back(j % NUM_SUBTESTS);
			}
		}

		~Experiment()
		{
#if PROFILE_ANY_CPHFT
			DebugCPHFTProfilingData();
#endif

			// delete all the brains
			for(unsigned int i = 0; i < candidates.size(); ++i)
				delete candidates[i];
		}

		void GenerateSubtestList()
		{
			subtests.clear();
			for(unsigned int i = 0; i < NUM_SUBTESTS; ++i)
			{
				Subtest s;
				s.initial_frame = 0;// i % 2;//walk_keyframes.size();
				//s.initial_pitch = Random3D::Rand(-1.0f, 1.0f) * (float(M_PI) * 0.45f);
				//s.initial_y     = 20.0f;//Random3D::RandInt() % 2 == 0 ? 0.0f : 20.0f;//Random3D::Rand( -0.015f, 0.0f );
				float mag;
				do
				{
					//s.yaw_move   = Random3D::Rand( -1.0f, 1.0f );
					//s.pitch_move = Random3D::Rand( -1.0f, 1.0f );
					//s.forward    = Random3D::Rand( -1.0f, 1.0f );
					//s.sidestep   = Random3D::Rand( -1.0f, 1.0f );
					//s.torso_vel  = Random3D::RandomNormalizedVector(Random3D::Rand(1.0f));
					//s.torso_rot  = Vec3(0, Random3D::Rand(-1.0f, 1.0f), 0);
					//s.gun_weight = Random3D::Rand(-1.0f, 1.0f);

					mag = s.yaw_move * s.yaw_move + s.pitch_move * s.pitch_move + s.forward * s.forward + s.sidestep * s.sidestep + /*s.torso_vel.ComputeMagnitudeSquared() + s.torso_rot.ComputeMagnitudeSquared() + */ s.gun_weight * s.gun_weight;// + s.joint_move_amt0 * s.joint_move_amt0 + s.joint_move_amt1 * s.joint_move_amt1;

					//for(unsigned int j = 0; j < 20 && mag <= 1.0f; ++j)
					//{
					//	float rmag = Random3D::Rand(0.5f);
					//	s.bone_rots[j] = Random3D::RandomNormalizedVector(rmag);
					//	mag += rmag * rmag;
					//}

					/*int idx = Random3D::RandInt() % 19;
					for(unsigned int j = 0; j < 19 && mag <= 1.0f; ++j)
					{
						if(j != idx)
							continue;
						float rmag = Random3D::Rand(1.0f);
						s.goal_rvecs[j] = Random3D::RandomNormalizedVector(rmag);
						mag += rmag * rmag * 0.25f;
					}*/

					//float ibt = Random3D::Rand(1.0f);
					//s.initial_bone_torque = Random3D::RandomNormalizedVector(ibt);
					//mag += ibt * ibt;
				} while(mag > 1.0f);

				s.goal_delay = Random3D::RandInt(MAX_TICK_AGE - 1);
				s.ibt_a = Random3D::RandInt() % 20;
				s.ibt_b = Random3D::RandInt() % 20;

				//s.desired_accel = Vec3(0, 0, i * 0.25f / float(NUM_SUBTESTS - 1));

				/*
				switch(i % 17)
				{
					//case 1: s.pitch_move = -1.0f; break;
					//case 2: s.pitch_move =  1.0f; break;
					//case 3: s.yaw_move   = -1.0f; break;
					//case 4: s.yaw_move   =  1.0f; break;

					case 1: s.torso_vel.x = -1.0f; break;
					case 2: s.torso_vel.x =  1.0f; break;
					case 3: s.torso_vel.y = -1.0f; break;
					case 4: s.torso_vel.y =  1.0f; break;
					case 5: s.torso_vel.z = -1.0f; break;
					case 6: s.torso_vel.z =  1.0f; break;
					case 7: s.pitch_move = -1.0f; break;
					case 8: s.pitch_move =  1.0f; break;
					case 9: s.yaw_move   = -1.0f; break;
					case 10: s.yaw_move  =  1.0f; break;

					case 11: s.torso_rot.y = -1.0f; break;
					case 12: s.torso_rot.y =  1.0f; break;

					case 13: s.pitch_move = -1.0f; s.gun_weight = 1.0f; break;
					case 14: s.pitch_move =  1.0f; s.gun_weight = 1.0f; break;
					case 15: s.yaw_move   = -1.0f; s.gun_weight = 1.0f; break;
					case 16: s.yaw_move   =  1.0f; s.gun_weight = 1.0f; break;
				}
				*/

				subtests.push_back(s);
			}
		}

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
						// TODO: print info about the brain
						Debug(ss.str());
					}
				}
			}
		}

		GABrain* GetBrain(Subtest& subtest, unsigned int& subtest_index)
		{
			bool verbose = false;
			if(candidates.empty())
			{
				if(verbose)
					Debug("candidate list is empty (1)\n");
				return NULL;
			}

			GABrain* brain = *candidates.rbegin();

			if(verbose)
				Debug(((stringstream&)(stringstream() << "brain " << brain->id << " trials not started = " << brain->trials_not_started.size() << "; fail early = " << brain->fail_early << endl)).str());

			while(brain->trials_not_started.empty() || brain->fail_early)
			{
				if(brain->fail_early)
				{
					for(unsigned int i = 0; i < brain->trials_not_started.size(); ++i)
						brain->trials_finished.push_back(brain->trials_not_started[i]);
					brain->trials_not_started.clear();
				}

				if(verbose)
					Debug(((stringstream&)(stringstream() << "popping brain " << brain->id << endl)).str());
				//swap(candidates[brain_index], candidates[candidates.size() - 1]);
				candidates.pop_back();
				if(candidates.empty())
				{
					if(verbose)
						Debug("candidate list is empty (2)\n");
					return NULL;
				}
				brain = *candidates.rbegin();
				if(verbose)
					Debug(((stringstream&)(stringstream() << "brain " << brain->id << " trials not started = " << brain->trials_not_started.size() << "; fail early = " << brain->fail_early << endl)).str());
			}

			if(brain->trials_not_finished.size() == 0)
			{
#if GENERATE_SUBTEST_LIST_EVERY == 3
				GenerateSubtestList();
#endif
				brain->subtests = subtests;
			}

			if(verbose)
				Debug(((stringstream&)(stringstream() << "returning brain " << brain->id << ", subtest " << NUM_TRIALS - brain->trials_not_started.size() << endl)).str());

			subtest_index = *brain->trials_not_started.rbegin();
			brain->trials_not_started.pop_back();
			brain->trials_not_finished.push_back(subtest_index);

			subtest = brain->subtests[subtest_index];
			return brain;
		}

		float GetFailEarlyThreshold()
		{
			unique_lock<std::mutex> lock(mutex);
			return elites.size() < NUM_ELITES ? -1.0f : (*elites.rbegin())->scores.ComputeTotal() * NUM_TRIALS;
		}

		void ExperimentDone(GABrain*& test, Subtest& subtest, unsigned int& subtest_index, const Scores& scores, string& debug_text, int numerator, int denominator, bool shouldFail)
		{
			unique_lock<std::mutex> lock(mutex);

			test->tick_tot += numerator;

			if(shouldFail)
				test->fail_early = true;

			if(test->trials_finished.size() == 0)
				test->scores = Scores();
			test->scores += scores;

			debug_text = ((stringstream&)(stringstream() << "batch " << batch << "; genome " << gen_done << " of " << gen_size << "; (id = " << test->id << " p = " << test->pa << ", " << test->pb << "); trial = " << test->trials_finished.size() << endl << saved_debug_text)).str();

			for(unsigned int i = test->trials_not_finished.size() - 1; i < test->trials_not_finished.size(); --i)	// relying on uint rollover
				if(test->trials_not_finished[i] == subtest_index)
				{
					swap(test->trials_not_finished[i], test->trials_not_finished[test->trials_not_finished.size() - 1]);
					test->trials_not_finished.pop_back();
				}
			test->trials_finished.push_back(subtest_index);
			
			float quasi_score = test->scores.total;// + test->scores.helper_force_penalty * (float(NUM_TRIALS) / trial - 1.0f);
			if(elites.size() >= NUM_ELITES && quasi_score >= (**elites.rbegin()).scores.total * NUM_TRIALS)
				test->fail_early = true;
			if(test->trials_finished.size() == NUM_TRIALS)
			{
				test->scores.ComputeTotal();
				test->scores /= float(test->trials_finished.size());

				// find out where to put this score in the elites array; the elites list size may temporarily exceed NUM_ELITES
				list<GABrain*>::iterator insert_where = elites.begin();
				while(insert_where != elites.end() && (test->fail_early || test->scores.total > (**insert_where).scores.total))
					++insert_where;
				elites.insert(insert_where, test);

				string failstring = ((stringstream&)(stringstream() << " fail (" << test->tick_tot << " / " << denominator * NUM_TRIALS << ")")).str();

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
								if(test->fail_early)
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
					//Debug(((stringstream&)(stringstream() << "deleting brain " << (*elites.rbegin())->id << endl)).str());
					delete *elites.rbegin();
					elites.pop_back();
				}

				// generation finished? save elites and build the next generation
				++gen_done;
				if(gen_done == gen_size)
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
			}

			test = GetBrain(subtest, subtest_index);
		}
		void MakeFirstGeneration()
		{
			static const bool random_from_middle = false;

			unsigned int first_gen_size = NUM_ELITES * NUM_ELITES;

			for(unsigned int i = 0; i < first_gen_size; ++i)
			{
				GABrain* b = new GABrain(next_id++, 0, 0);
				for(unsigned int j = 0; j < NUM_LAYERS; ++j)
				{
					b->nn[j] = new SparseNet(j + 1 == NUM_LAYERS ? OUTPUT_LAYER_SIZE : MIDDLE_LAYER_SIZE, NUM_STRICT_INPUTS + j * MIDDLE_LAYER_SIZE);
					if(j + 1 == NUM_LAYERS)
						b->nn[j]->min_input = NUM_STRICT_INPUTS;

					//for(unsigned int k = 0; k < b->nn[j]->nodes.size(); ++k)
					//	for(unsigned int m = b->nn[j]->min_input; m < b->nn[j]->max_input; ++m)
					//		b->nn[j]->nodes[k][m] = Random3D::Rand(-0.01f, 0.01f);

					if(j + 1 == NUM_LAYERS)
					{
						for(unsigned int k = 0; k < OUTPUT_LAYER_SIZE; ++k)
							b->nn[j]->nodes[b->nn[j]->nodes.size() - 1 - k][k] = INITIAL_NONZERO_DIAGONAL;
					}
					else if(j == 0)
					{
						//for(unsigned int k = 1; k <= 6; ++k)
							//b->nn[j]->nodes[k][k] = INITIAL_NONZERO_DIAGONAL;
					}
				}

				//for(unsigned int i = 0; i < NUM_INITIAL_VALUES; ++i)
				//	b->initial_values[i] = Random3D::Rand(-NN_COEFFS_RANGE, NN_COEFFS_RANGE);


				b->Randomize(MUTATION_COUNT, MUTATION_SCALE);

				if(i >= NUM_ELITES)
					for(int j = 0; j < 10; ++j)
						b->Randomize(MUTATION_COUNT, MUTATION_SCALE);	
					
				
				//b->Randomize(1000, 1.0f);

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
					mutant->Randomize(MUTATION_COUNT, MUTATION_SCALE);
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
							crossover->Randomize(MUTATION_COUNT, MUTATION_SCALE);
							candidates.push_back(crossover);
						}
					}
			}

			RecordGenSize();

#if GENERATE_SUBTEST_LIST_EVERY == 2
			GenerateSubtestList();
#endif

			elites.clear();

			for(unsigned int i = 0; i < candidates.size(); ++i)
			{
				candidates[i]->fail_early = false;
				candidates[i]->tick_tot = 0;
				candidates[i]->trials_finished.clear();
				candidates[i]->trials_not_finished.clear();
				candidates[i]->trials_not_started.clear();
				for(unsigned int j = 0; j < NUM_TRIALS; ++j)
					candidates[i]->trials_not_started.push_back(j % NUM_SUBTESTS);
			}
		}

		void RecordGenSize()
		{
			gen_size = candidates.size();
			gen_done = 0;
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

			Vec3 initial_pos;
			Quaternion initial_ori;

			Vec3 last_vel, last_rot;
			Vec3 net_impulse_linear, net_impulse_angular;

			CBone() { }
			CBone(const Soldier* dood, const string& name) : name(name), rb(dood->RigidBodyForNamedBone(name)), posey(dood->posey->skeleton->GetNamedBone(name)), local_com(rb->GetLocalCoM()), initial_pos(rb->GetPosition()), initial_ori(rb->GetOrientation()), last_vel(rb->GetLinearVelocity()), last_rot(rb->GetAngularVelocity()), net_impulse_linear(), net_impulse_angular() { }

			void Reset(float inv_timestep)
			{
				Vec3 vel = rb->GetLinearVelocity();
				Vec3 rot = rb->GetAngularVelocity();
				net_impulse_linear = (vel - last_vel) * (inv_timestep * rb->GetMass());
				net_impulse_angular = Mat3(rb->GetTransformedMassInfo().moi) * (rot - last_rot) * inv_timestep;
				last_vel = vel;
				last_rot = rot;
				desired_torque = applied_torque = desired_force = Vec3();
			}

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

			Mat3 last_desired;

			Vec3 r1, r2;

			Vec3 initial_rvec;
			Vec3 goal_rvec;
			Vec3 prev_goal;
			Vec3 integral;

			int mode;
			Vec3 pd_result;
			float chain_coeff;

			CJoint() : sjc(NULL), a(NULL), b(NULL), mode(0) { }
			CJoint(const Soldier* dood, CBone& bone_a, CBone& bone_b, float max_torque, int mode = 0) : mode(mode)
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

				//sjc->apply_torque = actual = Vec3();
				oriented_axes = sjc->axes * Quaternion::Reverse(sjc->obj_a->GetOrientation()).ToMat3();
			}

			Vec3 GetRVec() const
			{
				Quaternion a_to_b = sjc->obj_a->GetOrientation() * Quaternion::Reverse(sjc->obj_b->GetOrientation());
				return oriented_axes * a_to_b.ToRVec();
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

		Vec3 initial_com, desired_com;
		Vec3 desired_foot_vel[4];
		Vec3 desired_foot_rot[4];

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
					}

					float magsq = nu_force.ComputeMagnitudeSquared();
					if(magsq > max_forcesq)
						nu_force *= sqrtf(max_forcesq / magsq);

					nu_torque = force_to_torque * nu_force;
				}
			}

			void ApplySelectedForce(float timestep)
			{
				GetNudgeEffects(Vec3(), world_force, world_torque);

				//bone->rb->ApplyWorldForce(world_force, apply_pos);				// TODO: make this work?
				bone->rb->ApplyWorldImpulse(world_force * timestep, apply_pos);
			}
		};

		Vec3 initial_pos;

		vector<JetpackNozzle> jetpack_nozzles;
		bool jetpacking;
		Vec3 desired_jp_accel;

		Vec3 desired_aim;
		Vec3 prev_com_vel;

		bool failed_head, failed_foot;

		GABrain* brain;
		vector<float> simple_feedbacks[20];
		vector<float> node_comms[20];
		Subtest subtest;
		unsigned int subtest_index;

		unsigned int frame_index;
		float frame_time;

		//unsigned int lchoice, rchoice;

		struct MyContactPoint 
		{
			ContactPoint cp;
			Imp* imp;
			MyContactPoint(const ContactPoint& cp, Imp* imp) : cp(cp), imp(imp) { }
		};
		vector<MyContactPoint> old_contact_points;
		vector<MyContactPoint> new_contact_points;

		Imp() :
			init(false),
			experiment_done(false),
			timestep(0),
			inv_timestep(0),
			tick_age(0),
			max_tick_age(MAX_TICK_AGE),
			brain(NULL)
		{
			for(unsigned int i = 0; i < 20; ++i)
			{
				simple_feedbacks[i] = vector<float>();
				node_comms[i] = vector<float>();
			}
		}

		~Imp() 
		{
			for(unsigned int i = 0; i < 20; ++i)
			{
				simple_feedbacks[i].clear();
				node_comms[i].clear();
			}
		}

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
			float SP = 1200, N = 150, W = 200, E = 350, SB = 600, SA = 700, H = 1400, K = 1000, A = 600, HT = 600;

			RegisterJoint( spine1 = CJoint( dood, pelvis,    torso1,    SP,  0 ));
			RegisterJoint( spine2 = CJoint( dood, torso1,    torso2,    SP,  0 ));
			RegisterJoint( neck   = CJoint( dood, torso2,    head,      N,   0 ));
			RegisterJoint( lsja   = CJoint( dood, torso2,    lshoulder, SA,  0 ));
			RegisterJoint( lsjb   = CJoint( dood, lshoulder, luarm,     SB,  0 ));
			RegisterJoint( lelbow = CJoint( dood, luarm,     llarm,     E,   0 ));
			RegisterJoint( lwrist = CJoint( dood, llarm,     lhand,     W,   0 ));
			RegisterJoint( rsja   = CJoint( dood, torso2,    rshoulder, SA,  0 ));
			RegisterJoint( rsjb   = CJoint( dood, rshoulder, ruarm,     SB,  0 ));
			RegisterJoint( relbow = CJoint( dood, ruarm,     rlarm,     E,   0 ));
			RegisterJoint( rwrist = CJoint( dood, rlarm,     rhand,     W,   0 ));
			RegisterJoint( lhip   = CJoint( dood, pelvis,    luleg,     H,  -0 ));
			RegisterJoint( lknee  = CJoint( dood, luleg,     llleg,     K,  -0 ));
			RegisterJoint( lankle = CJoint( dood, llleg,     lheel,     A,  -0 ));
			RegisterJoint( lht    = CJoint( dood, lheel,     ltoe,      HT, -0 ));
			RegisterJoint( rhip   = CJoint( dood, pelvis,    ruleg,     H,  -0 ));
			RegisterJoint( rknee  = CJoint( dood, ruleg,     rlleg,     K,  -0 ));
			RegisterJoint( rankle = CJoint( dood, rlleg,     rheel,     A,  -0 ));
			RegisterJoint( rht    = CJoint( dood, rheel,     rtoe,      HT, -0 ));

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
			float jp_force = 200.0f;//150.0f;		// 98kg * 15m/s^2 accel / 10 nozzles ~= 150N per nozzle

			RegisterSymmetricJetpackNozzles( lshoulder, rshoulder, Vec3( 0.442619f, 1.576419f, -0.349652f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lshoulder, rshoulder, Vec3( 0.359399f, 1.523561f, -0.366495f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lshoulder, rshoulder, Vec3( 0.277547f, 1.480827f, -0.385142f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( ltoe,      rtoe,      Vec3( 0.237806f, 0.061778f,  0.038247f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lheel,     rheel,     Vec3( 0.238084f, 0.063522f, -0.06296f  ), upward, jp_angle, jp_force );
		}

		void Init(Soldier* dood)
		{
			initial_pos = dood->pos;
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

			foot_touchy.imp = this;
			foot_touchy.standing_callback = &dood->standing_callback;
			for(vector<FootState*>::iterator iter = dood->feet.begin(); iter != dood->feet.end(); ++iter)
				(*iter)->body->SetContactCallback(&foot_touchy);


			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
				gun_rb = gun->rigid_body;
			else
				gun_rb = NULL;

			SharedInit(dood);
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

				gun_rb->SetPosition(pos);
				gun_rb->SetOrientation(Quaternion::FromRotationMatrix(Mat3(a.x, b.x, c.x, a.y, b.y, c.y, a.z, b.z, c.z)));
				gun_rb->SetLinearVelocity (Vec3());
				gun_rb->SetAngularVelocity(Vec3());
			}

			for(unsigned int i = 0; i < all_joints.size(); ++i)
				all_joints[i]->last = Vec3();

			SharedInit(dood);
		}

		void SharedInit(Soldier* dood)
		{
			experiment_done = false;
			tick_age = 0;

			frame_index = 0;
			frame_time = 0.0f;

			cat_scores = Scores();

			failed_head = failed_foot = false;

			if(brain == NULL)
			{
				unique_lock<std::mutex> lock(experiment->mutex);
				brain = experiment->GetBrain(subtest, subtest_index);
			}

			if(brain != NULL)
			{
				for(unsigned int i = 0; i < 20; ++i)
				{
					simple_feedbacks[i].clear();
					simple_feedbacks[i].resize(NUM_SIMPLE_FEEDBACKS);
					node_comms[i].clear();
					node_comms[i].resize(NUM_NODE_COMMS);
				}
			}

			prev_com_vel = Vec3();

			for(unsigned int i = 0; i < 4; ++i)
				desired_foot_vel[i] = desired_foot_rot[i] = Vec3();

			//lchoice = rchoice = 0;

			//old_contact_points.clear();
			//new_contact_points.clear();
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

				//dood->PreparePAG(time, t2ori);

				//lhand    .ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				llarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				luarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				lshoulder.ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				//rhand    .ComputeDesiredTorqueWithPosey( hng_moi,     inv_timestep );
				rlarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				ruarm    .ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );
				rshoulder.ComputeDesiredTorqueWithDefaultMoIAndPosey( inv_timestep );

				set<RigidBody*> hng_set;
				hng_set.insert(lhand.rb);
				hng_set.insert(rhand.rb);
				hng_set.insert(gun->rigid_body);
				float hng_mass;
				Vec3 hng_com;
				Vec3 hng_vel;
				Vec3 hng_amom;
				ComputeMomentumStuff(hng_set, hng_mass, hng_com, hng_vel, hng_amom);

				Quaternion desired_gun_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);		// same as desired_head_ori
				Vec3 desired_gun_rot = (desired_gun_ori * Quaternion::Reverse(gun->rigid_body->GetOrientation())).ToRVec() * -inv_timestep;
				Vec3 desired_gun_aaccel = (desired_gun_rot - Mat3::Invert(hng_moi) * hng_amom) * inv_timestep;
				Vec3 gun_undo = hng_moi * -desired_gun_aaccel;

				// compute applied joint torques to achieve the per-bone applied torques we just came up with
				lwrist.SetWorldTorque(gun_undo * 0.25f);
				rwrist.SetWorldTorque(gun_undo - lwrist.actual);
				lwrist.SetWorldTorque(gun_undo - rwrist.actual);
				
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
			float aimfrac = sinf(min(1.0f, (float)tick_age / float(AIMMOVE_MAX_AGE)) * float(M_PI));
			float aimmove = aimfrac * aimfrac * timestep * 1.0f;

			dood->control_state->SetFloatControl("yaw",   dood->control_state->GetFloatControl("yaw"  ) + subtest.yaw_move   * aimmove);
			dood->control_state->SetFloatControl("pitch", dood->control_state->GetFloatControl("pitch") + subtest.pitch_move * aimmove);

			dood->control_state->SetFloatControl("forward",  subtest.forward);
			dood->control_state->SetFloatControl("sidestep", subtest.sidestep);
		}
		void DoSubtestInitialVelocity(Soldier* dood, const Subtest& subtest)
		{
			/*
			Vec3 vvec = subtest.torso_vel * 0.1f;
			head  .rb->SetLinearVelocity(vvec);
			torso2.rb->SetLinearVelocity(vvec);
			torso1.rb->SetLinearVelocity(vvec * 0.6f);
			pelvis.rb->SetLinearVelocity(vvec * 0.2f);

			static const float r = 0.00125f;
			Vec3 avvec = subtest.torso_rot * r;
			head  .rb->SetAngularVelocity(avvec);
			torso2.rb->SetAngularVelocity(avvec);
			torso1.rb->SetAngularVelocity(avvec * 0.6f);
			pelvis.rb->SetAngularVelocity(avvec * 0.2f);
			*/

			//for(unsigned int i = 0; i < 20; ++i)
			//	all_bones[i]->rb->SetAngularVelocity(subtest.bone_rots[i] * 0.1f);

			Vec3 ibt = subtest.initial_bone_torque * 0.2f;
			all_bones[subtest.ibt_a]->rb->ApplyAngularImpulse(ibt);
			all_bones[subtest.ibt_b]->rb->ApplyAngularImpulse(-ibt);
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

			if(brain == NULL)
			{
				experiment_done = true;
				return;
			}

			timestep     = time.elapsed;
			inv_timestep = 1.0f / timestep;

			if (Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				gun_rb = gun->rigid_body;
				gun_rb->ComputeInvMoI();		// force recomputation of stuffs

				gun_rb->ApplyCentralImpulse(Vec3(0, -subtest.gun_weight * 9.8f * timestep, 0));
			}
			else
				gun_rb = NULL;

			DoSubtestAimMove(dood, subtest);
			if(tick_age == 0)
				DoSubtestInitialVelocity(dood, subtest);

			//RigidBody& t2rb = *torso2.rb;
			//t2rb.SetLinearVelocity(t2rb.GetLinearVelocity() + subtest.torso_vel * (10.0f * timestep / t2rb.GetMass()));
			//t2rb.SetAngularVelocity(t2rb.GetAngularVelocity() + t2rb.GetInvMoI() * subtest.torso_rot * (1.0f * timestep));

#if PROFILE_CPHFT
			timer_init += timer.GetAndRestart();
#endif

			// reset all the joints and bones
			int ji = 0;
			for(unsigned int i = 0; i < all_joints.size(); ++i)
			{
				CJoint& joint = *all_joints[i];
				joint.Reset();
				if(tick_age == 0)
				{
					joint.sjc->apply_torque = Vec3();
					joint.actual = Vec3();
					joint.last = Vec3();
					joint.prev_goal = joint.goal_rvec = joint.initial_rvec = joint.GetRVec();
					joint.integral = Vec3();
				}

				if(tick_age == subtest.goal_delay)
					for(unsigned int j = 0; j < 3; ++j)
					{
						float minv = ((float*)&joint.sjc->min_extents)[j];
						float maxv = ((float*)&joint.sjc->max_extents)[j];

						float& goal = ((float*)&joint.goal_rvec)[j];
						float frac = ((float*)&subtest.goal_rvecs[i])[j];
						frac *= 0.125f;
						goal = frac * (maxv - minv);
						//goal = frac < 0 ? minv * -frac : maxv * frac;
						goal += ((float*)&joint.initial_rvec)[j];
						goal = max(minv, min(maxv, goal));


						/*
						if(minv != maxv)
						{
						if(ji == subtest.joint_move0 || ji == subtest.joint_move1)
						{
						float& vv = ((float*)&joint.goal_rvec)[i];
						vv += (ji == subtest.joint_move0 ? subtest.joint_move_amt0 : subtest.joint_move_amt1) * timestep * 20.0f;
						vv = max(minv, min(maxv, vv));
						}
						++ji;
						}
						*/
					}
			}
			for (vector<CBone*>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
			{
				(*iter)->Reset(inv_timestep);
				(*iter)->rb->ComputeInvMoI();		// force recomputation of stuffs
			}

#if PROFILE_CPHFT
			timer_reset += timer.GetAndRestart();
#endif

			float dood_mass;
			Vec3 dood_com, com_vel, angular_momentum;

			set<RigidBody*> included_rbs;
			//included_rbs.insert(dood->velocity_change_bodies.begin(), dood->velocity_change_bodies.end());
			//for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
			//	included_rbs.insert(lower_body_bones[i]->rb);
			included_rbs.insert(pelvis.rb);
			included_rbs.insert(torso1.rb);
			included_rbs.insert(torso2.rb);
			//included_rbs.insert(head.rb);
			//included_rbs.insert(lshoulder.rb);
			//included_rbs.insert(rshoulder.rb);

			//if(tick_age == 1)
			//{
			//	for(set<RigidBody*>::iterator iter = dood->velocity_change_bodies.begin(); iter != dood->velocity_change_bodies.end(); ++iter)
			//	{
			//		(*iter)->SetLinearVelocity(Vec3());
			//		(*iter)->SetAngularVelocity(Vec3());
			//	}
			//}

			ComputeMomentumStuff(included_rbs, dood_mass, dood_com, com_vel, angular_momentum);

			if(tick_age == 0)
				desired_com = initial_com = dood_com;

#if PROFILE_CPHFT
			timer_massinfo += timer.GetAndRestart();
#endif

			// upper body stuff; mostly working
			Quaternion p, t1, t2;
			Quaternion yaw_ori = Quaternion::FromAxisAngle(0, 1, 0, -dood->yaw);
			Mat3 yawmat = yaw_ori.ToMat3();
			Mat3 unyaw = yawmat.Transpose();

			/*{
				Vec3 desired_vel = (tick_age * timestep) * subtest.desired_accel;
				float dvmag = desired_vel.ComputeMagnitude();
				if(dvmag > 5.0f)
					desired_vel *= 5.0f / dvmag;

				desired_com += desired_vel;
			}*/

			//Vec3 momentum = com_vel * dood_mass;
			//Vec3 com_error = desired_com - dood_com;
			//Vec3 desired_momentum = com_error * (dood_mass * inv_timestep * 0.25f);
			//Vec3 desired_force = (desired_momentum - momentum) * (inv_timestep * 0.25f);

//			for(unsigned int i = 0; i < 4; ++i)
//				desired_force -= dood->feet[i]->net_force * use_params.ground_netf_subfrac[i];

			//Vec3 unyaw_df = unyaw * desired_force;
			//Vec3 unyaw_dc = unyaw * com_error * (dood_mass);

			vector<float> rb_kinetic(all_bones.size());
			for(unsigned int i = 0; i < all_bones.size(); ++i)
			{
				const RigidBody& rb = *all_bones[i]->rb;
				float ke = rb.GetMass() * rb.GetLinearVelocity().ComputeMagnitudeSquared();
				Vec3 av = rb.GetAngularVelocity();
				ke += Vec3::Dot(av, Mat3(rb.GetTransformedMassInfo().moi) * av);
				ke *= 0.5f;
				rb_kinetic[i] = ke;
			}
			

			//Vec3 torso2_fudge;
			//Vec3 pelvis_fudge;
			GetDesiredTorsoOris(dood, p, t1, t2);//, torso2_fudge, pelvis_fudge);

			DoHeadOri      ( dood, time     );
			DoArmsAimingGun( dood, time, t2 );

			Quaternion desired_head_ori = Quaternion::FromRVec(0, -dood->yaw, 0) * Quaternion::FromRVec(dood->pitch, 0, 0);

			for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
				iter->SolverInit(dood_com, 0.0f);

			//for(vector<CJoint*>::iterator iter = all_joints.begin(); iter != all_joints.end(); ++iter)
			//	(*iter)->SetOrientedTorque(Vec3());
			//DoScriptedMotorControl(dood);

			pelvis.posey->ori = p;
			torso1.posey->ori = t1 * Quaternion::Reverse(p);
			torso2.posey->ori = t2 * Quaternion::Reverse(t1);
			head  .posey->ori = desired_head_ori * Quaternion::Reverse(t2);

			torso2.ComputeDesiredTorqueWithDefaultMoIAndPosey(inv_timestep);
			torso1.ComputeDesiredTorqueWithDefaultMoIAndPosey(inv_timestep);
			pelvis.ComputeDesiredTorqueWithDefaultMoIAndPosey(inv_timestep);

			neck.SetTorqueToSatisfyB();
			spine2.SetTorqueToSatisfyB();
			spine1.SetTorqueToSatisfyB();

			for(unsigned int i = 0; i < all_joints.size(); ++i)
				all_joints[i]->SetWorldTorque(Vec3());

			Vec3 expected_vel = prev_com_vel;
			expected_vel -= dood->desired_vel_2d;
			expected_vel *= expf(-5.0f * timestep);	// ground_traction = 20.0f
			expected_vel += dood->desired_vel_2d;

			Vec3 vel_error_2d = expected_vel - com_vel;
			vel_error_2d.y = 0;
			Vec3 walk_error = vel_error_2d;

			prev_com_vel = com_vel;

			Vec3 com_error = walk_error;
			com_error.y = initial_com.y - dood_com.y;

			float any_cps[20];
			for(unsigned int i = 0; i < all_bones.size(); ++i)
			{
				RigidBody* rb = all_bones[i]->rb;
				any_cps[i] = 0.0f;
				for(unsigned int j = 0; j < new_contact_points.size(); ++j)
					if(new_contact_points[j].cp.obj_a == rb || new_contact_points[j].cp.obj_b == rb)
					{
						any_cps[i] = 1.0f;
						break;
					}
			}

			for(unsigned int i = 0; i < all_joints.size(); ++i)
				all_joints[i]->SetOrientedTorque(all_joints[i]->sjc->apply_torque);

			static const unsigned int batch_iterations = BATCH_ITERATIONS;
			for(unsigned int batch_iteration = 0; batch_iteration < BATCH_ITERATIONS; ++batch_iteration)
			{
				vector<float> nu_node_comms[20];
				for(unsigned int i = 0; i < 20; ++i)
					nu_node_comms[i].resize(NUM_NODE_COMMS);

				for(unsigned int controller_index = 0; controller_index < 20; ++controller_index)
				{
					// set inputs
					vector<float> inputs;

					const CBone* cbone = all_bones[controller_index];

					for(unsigned int i = 0; i < NUM_SIMPLE_FEEDBACKS; ++i)
						if(tick_age == 0)
							inputs.push_back(brain->initial_values[i] / NN_COEFFS_RANGE);
						else
							inputs.push_back(simple_feedbacks[controller_index][i]);

					inputs.push_back(tick_age == 0 ? 1.0f : 0.0f);

					inputs.push_back(cbone == &pelvis || cbone == &torso1 || cbone == &torso2 || cbone == &head ? 1.0f : 0.0f);
					inputs.push_back(cbone == &lshoulder || cbone == &luarm || cbone == &llarm || cbone == &lhand || cbone == &luleg || cbone == &llleg || cbone == &lheel || cbone == &ltoe ? 1.0f : 0.0f);
					inputs.push_back(cbone == &rshoulder || cbone == &ruarm || cbone == &rlarm || cbone == &rhand || cbone == &ruleg || cbone == &rlleg || cbone == &rheel || cbone == &rtoe ? 1.0f : 0.0f);
					inputs.push_back(cbone == &lshoulder || cbone == &luarm || cbone == &llarm || cbone == &lhand || cbone == &rshoulder || cbone == &ruarm || cbone == &rlarm || cbone == &rhand ? 1.0f : 0.0f);
					inputs.push_back(cbone == &luleg || cbone == &llleg || cbone == &lheel || cbone == &ltoe || cbone == &ruleg || cbone == &rlleg || cbone == &rheel || cbone == &rtoe ? 1.0f : 0.0f);
					inputs.push_back(cbone == &pelvis || cbone == &lshoulder || cbone == &rshoulder || cbone == &luleg || cbone == &ruleg ? 1.0f : 0.0f);
					inputs.push_back(cbone == &torso1 || cbone == &luarm || cbone == &ruarm || cbone == &llleg || cbone == &rlleg ? 1.0f : 0.0f);
					inputs.push_back(cbone == &torso2 || cbone == &llarm || cbone == &rlarm || cbone == &lheel || cbone == &rheel ? 1.0f : 0.0f);
					inputs.push_back(cbone == &head || cbone == &lhand || cbone == &rhand || cbone == &ltoe || cbone == &rtoe ? 1.0f : 0.0f);

					inputs.push_back(batch_iteration == 0 ? 1.0f : 0.0f);
					inputs.push_back(batch_iteration + 1 == BATCH_ITERATIONS ? 1.0f : 0.0f);
					inputs.push_back(float(batch_iteration) / float(BATCH_ITERATIONS));

					RigidBody* rb = cbone->rb;
					inputs.push_back(rb->GetCachedCoM().y);
					Mat3 invmat = Quaternion::Reverse(cbone->rb->GetOrientation()).ToMat3();
					PushVec3(inputs, invmat * Vec3(0, 1, 0));
					PushVec3(inputs, invmat * rb->GetLinearVelocity() * 0.1f);
					PushVec3(inputs, invmat * rb->GetAngularVelocity() * 0.1f);
					// TODO: net force?
					inputs.push_back(any_cps[controller_index]);
					// TODO: contact points net force?
					CJoint* etp = NULL;
					for(unsigned int i = 0; i < all_joints.size(); ++i)
						if(all_joints[i]->b == cbone)
						{
							etp = all_joints[i];
							break;
						}
					PushVec3(inputs, etp != NULL ? etp->GetRVec() : Vec3());
					PushVec3(inputs, etp != NULL ? etp->sjc->apply_torque / (etp->sjc->max_torque - etp->sjc->min_torque).ComputeMagnitude() : Vec3());
					PushVec3(inputs, cbone->applied_torque * 0.001f);
					PushVec3(inputs, cbone->applied_torque * 0.01f);
					inputs.push_back(tanhf(cbone->applied_torque.ComputeMagnitude() * 0.001f));
					inputs.push_back(tanhf(cbone->applied_torque.ComputeMagnitude() * 0.01f));

					for(unsigned int i = 0; i < NUM_NODE_COMMS; ++i)
						inputs.push_back(node_comms[controller_index][i]);

					int num_inputs = inputs.size();
					static bool printed_num_inputs = false;
					if(!printed_num_inputs)
					{
						printed_num_inputs = true;
						Debug(((stringstream&)(stringstream() << "num inputs = " << num_inputs << endl)).str());
						//for(unsigned int i = 0; i < inputs.size(); ++i)
							//Debug(((stringstream&)(stringstream() << "\tinputs[" << i << "] = " << inputs[i] << endl)).str());
					}

#if 0
					bool any = false;
					for(unsigned int i = 0; i < inputs.size(); ++i)
						if(abs(inputs[i]) > 0.98f)
						{
							Debug(((stringstream&)(stringstream() << "\tinputs[" << i << "] = " << inputs[i] << "; pre-tanh = " << atanhf(inputs[i]) << endl)).str());
							any = true;
						}
					if(any)
						Debug(((stringstream&)(stringstream() << "tick_age = " << tick_age << endl)).str());
#endif
			
					// evaluate
					vector<float> data = inputs;
					for(unsigned int i = 0; i < NUM_LAYERS; ++i)
						brain->nn[i]->Evaluate(data);

					//for(unsigned int i = num_inputs; i < data.size(); ++i)
					//	Debug(((stringstream&)(stringstream() << "data[" << i << "] = " << data[i] << endl)).str());	

					// apply outputs
					float* optr = data.data() + data.size() - OUTPUT_LAYER_SIZE;
					float* optr0 = optr;

					//Vec3 hip_avg = (pelvis.desired_torque - pelvis.applied_torque) * 0.5f;

					//Debug(((stringstream&)(stringstream() << "tick_age = " << tick_age << endl)).str());
					//Debug(((stringstream&)(stringstream() << "pos.y = " << pelvis.rb->GetPosition().y << "f;" << endl)).str());
					//Vec3 pori = pelvis.rb->GetOrientation().ToRVec();
					//Debug(((stringstream&)(stringstream() << "posey->skeleton->GetNamedBone( \"pelvis\" )->ori = Quaternion::FromRVec( " << pori.x << "f, " << pori.y << "f, " << pori.z << "f );" << endl)).str());
					//for(unsigned int i = 0; i < all_joints.size(); ++i)
					//{
					//	CJoint& j = *all_joints[i];
					//	//if(&j == &lhip || &j == &rhip || &j == &lknee || &j == &rknee || &j == &lankle || &j == &rankle || &j == &lht || &j == &rht)
					//	{
					//		const float* mins = (float*)&j.sjc->min_torque;
					//		const float* maxs = (float*)&j.sjc->max_torque;
					//		Vec3 use_torque;
					//		float* uts = (float*)&use_torque;
					//		for(unsigned int k = 0; k < 3; ++k)
					//		{
					//			if(mins[k] != maxs[k])
					//			{
					//				float ov = max(-1.0f, min(1.0f, 1.1f * *(optr++)));		// multiplication by 1.1 is to make it easier to hit the min/max
					//				uts[k] = ov >= 0 ? maxs[k] * ov : mins[k] * -ov;
					//			}
					//		}

					//		Vec3 r = (j.a->rb->GetOrientation() * Quaternion::Reverse(j.b->rb->GetOrientation())).ToRVec();
					//		Debug(((stringstream&)(stringstream() << "posey->skeleton->GetNamedBone( \"" << j.b->name << "\" )->ori = Quaternion::FromRVec( " << r.x << "f, " << r.y << "f, " << r.z << "f );" << endl)).str());

					//		//j.SetOrientedTorque(use_torque);

					//		//if(&j == &lhip || &j == &rhip)
					//		//	j.SetWorldTorque(j.actual + hip_avg);
					//	}
					//}

					//Debug(((stringstream&)(stringstream() << "optr was incremeneted this many times: " << (optr - optr0) << endl)).str());

					if(etp != NULL)// && batch_iteration + 1 == BATCH_ITERATIONS)
					{
						const float* mins = (float*)&etp->sjc->min_torque;
						const float* maxs = (float*)&etp->sjc->max_torque;
						Vec3 use_torque;
						float* uts = (float*)&use_torque;
						for(unsigned int k = 0; k < 3; ++k)
						{
							//float ov = max(-1.0f, min(1.0f, 1.1f * *(optr++)));		// multiplication by 1.1 is to make it easier to hit the min/max
							float ov = *(optr++);
							uts[k] = ov >= 0 ? maxs[k] * ov : mins[k] * -ov;
						}

						etp->SetOrientedTorque(etp->sjc->apply_torque + use_torque * 0.1f);

						//Vec3 r = (j.a->rb->GetOrientation() * Quaternion::Reverse(j.b->rb->GetOrientation())).ToRVec();
						//Debug(((stringstream&)(stringstream() << "posey->skeleton->GetNamedBone( \"" << j.b->name << "\" )->ori = Quaternion::FromRVec( " << r.x << "f, " << r.y << "f, " << r.z << "f );" << endl)).str());
					}

					for(unsigned int i = 0; i < all_joints.size(); ++i)
					{
						const CJoint& j = *all_joints[i];
						CBone* other = NULL;
						if(cbone == j.a)
							other = j.b;
						else if(cbone == j.b)
							other = j.a;
						if(other != NULL)
						{
							for(unsigned int j = 0; j < all_bones.size(); ++j)
								if(all_bones[j] == other)
								{
									float* optr2 = optr;
									for(unsigned int k = 0; k < NUM_NODE_COMMS; ++k, ++optr2)
										nu_node_comms[j][k] += *optr2;
									break;
								}
						}
					}
					optr += NUM_NODE_COMMS;

					// update simple feedbacks
					optr = data.data() + data.size() - OUTPUT_LAYER_SIZE + NUM_OUTPUTS;
			

			
					for(unsigned int i = 0; i < NUM_SIMPLE_FEEDBACKS; ++i)
						simple_feedbacks[controller_index][i] = *(optr++);
				}

				for(unsigned int i = 0; i < 20; ++i)
					node_comms[i] = nu_node_comms[i];
			}

			// orient pelvis; maintain pelvis' CoM
			//Vec3 pelvis_com = pelvis.rb->GetCenterOfMass();
			//pelvis.rb->SetOrientation(p);
			//pelvis.rb->SetPosition(pelvis.rb->GetPosition() + pelvis_com - pelvis.rb->GetCenterOfMass());


#if ENABLE_NEW_JETPACKING
			if(jetpacking)
			{
				for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
					iter->ApplySelectedForce(timestep);
			}
			else
			{
				for(vector<JetpackNozzle>::iterator iter = jetpack_nozzles.begin(); iter != jetpack_nozzles.end(); ++iter)
					iter->Reset();
			}

#endif

#if PROFILE_CPHFT
			timer_ub_stuff += timer.GetAndRestart();
#endif

			// scoring
			Scores cat_weights;
			cat_weights.lowness = 1.0f;
			//for(unsigned int i = 0; i < 19; ++i)
			//	cat_weights.ori_error[i] = 1.0f;

			cat_weights.energy_cost     = 0.00000001f;
			//cat_weights.kinetic_energy  = 0.1f;
			//cat_weights.feet_verror        = 1.0f;
			//cat_weights.feet_rerror        = 0.001f;
			//cat_weights.head_error      = 10.0f;
			//cat_weights.t2_error        = 10.0f;
			//cat_weights.t1_error        = 10.0f;
			//cat_weights.pelvis_error    = 100.0f;
			//cat_weights.gun_error       = min(1.0f, max(0.0f, float(tick_age) - 2) / 5.0f) * 10.0f;
			//cat_weights.gun_yerror      = min(1.0f, max(0.0f, float(tick_age) - 2) / 5.0f) * 2.0f;
			//cat_weights.com_errx        = 1.0f;
			//cat_weights.com_erry        = 1000.0f;
			//cat_weights.com_errz        = 1.0f;
			//cat_weights.early_fail_head = 1000.0f;
			//cat_weights.early_fail_feet = 1000.0f;

			float energy_cost = 0.0f;
			for(unsigned int i = 0; i < all_joints.size(); ++i)
				energy_cost += all_joints[i]->actual.ComputeMagnitudeSquared();

			//float kinetic_energy = 0.0f;
			//for(unsigned int i = 0; i < rb_kinetic.size(); ++i)
			//	if(all_bones[i] == &lheel || all_bones[i] == &rheel || all_bones[i] == &ltoe || all_bones[i] == &rtoe)
			//		kinetic_energy += rb_kinetic[i];	// quantity has already been squared

			/*if(head.rb->GetCenterOfMass().y < 1.7f)
				failed_head = true;*/


			//if(!failed_foot)
			//{
			//	float footfail_tot = 0.0f;
			//	footfail_tot += (ltoe.rb->GetPosition() - ltoe.initial_pos).ComputeMagnitudeSquared();
			//	footfail_tot += (lheel.rb->GetPosition() - lheel.initial_pos).ComputeMagnitudeSquared();
			//	footfail_tot += (rtoe.rb->GetPosition() - rtoe.initial_pos).ComputeMagnitudeSquared();
			//	footfail_tot += (rheel.rb->GetPosition() - rheel.initial_pos).ComputeMagnitudeSquared();
			//	if(footfail_tot > 1.0f)
			//		failed_foot = true;
			//}
			//if(dood->control_state->GetBoolControl("jump"))		// press spacebar to make candidates do badly (e.g. if stuck in a local minimum)
			//	failed_foot = true;

			/*float feet_verror = 0.0f;
			float feet_rerror = 0.0f;
			for(unsigned int i = 0; i < 4; ++i)
			{
				const RigidBody& rb = *dood->feet[i]->body;
				Vec3 vel = rb.GetLinearVelocity();
				Vec3 rot = rb.GetAngularVelocity();

				Vec3 verror = desired_foot_vel[i] - vel;
				verror.y = 0.0f;
				feet_verror += verror.ComputeMagnitudeSquared();

				feet_rerror += (desired_foot_rot[i] - rot).ComputeMagnitudeSquared();

				desired_foot_vel[i] = nu_dfootvel[i];
				desired_foot_rot[i] = nu_dfootrot[i];
			}*/

			Scores instant_scores;

			instant_scores.energy_cost = energy_cost;
			//instant_scores.kinetic_energy = kinetic_energy;
			//instant_scores.feet_verror = feet_verror;
			//instant_scores.feet_rerror = feet_rerror;
			////instant_scores.head_error = head_ori_error.ToRVec().ComputeMagnitudeSquared();
			////instant_scores.t2_error = t2_ori_error.ToRVec().ComputeMagnitudeSquared();
			////instant_scores.t1_error = t1_ori_error.ToRVec().ComputeMagnitudeSquared();
			//instant_scores.pelvis_error = pelvis_ori_error.ToRVec().ComputeMagnitudeSquared();
			////instant_scores.gun_error = gun_error.ComputeMagnitudeSquared();
			////instant_scores.gun_yerror = gun_yerror.ComputeMagnitudeSquared();
			//instant_scores.com_errx = com_error.x * com_error.x;
			//instant_scores.com_erry = com_error.y * com_error.y;
			//instant_scores.com_errz = com_error.z * com_error.z;

			//if(failed_head)
			//	instant_scores.early_fail_head = 1.0f;
			//if(failed_foot)
			//	instant_scores.early_fail_feet = 1.0f;

			//for(unsigned int i = 0; i < all_joints.size(); ++i)
			//	instant_scores.ori_error[i] = (all_joints[i]->goal_rvec - all_joints[i]->GetRVec()).ComputeMagnitudeSquared();
			instant_scores.lowness = tick_age > 30 ? max(0.0f, 1.5f - torso2.rb->GetCachedCoM().y) : 0.0f;

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
				if(brain->fail_early)
					early_fail = true;
				else
				{
					float fail_early_threshold = experiment->GetFailEarlyThreshold();
					if(fail_early_threshold >= 0.0f)
					{
						Scores quasi_scores;
						if(!brain->trials_finished.empty())
						{
							unique_lock<std::mutex>(experiment->mutex);
							quasi_scores = brain->scores;
						}

						cat_scores.ComputeTotal();
						quasi_scores.ComputeTotal();

						quasi_scores += cat_scores;
						if(quasi_scores.ComputeTotal() >= fail_early_threshold)
							early_fail = true;
					}
				}

				if(tick_age >= max_tick_age || early_fail)
				{
					// end-of-test stuff
					cat_scores.ComputeTotal();

					experiment->ExperimentDone(brain, subtest, subtest_index, cat_scores, ((TestGame*)dood->game_state)->debug_text, tick_age, max_tick_age, early_fail);
					experiment_done = true;
				}
			}


#if PROFILE_CPHFT
			timer_end_of_test += timer.GetAndRestart();
			timer_cphft_total += timer2.Stop();

			++counter_cphft;
#endif
		}

		static void PushVec3(vector<float>& inputs, const Vec3& v)
		{
			inputs.push_back(tanhf(v.x));
			inputs.push_back(tanhf(v.y));
			inputs.push_back(tanhf(v.z));
		}

		void DoScriptedMotorControl(Soldier* dood)
		{
			string filename = ((stringstream&)(stringstream() << "Files/Scripts/soldier_motor_control.lua")).str();

			ScriptingState script = ScriptSystem::GetGlobalState();
			lua_State* L = script.GetLuaState();

			// hook variables (hv)
			lua_newtable(L);

			// hv.bones
			lua_newtable(L);
			for(unsigned int i = 0; i < all_bones.size(); ++i)
				AddBoneToTable(all_bones[i], L);
			lua_setfield(L, -2, "bones");

			// hv.joints
			lua_newtable(L);
			for(unsigned int i = 0; i < all_joints.size(); ++i)
				AddJointToTable(all_joints[i], i + 1, L);
			lua_setfield(L, -2, "joints");


			// hv.nozzles
			lua_newtable(L);
			for(unsigned int i = 0; i < jetpack_nozzles.size(); ++i)
				AddNozzleToTable(i, L);
			lua_setfield(L, -2, "nozzles");

			// hv.controls
			lua_newtable(L);
			PushLuaMat3(L, Mat3::FromRVec(0, -dood->yaw, 0));
			lua_setfield(L, -2, "yaw_mat");
			lua_pushnumber(L, dood->pitch);
			lua_setfield(L, -2, "pitch");
			PushLuaVector(L, dood->desired_vel_2d);
			lua_setfield(L, -2, "walkvel");
			PushLuaVector(L, desired_jp_accel);
			lua_setfield(L, -2, "jpaccel");
			lua_pushboolean(L, dood->control_state->GetBoolControl("jump"));
			lua_setfield(L, -2, "jump");
			lua_setfield(L, -2, "controls");

			// hv.newcp
			lua_newtable(L);
			for(unsigned int i = 0; i < new_contact_points.size(); ++i)
				AddContactPointToTable(new_contact_points[i], i, L);
			lua_setfield(L, -2, "newcp");

			// hv.oldcp
			lua_newtable(L);
			for(unsigned int i = 0; i < old_contact_points.size(); ++i)
				AddContactPointToTable(old_contact_points[i], i, L);
			lua_setfield(L, -2, "oldcp");

			lua_pushnumber(L, tick_age);
			lua_setfield(L, -2, "age");
			
			lua_setglobal(L, "hv");

			script.DoFile(filename);

			lua_pushnil(L);
			lua_setglobal(L, "hv");
		}

		void AddBoneToTable(CBone* bone, lua_State* L)
		{
			CBone** bptr = (CBone**)lua_newuserdata(L, sizeof(CBone*));
			*bptr = bone;

			lua_getglobal(L, "CBoneMeta");
			if(lua_isnil(L, -1))
			{
				lua_pop(L, 1);
				// must create metatable for globals
				lua_newtable(L);									// push; top = 2

				lua_pushcclosure(L, cbone_index, 0);				// push; top = 3
				lua_setfield(L, -2, "__index");						// pop; top = 2

				lua_pushcclosure(L, cbone_newindex, 0);
				lua_setfield(L, -2, "__newindex");

				lua_setglobal(L, "CBoneMeta");
				lua_getglobal(L, "CBoneMeta");
			}
			lua_setmetatable(L, -2);								// set field of 1; pop; top = 1

			lua_setfield(L, -2, bone->name.c_str());
		}

		void AddJointToTable(CJoint* joint, int i, lua_State* L)
		{
			lua_pushinteger(L, i);			// the table (created below) will be assigned to index i of whatever was on the top of the stack (i.e. the joints list)

			CJoint** jptr = (CJoint**)lua_newuserdata(L, sizeof(CJoint*));
			*jptr = joint;

			lua_getglobal(L, "CJointMeta");
			if(lua_isnil(L, -1))
			{
				lua_pop(L, 1);
				// must create metatable for globals
				lua_newtable(L);									// push; top = 2

				lua_pushcclosure(L, cjoint_index, 0);				// push; top = 3
				lua_setfield(L, -2, "__index");						// pop; top = 2

				lua_setglobal(L, "CJointMeta");
				lua_getglobal(L, "CJointMeta");
			}
			lua_setmetatable(L, -2);								// set field of 1; pop; top = 1

			lua_settable(L, -3);
		}

		void AddNozzleToTable(int i, lua_State* L)
		{
			lua_pushinteger(L, i + 1);			// the table (created below) will be assigned to index i of whatever was on the top of the stack (i.e. the nozzles list)

			JetpackNozzle** jptr = (JetpackNozzle**)lua_newuserdata(L, sizeof(JetpackNozzle*));
			*jptr = jetpack_nozzles.data() + i;

			lua_getglobal(L, "JNozzleMeta");
			if(lua_isnil(L, -1))
			{
				lua_pop(L, 1);
				// must create metatable for globals
				lua_newtable(L);									// push; top = 2

				lua_pushcclosure(L, jnozzle_index, 0);
				lua_setfield(L, -2, "__index");

				lua_setglobal(L, "JNozzleMeta");
				lua_getglobal(L, "JNozzleMeta");
			}
			lua_setmetatable(L, -2);								// set field of 1; pop; top = 1

			lua_settable(L, -3);
		}

		

		void AddContactPointToTable(MyContactPoint& cp, int i, lua_State* L)
		{
			lua_pushinteger(L, i + 1);			// the table (created below) will be assigned to index i of whatever was on the top of the stack (i.e. the nozzles list)

			MyContactPoint** cpptr = (MyContactPoint**)lua_newuserdata(L, sizeof(MyContactPoint*));
			*cpptr = &cp;

			lua_getglobal(L, "ContactPointMeta");
			if(lua_isnil(L, -1))
			{
				lua_pop(L, 1);
				// must create metatable for globals
				lua_newtable(L);									// push; top = 2

				lua_pushcclosure(L, cp_index, 0);
				lua_setfield(L, -2, "__index");

				lua_setglobal(L, "ContactPointMeta");
				lua_getglobal(L, "ContactPointMeta");
			}
			lua_setmetatable(L, -2);								// set field of 1; pop; top = 1

			lua_settable(L, -3);
		}




		static int cbone_index(lua_State* L)
		{
			CBone* bone = *((CBone**)lua_touserdata(L, 1));

			if(lua_isstring(L, 2))
			{
				string key = lua_tostring(L, 2);

				lua_settop(L, 0);

				if		(key == "pos")				{ PushLuaVector(	L, bone->rb->GetCenterOfMass());									return 1; }
				else if	(key == "vel")				{ PushLuaVector(	L, bone->rb->GetLinearVelocity());									return 1; }
				else if	(key == "ori")				{ PushLuaMat3(		L, bone->rb->GetOrientation().ToMat3());							return 1; }
				else if	(key == "rot")				{ PushLuaVector(	L, bone->rb->GetAngularVelocity());									return 1; }
				else if	(key == "mass")				{ lua_pushnumber(	L, bone->rb->GetMass());											return 1; }
				else if (key == "moi")				{ PushLuaMat3(		L, Mat3(bone->rb->GetTransformedMassInfo().moi));					return 1; }
				else if (key == "impulse_linear")	{ PushLuaVector(	L, bone->net_impulse_linear);										return 1; }
				else if (key == "impulse_angular")	{ PushLuaVector(	L, bone->net_impulse_angular);										return 1; }
				else if (key == "desired_torque")	{ PushLuaVector(	L, bone->desired_torque);											return 1; }
				else if (key == "applied_torque")	{ PushLuaVector(	L, bone->applied_torque);											return 1; }
				else
					Debug("unrecognized key for CBone:index: " + key + "\n");
			}
			return 0;
		}

		static int cbone_newindex(lua_State* L)
		{
			CBone* bone = *((CBone**)lua_touserdata(L, 1));
			if(lua_isstring(L, 2))
			{
				string key = lua_tostring(L, 2);
				if(key == "desired_torque")
				{
					if(lua_isuserdata(L, 3))
					{
						Vec3* vec = (Vec3*)lua_touserdata(L, 3);
						bone->desired_torque = *vec;

						lua_settop(L, 0);
						return 0;
					}
				}
				else
					Debug("unrecognized key for CBone:newindex: " + key + "\n");
			}

			return 0;
		}



		static int cjoint_index(lua_State* L)
		{
			CJoint* joint = *((CJoint**)lua_touserdata(L, 1));

			if(lua_isstring(L, 2))
			{
				string key = lua_tostring(L, 2);

				lua_settop(L, 0);

				if		(key == "pos")				{ PushLuaVector(	L, joint->sjc->ComputeAveragePosition());							return 1; }
				else if	(key == "axes")				{ PushLuaMat3(		L, joint->oriented_axes);											return 1; }
				else if	(key == "min_extents")		{ PushLuaVector(	L, joint->sjc->min_extents);										return 1; }
				else if	(key == "max_extents")		{ PushLuaVector(	L, joint->sjc->max_extents);										return 1; }
				else if	(key == "min_torque")		{ PushLuaVector(	L, joint->sjc->min_torque);											return 1; }
				else if	(key == "max_torque")		{ PushLuaVector(	L, joint->sjc->max_torque);											return 1; }
				else if (key == "rvec")
				{
					PushLuaVector(L, joint->oriented_axes * -(joint->a->rb->GetOrientation() * Quaternion::Reverse(joint->b->rb->GetOrientation())).ToRVec());
					return 1;
				}
				else if (key == "local_torque")		{ PushLuaVector(	L, joint->sjc->apply_torque);										return 1; }
				else if (key == "world_torque")		{ PushLuaVector(	L, joint->actual);													return 1; }
				else if (key == "impulse_linear")	{ PushLuaVector(	L, joint->sjc->net_impulse_linear);									return 1; }
				else if (key == "impulse_angular")	{ PushLuaVector(	L, joint->sjc->net_impulse_angular);								return 1; }
				else if (key == "a")				{ lua_pushstring(	L, joint->a->name.c_str());											return 1; }
				else if (key == "b")				{ lua_pushstring(	L, joint->b->name.c_str());											return 1; }
				else if (key == "setWorldTorque")
				{
					lua_pushlightuserdata(L, joint);
					lua_pushcclosure(L, cjoint_setworld, 1);
					return 1;
				}
				else if (key == "setOrientedTorque")
				{
					lua_pushlightuserdata(L, joint);
					lua_pushcclosure(L, cjoint_setoriented, 1);
					return 1;
				}
				else if (key == "satisfyA")
				{
					lua_pushlightuserdata(L, joint);
					lua_pushcclosure(L, cjoint_satisfy_a, 1);
					return 1;
				}
				else if (key == "satisfyB")
				{
					lua_pushlightuserdata(L, joint);
					lua_pushcclosure(L, cjoint_satisfy_b, 1);
					return 1;
				}
				else
					Debug("unrecognized key for CJoint:index: " + key + "\n");
			}
			return 0;
		}

		static int cjoint_setworld(lua_State* L)
		{
			int n = lua_gettop(L);
			if(n == 1 && lua_isuserdata(L, 1))
			{
				void* ptr = lua_touserdata(L, 1);
				Vec3* vec = dynamic_cast<Vec3*>((Vec3*)ptr);

				lua_settop(L, 0);

				if(vec != NULL)
				{
					lua_pushvalue(L, lua_upvalueindex(1));
					CJoint* joint = (CJoint*)lua_touserdata(L, 1);
					lua_pop(L, 1);

					joint->SetWorldTorque(*vec);

					return 0;
				}
			}

			Debug("cjoint.setWorldTorque takes exactly 1 argument, a world-coords torque vector\n");
			return 0;
		}

		static int cjoint_satisfy_a(lua_State* L)
		{
			int n = lua_gettop(L);
			if(n == 0)
			{
				lua_pushvalue(L, lua_upvalueindex(1));
				CJoint* joint = (CJoint*)lua_touserdata(L, 1);
				lua_pop(L, 1);

				joint->SetTorqueToSatisfyA();

				return 0;
			}

			Debug("cjoint.satisfyA does not take any arguments\n");
			return 0;
		}

		static int cjoint_satisfy_b(lua_State* L)
		{
			int n = lua_gettop(L);
			if(n == 0)
			{
				lua_pushvalue(L, lua_upvalueindex(1));
				CJoint* joint = (CJoint*)lua_touserdata(L, 1);
				lua_pop(L, 1);

				joint->SetTorqueToSatisfyB();

				return 0;
			}

			Debug("cjoint.satisfyB does not take any arguments\n");
			return 0;
		}

		static int cjoint_setoriented(lua_State* L)
		{
			int n = lua_gettop(L);
			if(n == 1 && lua_isuserdata(L, 1))
			{
				void* ptr = lua_touserdata(L, 1);
				Vec3* vec = dynamic_cast<Vec3*>((Vec3*)ptr);

				lua_settop(L, 0);

				if(vec != NULL)
				{
					lua_pushvalue(L, lua_upvalueindex(1));
					CJoint* joint = (CJoint*)lua_touserdata(L, 1);
					lua_pop(L, 1);

					joint->SetOrientedTorque(*vec);

					return 0;
				}
			}

			Debug("cjoint.setOrientedTorque takes exactly 1 argument, a joint-coords torque vector\n");
			return 0;
		}



		static int jnozzle_index(lua_State* L)
		{
			JetpackNozzle** jptr = (JetpackNozzle**)lua_touserdata(L, 1);
			JetpackNozzle& nozzle = **jptr;

			if(lua_isstring(L, 2))
			{
				string key = lua_tostring(L, 2);

				lua_settop(L, 0);

				if		(key == "bone")			{ lua_pushstring	(L, nozzle.bone->name.c_str());					return 1; }
				else if (key == "pos")			{ PushLuaVector		(L, nozzle.apply_pos);							return 1; }
				else if (key == "center")		{ PushLuaVector		(L, nozzle.world_center);						return 1; }
				else if (key == "cone_cossq")	{ lua_pushnumber	(L, nozzle.cone_cossq);							return 1; }
				else if (key == "max_force")	{ lua_pushnumber	(L, nozzle.max_force);							return 1; }
				else if (key == "force")		{ PushLuaVector		(L, nozzle.world_force);						return 1; }
				else if (key == "torque")		{ PushLuaVector		(L, nozzle.world_torque);						return 1; }
				else if (key == "ftt")			{ PushLuaMat3		(L, nozzle.force_to_torque);					return 1; }
				else if (key == "setForce")
				{
					lua_pushlightuserdata(L, jptr);
					lua_pushcclosure(L, jnozzle_set_force, 1);
					return 1;
				}
				else
					Debug("unrecognized key for JetpackNozzle:index: " + key + "\n");
			}
			return 0;
		}

		static int jnozzle_set_force(lua_State* L)
		{
			lua_pushvalue(L, lua_upvalueindex(1));
			JetpackNozzle** jptr = (JetpackNozzle**)lua_touserdata(L, -1);
			lua_pop(L, 1);

			JetpackNozzle& nozzle = **jptr;

			int n = lua_gettop(L);
			if(n == 1 && lua_isuserdata(L, 1))
			{
				void* ptr = lua_touserdata(L, 1);
				Vec3* vec = dynamic_cast<Vec3*>((Vec3*)ptr);

				lua_settop(L, 0);

				if(vec != NULL)
				{
					nozzle.world_force = *vec;
					nozzle.GetNudgeEffects(Vec3(), nozzle.world_force, nozzle.world_torque);

					return 0;
				}
			}

			Debug("jnozzle.setForce takes exactly 1 argument, a world-coords forcej vector\n");
			return 0;
		}


		static int cp_index(lua_State* L)
		{
			MyContactPoint** cpptr = (MyContactPoint**)lua_touserdata(L, 1);
			MyContactPoint& mcp = **cpptr;
			ContactPoint& cp = mcp.cp;
			Imp* imp = mcp.imp;

			if(lua_isstring(L, 2))
			{
				string key = lua_tostring(L, 2);

				lua_settop(L, 0);

				if(key == "a" || key == "b")
				{
					string rbname = "<external>";

					RigidBody* ab = key == "a" ? cp.obj_a : cp.obj_b;
					if(ab == imp->gun_rb)
						rbname = "gun";
					else
					{
						for(unsigned int i = 0; i < imp->all_bones.size(); ++i)
							if(imp->all_bones[i]->rb == ab)
								rbname = imp->all_bones[i]->name;
					}
							
					lua_pushstring(L, rbname.c_str());
					return 1; 
				}
				else if (key == "pos")				{ PushLuaVector	(L, cp.pos);									return 1; }
				else if (key == "normal")			{ PushLuaVector	(L, cp.normal);									return 1; }
				else if (key == "restitution")		{ lua_pushnumber(L, cp.restitution_coeff);						return 1; }
				else if (key == "friction")			{ lua_pushnumber(L, cp.fric_coeff);								return 1; }
				else if (key == "bounce_threshold")	{ lua_pushnumber(L, cp.bounce_threshold);						return 1; }
				else if (key == "impulse_linear")	{ PushLuaVector	(L, cp.net_impulse_linear);						return 1; }
				else if (key == "impulse_angular")	{ PushLuaVector	(L, cp.net_impulse_angular);					return 1; }
				else
					Debug("unrecognized key for ContactPoint:index: " + key + "\n");
			}
			return 0;
		}



		struct NoTouchy : public ContactCallback
		{
			Imp* imp;
			void OnContact(const ContactPoint& contact) { imp->new_contact_points.push_back(MyContactPoint(contact, imp)); }
			void AfterResolution(const ContactPoint& cp) { imp->old_contact_points.push_back(MyContactPoint(cp, imp)); }
		} no_touchy;
		struct FootTouchy : public ContactCallback
		{
			Dood::StandingCallback* standing_callback;
			Imp* imp;
			void OnContact(const ContactPoint& contact) { standing_callback->OnContact(contact); imp->new_contact_points.push_back(MyContactPoint(contact, imp)); }
			void AfterResolution(const ContactPoint& cp) { standing_callback->AfterResolution(cp); imp->old_contact_points.push_back(MyContactPoint(cp, imp)); }
		} foot_touchy;
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
			if(standing_callback.IsStanding() && time.total > jump_start_timer && jump_to_fly_delay > 0)							// jump off the ground
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

	void Soldier::Vis(SceneRenderer* renderer)
	{
		Dood::Vis(renderer);

		BillboardMaterial* jetpack_trail = (BillboardMaterial*)((TestGame*)game_state)->mat_cache->Load("jetpack");

		for(unsigned int i = 0; i < imp->jetpack_nozzles.size(); ++i)
		{
			const Imp::JetpackNozzle& jn = imp->jetpack_nozzles[i];
			if(float magsq = jn.world_force.ComputeMagnitudeSquared())
			{
				static const float max_len = 0.8f;
				Vec3 lenvec = jn.world_force * (max_len / jn.max_force);
				Vec3 front = jn.apply_pos;// + lenvec / 8.0f;	// the "orb" at the front of the flame effect is centered at x=16 in the image, out of 128 total width
				Vec3 back = front - lenvec;

				BillboardMaterial::NodeData* nd = jetpack_trail->NewNodeData(front, back, 0.15f);
				renderer->objects.push_back(RenderNode(jetpack_trail, nd, Vec3::Dot(renderer->camera->GetForward(), front)));
			}
		}
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
		Subtest subtest = imp != NULL ? imp->subtest : Subtest();

		Quaternion yaw_ori = Quaternion::FromAxisAngle(0, 1, 0, -yaw);
		//Quaternion lb_oris[NUM_LOWER_BODY_BONES];
		//const WalkKeyframe& frame = walk_keyframes[subtest.initial_frame];
		//for(unsigned int i = 0; i < NUM_LOWER_BODY_BONES; ++i)
		//	lb_oris[i] = yaw_ori * Quaternion::FromRVec(frame.desired_oris[i]);

		//pos.y += subtest.initial_y - 0.01f;

		pitch += subtest.initial_pitch;

		pos.y = 0.548209f;

		Dood::DoInitialPose();
		
		posey->skeleton->GetNamedBone( "pelvis" )->ori = Quaternion::FromRVec( 0.737025f, -1.70684f, 1.94749f );
		posey->skeleton->GetNamedBone( "torso 1" )->ori = Quaternion::FromRVec( -0.0951396f, -0.00468156f, 0.719183f );
		posey->skeleton->GetNamedBone( "torso 2" )->ori = Quaternion::FromRVec( -0.305723f, -0.320675f, 0.740507f );
		posey->skeleton->GetNamedBone( "head" )->ori = Quaternion::FromRVec( -0.0283016f, 1.12843f, -0.362935f );
		posey->skeleton->GetNamedBone( "l shoulder" )->ori = Quaternion::FromRVec( -0.252179f, -0.111962f, -0.197518f );
		posey->skeleton->GetNamedBone( "l arm 1" )->ori = Quaternion::FromRVec( -1.3233f, 0.628318f, 0.587629f );
		posey->skeleton->GetNamedBone( "l arm 2" )->ori = Quaternion::FromRVec( -0.871514f, -0.569575f, 0.815938f );
		posey->skeleton->GetNamedBone( "l hand" )->ori = Quaternion::FromRVec( -1.76684f, -1.11402f, 0.13909f );
		posey->skeleton->GetNamedBone( "r shoulder" )->ori = Quaternion::FromRVec( -0.242087f, -0.0216953f, -0.191872f );
		posey->skeleton->GetNamedBone( "r arm 1" )->ori = Quaternion::FromRVec( 0.130442f, -0.196354f, 1.29011f );
		posey->skeleton->GetNamedBone( "r arm 2" )->ori = Quaternion::FromRVec( 0.247124f, -0.615489f, 0.621803f );
		posey->skeleton->GetNamedBone( "r hand" )->ori = Quaternion::FromRVec( -0.394564f, 1.23705f, 0.908416f );
		posey->skeleton->GetNamedBone( "l leg 1" )->ori = Quaternion::FromRVec( -0.244577f, 0.549211f, 0.205488f );
		posey->skeleton->GetNamedBone( "l leg 2" )->ori = Quaternion::FromRVec( 0.583199f, -0.0798103f, 0.00289411f );
		posey->skeleton->GetNamedBone( "l heel" )->ori = Quaternion::FromRVec( -0.130158f, -0.130852f, 0.390745f );
		posey->skeleton->GetNamedBone( "l toe" )->ori = Quaternion::FromRVec( 0.00874208f, -0.0782354f, 0.0494762f );
		posey->skeleton->GetNamedBone( "r leg 1" )->ori = Quaternion::FromRVec( -0.550917f, 0.103198f, 0.775355f );
		posey->skeleton->GetNamedBone( "r leg 2" )->ori = Quaternion::FromRVec( 0.728848f, -0.12879f, -0.798952f );
		posey->skeleton->GetNamedBone( "r heel" )->ori = Quaternion::FromRVec( 0.386118f, -0.00473954f, 0.0593444f );
		posey->skeleton->GetNamedBone( "r toe" )->ori = Quaternion::FromRVec( -0.24557f, 0.151427f, 0.0580851f );

		//Quaternion p, t1, t2;
		//imp->GetDesiredTorsoOris(this, p, t1, t2);

		//posey->skeleton->GetNamedBone( "pelvis"  )->ori = p;
		//posey->skeleton->GetNamedBone( "torso 1" )->ori = t1 * Quaternion::Reverse(p);
		//posey->skeleton->GetNamedBone( "torso 2" )->ori = t2 * Quaternion::Reverse(t1);

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
		//if(true)//subtest.initial_frame == 0)
		//{
		//	posey->skeleton->GetNamedBone( "l leg 1" )->ori = Quaternion::FromRVec( -0.05f, 0.1f,  0.05f );
		//	posey->skeleton->GetNamedBone( "l leg 2" )->ori = Quaternion::FromRVec(  0.1f,  0,     0     );
		//	posey->skeleton->GetNamedBone( "l heel"  )->ori = Quaternion::FromRVec( -0.05f, 0.1f, -0.05f );

		//	posey->skeleton->GetNamedBone( "r leg 1" )->ori = Quaternion::FromRVec(  0,    -0.1f,  0     );
		//	posey->skeleton->GetNamedBone( "r heel"  )->ori = Quaternion::FromRVec(  0,    -0.1f,  0     );
		//}
		//else
		//{
		//	posey->skeleton->GetNamedBone( "l leg 1" )->ori = Quaternion::FromRVec( -0.1f,  0.2f,  0     );
		//	posey->skeleton->GetNamedBone( "l leg 2" )->ori = Quaternion::FromRVec(  0.2f,  0,     0     );
		//	posey->skeleton->GetNamedBone( "l heel"  )->ori = Quaternion::FromRVec( -0.1f,  0.2f,  0     );

		//	posey->skeleton->GetNamedBone( "r leg 1" )->ori = Quaternion::FromRVec(  0,    -0.1f,  0     );
		//	posey->skeleton->GetNamedBone( "r heel"  )->ori = Quaternion::FromRVec(  0,     0,     0     );
		//}
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

		//PreparePAG(TimingInfo(), t2);

		//for(unsigned int i = 0; i < posey->skeleton->bones.size(); ++i)
		//	posey->skeleton->bones[i]->ori = Quaternion::Identity();
	}

	void Soldier::PreparePAG(const TimingInfo& time, const Quaternion& t2ori)
	{
		p_ag->yaw   = yaw;
		p_ag->pitch = pitch;
		p_ag->torso2_ori = t2ori;
		p_ag->UpdatePose(time);

		for(unordered_map<unsigned int, BoneInfluence>::iterator iter = p_ag->bones.begin(); iter != p_ag->bones.end(); ++iter)
			posey->skeleton->GetNamedBone(iter->first)->ori = Quaternion::FromRVec(iter->second.ori);

		posey->skeleton->InvalidateCachedBoneXforms();
	}

	void Soldier::PreCPHFT(float timestep) { Dood::PreCPHFT(timestep); }
	void Soldier::PostCPHFT(float timestep) { Dood::PostCPHFT(timestep); imp->old_contact_points.clear(); imp->new_contact_points.clear(); }



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
