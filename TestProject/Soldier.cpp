#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#include "BetterBrain.h"

#define DIE_AFTER_ONE_SECOND	0

#define ENABLE_NEW_JETPACKING	0

#define MAX_TICK_AGE			20

#define NUM_PARENTS				5
#define CHILDREN_PER_PAIR       4
#define MUTANTS_PER_SINGLE      6

#define NUM_TRIALS				100

#define NUM_LRINPUTS			85
#define NUM_CINPUTS				51
#define NUM_LROUTPUTS			63

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

	struct GABrain
	{
		BetterBrain* nn;
		vector<float> initial_mems;

		GABrain() : nn(NULL) { }

		~GABrain() { if(nn) { delete nn; nn = NULL; } }

		void InitBrain()
		{
			if(nn == NULL)
			{
				static const unsigned int num_lr_mems      = 20;
				static const unsigned int num_c_mems       = 20;

				static const unsigned int num_lr_hiddens_a = 20;
				static const unsigned int num_lr_hiddens_b = 20;
				static const unsigned int num_c_hiddens_a  = 40;
				static const unsigned int num_c_hiddens_b  = 40;		// needs to be large enough to hold instructions for both legs + updated c mems

				nn = new BetterBrain();

				unsigned int inputs_c   = nn->AddArray(NUM_CINPUTS);
				unsigned int inputs_l   = nn->AddArray(NUM_LRINPUTS);
				unsigned int inputs_r   = nn->AddArray(NUM_LRINPUTS);
				unsigned int outputs_l  = nn->AddArray(NUM_LROUTPUTS);
				unsigned int outputs_r  = nn->AddArray(NUM_LROUTPUTS);

				unsigned int mem_l      = nn->AddArray(num_lr_mems);
				unsigned int mem_r      = nn->AddArray(num_lr_mems);
				unsigned int mem_c      = nn->AddArray(num_c_mems);

				unsigned int hidden_l_a = nn->AddArray(num_lr_hiddens_a);
				unsigned int hidden_l_b = nn->AddArray(num_lr_hiddens_b);
				unsigned int hidden_r_a = nn->AddArray(num_lr_hiddens_a);
				unsigned int hidden_r_b = nn->AddArray(num_lr_hiddens_b);
				unsigned int hidden_c_a = nn->AddArray(num_c_hiddens_a);
				unsigned int hidden_c_b = nn->AddArray(num_c_hiddens_b);

				BetterBrain::Op lop_a;
				lop_a.from.push_back(inputs_l);
				lop_a.from.push_back(mem_l);
				lop_a.to.push_back(hidden_l_a);
				unsigned int lr_ha_mat = nn->AddOpCreateMatrix(lop_a);
				lop_a.from[0] = inputs_r;
				lop_a.from[1] = mem_r;
				lop_a.to[0]   = hidden_r_a;
				nn->AddOpExistingMatrix(lr_ha_mat, lop_a.from, lop_a.to);

				BetterBrain::Op lrc_a;
				lrc_a.from.push_back(hidden_l_a);
				lrc_a.from.push_back(hidden_r_a);
				lrc_a.from.push_back(inputs_c);
				lrc_a.from.push_back(mem_c);
				lrc_a.to.push_back(hidden_c_a);
				unsigned int cmat_a = nn->AddOpCreateMatrix(lrc_a);

				BetterBrain::Op lrc_b;
				lrc_b.from.push_back(hidden_c_a);
				lrc_b.to.push_back(hidden_c_b);
				unsigned int cmat_b = nn->AddOpCreateMatrix(lrc_b);

				BetterBrain::Op lrc_c;
				lrc_c.from.push_back(hidden_c_b);
				lrc_c.to.push_back(mem_c);
				unsigned int cmat_c = nn->AddOpCreateMatrix(lrc_c);

				BetterBrain::Op lop_b;
				lop_b.from.push_back(hidden_l_a);
				lop_b.from.push_back(hidden_c_b);
				lop_b.to.push_back(hidden_l_b);
				unsigned int lr_hb_mat = nn->AddOpCreateMatrix(lop_b);
				lop_b.from[0] = hidden_r_a;
				lop_b.from[1] = hidden_c_b;
				lop_b.to[0]   = hidden_r_b;
				nn->AddOpExistingMatrix(lr_hb_mat, lop_b.from, lop_b.to);

				BetterBrain::Op lop_c;
				lop_c.from.push_back(hidden_l_b);
				lop_c.to.push_back(outputs_l);
				unsigned int lr_o_mat = nn->AddOpCreateMatrix(lop_c);
				lop_c.from[0] = hidden_r_b;
				lop_c.to[0]   = outputs_r;
				nn->AddOpExistingMatrix(lr_o_mat, lop_c.from, lop_c.to);

				BetterBrain::Op lop_d;
				lop_d.from.push_back(hidden_l_b);
				lop_d.to.push_back(mem_l);
				unsigned int lr_m_mat = nn->AddOpCreateMatrix(lop_d);
				lop_d.from[0] = hidden_r_b;
				lop_d.to[0]   = mem_r;
				nn->AddOpExistingMatrix(lr_m_mat, lop_d.from, lop_d.to);

				bool def = true;
				nn->mats[lr_ha_mat]->mutate = def;
				nn->mats[cmat_a   ]->mutate = def;
				nn->mats[cmat_b   ]->mutate = def;
				nn->mats[cmat_c   ]->mutate = def;
				nn->mats[lr_hb_mat]->mutate = def;
				nn->mats[lr_o_mat ]->mutate = def;
				nn->mats[lr_m_mat ]->mutate = def;
			}

			initial_mems = vector<float>(14);
		}

		static void MakeChildCoeff(float& x, float a, float b, bool mutate)
		{
			if(a != 0.0f && b != 0.0f)
			{
				switch(Random3D::RandInt() % 3)
				{
					case 0: x = a; break;
					case 1: x = b; break;
					case 2: x = (a + b) * 0.5f; break;
				}
			}
			else if(a != 0.0f || b != 0.0f)
				x = Random3D::RandInt() % 2 == 0 ? a : b;
			else
				x = 0.0f;

			if(mutate)
			{
				static const unsigned int ztn_odds_against = 1000;		// zero to nonzero
				static const unsigned int ntz_odds_against = 900;		// nonzero to zero
				static const unsigned int mut_odds_against = 100;		// nonzero to some other nonzero

				static const float        ztn_rand_scale   = 0.05f;
				static const float        mut_rand_scale   = 0.01f;

				if(x == 0.0f)
				{
					if(Random3D::RandInt() % ztn_odds_against == 0)
						x = Random3D::Rand(-ztn_rand_scale, ztn_rand_scale);
				}
				else
				{
					if(Random3D::RandInt() % ntz_odds_against == 0)
						x = 0.0f;
					else if(Random3D::RandInt() % ztn_odds_against == 0)
						x = Random3D::Rand(-ztn_rand_scale, ztn_rand_scale);		// re-roll
					else if(Random3D::RandInt() % mut_odds_against == 0)
						x += Random3D::Rand(-mut_rand_scale, mut_rand_scale);
				}
			}
		}

		void InitCrossover(const GABrain& p1, const GABrain& p2)
		{
			InitBrain();

			for(unsigned int i = 0; i < nn->mats.size(); ++i)
			{
				BetterBrain::Matrix& cmat = *nn->mats[i];
				float *cptr = cmat.ptr, *cend = cptr + cmat.wh;
				for(const float *aptr = p1.nn->mats[i]->ptr, *bptr = p2.nn->mats[i]->ptr; cptr != cend; ++aptr, ++bptr, ++cptr)
					MakeChildCoeff(*cptr, *aptr, *bptr, cmat.mutate);
			}

			for(unsigned int i = 0; i < initial_mems.size(); ++i)
			{
				float *cptr = initial_mems.data(), *cend = cptr + initial_mems.size();
				for(const float *aptr = p1.initial_mems.data(), *bptr = p2.initial_mems.data(); cptr != cend; ++aptr, ++bptr, ++cptr)
					MakeChildCoeff(*cptr, *aptr, *bptr, true);
			}
		}
	};

	struct Experiment
	{
		struct Record
		{
			GABrain* brain;
			float score;

			unsigned int id;
			unsigned int p1, p2;

			Record(unsigned int id, unsigned int p1, unsigned int p2) : brain(new GABrain()), score(0), id(id), p1(p1), p2(p2) { }
			~Record() { if(brain) { delete brain; brain = NULL; } }

			bool operator <(const Record& r) { return score < r.score; }
		};

		unsigned int next_id;
		unsigned int batch, genome, trial;

		vector<Record*> genepool;

		Experiment()
		{
			next_id = 1;
			batch = genome = trial = 0;

			ifstream file("Files/brains", ios::in | ios::binary);
			if(!!file)
			{
				unsigned int num_genomes = ReadUInt32(file);

				for(unsigned int i = 0; i < num_genomes; ++i)
				{
					Record* r = new Record(next_id++, 0, 0);

					r->brain->nn = BetterBrain::Read(file);

					unsigned int num_mems = ReadUInt32(file);
					for(unsigned int j = 0; j < num_mems; ++j)
						r->brain->initial_mems.push_back(ReadSingle(file));

					genepool.push_back(r);
				}

				Debug(((stringstream&)(stringstream() << "Loaded " << num_genomes << " brains from file" << endl)).str());
			}
		}

		~Experiment()
		{
			//SaveGenepool("Files/brains");

			for(unsigned int i = 0; i < genepool.size(); ++i)
				delete genepool[i];
			genepool.clear();
		}

		void SaveGenepool(const string& filename)
		{
			ofstream file(filename, ios::out | ios::binary);
			if(!file)
				Debug("Failed to save brains!\n");
			else
			{
				WriteUInt32(genepool.size(), file);
				for(unsigned int i = 0; i < genepool.size(); ++i)
				{
					const GABrain& brain = *genepool[i]->brain;

					brain.nn->Write(file);

					WriteUInt32(brain.initial_mems.size(), file);
					for(unsigned int j = 0; j < brain.initial_mems.size(); ++j)
						WriteSingle(brain.initial_mems[j], file);
				}

				file.close();
			}
		}

		GABrain* NextBrain()
		{
			if(genepool.empty())
				FirstGeneration();
			else if(genome == genepool.size())
			{
				NextGeneration();

				genome = 0;
				++batch;
			}

			return genepool[genome]->brain;
		}

		void FirstGeneration()
		{
			static const unsigned int first_gen_size = NUM_PARENTS * ((NUM_PARENTS - 1) * CHILDREN_PER_PAIR / 2 + MUTANTS_PER_SINGLE + 1);
			for(unsigned int i = 0; i < first_gen_size; ++i)
			{
				Record* result = new Record(next_id++, 0, 0);

				result->brain = new GABrain();
				result->brain->InitBrain();
				result->brain->nn->MutateAll(50, 1.5f);
				
				for(unsigned int i = 0; i < result->brain->initial_mems.size(); ++i)
					result->brain->initial_mems[i] = Random3D::Rand(-0.5f, 0.5f);
				
				genepool.push_back(result);
			}
		}

		void NextGeneration()
		{
			ProfilingTimer timer;
			timer.Start();

			for(unsigned int i = 0; i < genepool.size(); ++i)
				for(unsigned int j = i + 1; j < genepool.size(); ++j)
					if(*genepool[j] < *genepool[i])
						swap(genepool[i], genepool[j]);

			while(genepool.size() > NUM_PARENTS)
			{
				Record* r = *genepool.rbegin();
				delete r;

				genepool.pop_back();
			}

			unsigned int actual_parents = genepool.size();

			time_t raw_time;
			time(&raw_time);
			tm now = *localtime(&raw_time);
			string filename = ((stringstream&)(stringstream() << "Files/brains-" << now.tm_year + 1900 << "-" << now.tm_mon + 1 << "-" << now.tm_mday << "-" << now.tm_hour << "-" << now.tm_min << "-" << now.tm_sec)).str();

			Debug(((stringstream&)(stringstream() << "Saving the selected parent brains to \"" << filename << "\"" << endl)).str());

			SaveGenepool(filename);
			SaveGenepool("Files/brains");

			for(unsigned int i = 0; i < actual_parents; ++i)
			{
				Record* p1 = genepool[i];
				for(unsigned int j = 0; j < MUTANTS_PER_SINGLE; ++j)
				{
					Record* c = new Record(next_id, p1->id, p1->id);
					c->brain->InitCrossover(*p1->brain, *p1->brain);
					genepool.push_back(c);
					++next_id;
				}

				for(unsigned int j = i + 1; j < actual_parents; ++j)
					for(unsigned int k = 0; k < CHILDREN_PER_PAIR; ++k)
					{
						Record* p2 = genepool[j];

						Record* c = new Record(next_id, p1->id, p2->id);
						c->brain->InitCrossover(*p1->brain, *p2->brain);
						genepool.push_back(c);
						++next_id;
					}
			}

			Debug(((stringstream&)(stringstream() << "generation " << batch << "; next gen will have " << genepool.size() << " genomes" << endl)).str());
			for(unsigned int i = 0; i < actual_parents; ++i)
			{
				Record* r = genepool[i];
				Debug(((stringstream&)(stringstream() << "\tparent " << i << ": score = " << genepool[i]->score << "; id = " << r->id << "; parent ids = (" << r->p1 << ", " << r->p2 << ")" << endl)).str());
			}

			for(unsigned int i = 0; i < genepool.size(); ++i)
			{
				swap(genepool[i], genepool[Random3D::RandInt(i, genepool.size() - 1)]);
				genepool[i]->score = 0;
			}

			float elapsed = timer.Stop();

			Debug(((stringstream&)(stringstream() << "building next gen took " << elapsed << " seconds" << endl)).str());
		}

		void GotScore(float score, string& ss_target)
		{
			Record* r = genepool[genome];
			GABrain* brain = r->brain;
			
			float& gscore = r->score;

			//score -= 5.4f;
			//score *= score;
			score /= NUM_TRIALS;

			gscore += score;

			{	// curly braces for scope
				stringstream ss;

				ss << "generation " << batch << ", genome " << genome << " of " << genepool.size() << ", trial " << trial << " of " << NUM_TRIALS << "; score so far = " << gscore << endl;
				for(unsigned int i = 0; i < genome && i < NUM_PARENTS; ++i)
				{
					const Record& p = *genepool[i];
					ss << endl << "top scorer # " << (i + 1) << ": score = " << p.score << "; id = " << p.id << "; parents = (" << p.p1 << ", " << p.p2 << ")";
				}
				
				ss_target = ss.str();
			}

			bool quick = genome >= NUM_PARENTS && gscore >= genepool[NUM_PARENTS - 1]->score;

			++trial;
			if(trial == NUM_TRIALS || quick)
			{
				stringstream ss;
				ss << "b " << batch << " g " << genome << ": score = " << gscore << "; id = " << r->id << "; parents = (" << r->p1 << ", " << r->p2 << ")";

				unsigned int i;
				for(i = genome; i != 0; --i)
				{
					if(genepool[i]->score < genepool[i - 1]->score)
						swap(genepool[i], genepool[i - 1]);
					else
						break;
				}

				if(i >= NUM_PARENTS)
					ss << "; fail (" << trial << " / " << NUM_TRIALS << "); proj = " << (gscore * NUM_TRIALS / trial) << endl;
				else
					ss << "; pass" << endl;
				Debug(ss.str());

				trial = 0;
				++genome;
			}
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

			Vec3 initial_pos;
			Quaternion initial_ori;

			CBone() { }
			CBone(const Soldier* dood, const string& name) : name(name), rb(dood->RigidBodyForNamedBone(name)), posey(dood->posey->skeleton->GetNamedBone(name)), local_com(rb->GetLocalCoM()), initial_pos(rb->GetPosition()), initial_ori(rb->GetOrientation()) { }

			void Reset(float inv_timestep) { desired_torque = applied_torque = Vec3(); }

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
		CBone luleg,     llleg,  lfoot;
		CBone ruleg,     rlleg,  rfoot;

		RigidBody* gun_rb;

		struct CJoint
		{
			SkeletalJointConstraint* sjc;
			CBone *a, *b;

			Vec3 actual;				// world-coords torque to be applied by this joint
			Vec3 last;

			Mat3 oriented_axes;			// gets recomputed every time Reset() is called

			Vec3 r1, r2;

			CJoint() { }
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
		CJoint lhip,   lknee,  lankle;
		CJoint rhip,   rknee,  rankle;

		vector<CBone*>  all_bones;
		vector<CJoint*> all_joints;

		GABrain* brain;
		vector<float> memories;
		vector<float> outputs;				// make this a member, for memory reuse purposes
		float worst, goal_error_cost;

		Vec3 desired_pelvis_pos;

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

		vector<JetpackNozzle> jetpack_nozzles;
		bool jetpacking;
		Vec3 desired_jp_accel;

		Vec3 desired_aim;

		Vec3 gun_initial_pos;
		Quaternion gun_initial_ori;

		Imp() :
			init(false),
			experiment_done(false),
			worst(0),
			goal_error_cost(0),
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

		void InitBrain(Soldier* dood)
		{
			brain = experiment->NextBrain();
			for(unsigned int i = 0; i < brain->nn->arrays.size(); ++i)
				brain->nn->arrays[i]->SetZero();

			vector<float> initial_mems = brain->initial_mems;
			float* mptr = initial_mems.data();

			InitialSetJointTorques(mptr, lankle);
			InitialSetJointTorques(mptr, rankle);
			InitialSetJointTorques(mptr, lknee, 1);
			InitialSetJointTorques(mptr, rknee, 1);
			InitialSetJointTorques(mptr, lhip);
			InitialSetJointTorques(mptr, rhip);
		}

		void Init(Soldier* dood)
		{
			//dood->collision_group->SetInternalCollisionsEnabled(true);		// TODO: resolve problems arising from torso2-arm1 collisions

			all_bones.clear();
			all_joints.clear();
			jetpack_nozzles.clear();

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
			RegisterBone( lfoot     = CBone( dood, "l foot"     ));
			RegisterBone( ruleg     = CBone( dood, "r leg 1"    ));
			RegisterBone( rlleg     = CBone( dood, "r leg 2"    ));
			RegisterBone( rfoot     = CBone( dood, "r foot"     ));

			float SP = 1500, N = 150, W = 200, E = 350, SB = 600, SA = 700, H = 1400, K = 800, A = 500;
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
			RegisterJoint( lankle = CJoint( dood, llleg,     lfoot,     A  ));
			RegisterJoint( rhip   = CJoint( dood, pelvis,    ruleg,     H  ));
			RegisterJoint( rknee  = CJoint( dood, ruleg,     rlleg,     K  ));
			RegisterJoint( rankle = CJoint( dood, rlleg,     rfoot,     A  ));

			lknee.sjc->min_torque.y = lknee.sjc->min_torque.z = lknee.sjc->max_torque.y = lknee.sjc->max_torque.z = 0.0f;
			rknee.sjc->min_torque.y = rknee.sjc->min_torque.z = rknee.sjc->max_torque.y = rknee.sjc->max_torque.z = 0.0f;

			Vec3 upward(0, 1, 0);
			float jp_angle = 1.0f;
			float jp_force = 150.0f;		// 98kg * 15m/s^2 accel / 10 nozzles ~= 150N per nozzle

			RegisterSymmetricJetpackNozzles( lshoulder, rshoulder, Vec3( 0.442619f, 1.576419f, -0.349652f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lshoulder, rshoulder, Vec3( 0.359399f, 1.523561f, -0.366495f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lshoulder, rshoulder, Vec3( 0.277547f, 1.480827f, -0.385142f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lfoot,     rfoot,     Vec3( 0.237806f, 0.061778f,  0.038247f ), upward, jp_angle, jp_force );
			RegisterSymmetricJetpackNozzles( lfoot,     rfoot,     Vec3( 0.238084f, 0.063522f, -0.06296f  ), upward, jp_angle, jp_force );

			InitBrain(dood);

			no_touchy.imp = this;
			for(set<RigidBody*>::iterator iter = dood->velocity_change_bodies.begin(); iter != dood->velocity_change_bodies.end(); ++iter)
				if(*iter != lfoot.rb && *iter != rfoot.rb)
					(*iter)->SetContactCallback(&no_touchy);

			desired_pelvis_pos = pelvis.rb->GetCenterOfMass();

			if(Gun* gun = dynamic_cast<Gun*>(dood->equipped_weapon))
			{
				gun_rb = gun->rigid_body;
				gun_initial_pos = gun_rb->GetPosition();
				gun_initial_ori = gun_rb->GetOrientation();
			}
		}

		void ReInit(Soldier* dood)
		{
			dood->yaw = dood->pitch = 0.0f;

			for(unsigned int i = 0; i < all_bones.size(); ++i)
			{
				CBone& cb = *all_bones[i];
				RigidBody& rb = *cb.rb;
				rb.SetPosition(cb.initial_pos);
				rb.SetOrientation(cb.initial_ori);
				rb.SetLinearVelocity(Vec3());
				rb.SetAngularVelocity(Vec3());
			}

			if(gun_rb != NULL)
			{
				gun_rb->SetPosition(gun_initial_pos);
				gun_rb->SetOrientation(gun_initial_ori);
				gun_rb->SetLinearVelocity(Vec3());
				gun_rb->SetAngularVelocity(Vec3());
			}

			for(unsigned int i = 0; i < all_joints.size(); ++i)
				all_joints[i]->last = Vec3();

			InitBrain(dood);

			experiment_done = false;
			worst = goal_error_cost = 0.0f;
			tick_age = 0;
		}

		void GetDesiredTorsoOris(Soldier* dood, Quaternion& p, Quaternion& t1, Quaternion& t2)
		{
			float pfrac = dood->pitch * (2.0f / float(M_PI)), pfsq = pfrac * pfrac;

			float t1_yaw   = dood->yaw + torso2_yaw_offset;
			float t1_pitch = dood->pitch * 0.4f + pfsq * pfrac * 0.95f;
			float t1_yaw2  = pfsq * 0.7f;

			t2 = Quaternion::FromRVec(0, -t1_yaw, 0) * Quaternion::FromRVec(t1_pitch, 0, 0) * Quaternion::FromRVec(0, -t1_yaw2, 0);

			float t2_yaw   = dood->yaw + torso2_yaw_offset * 0.5f;
			float t2_pitch = pfrac * 0.05f + pfrac * pfsq * 0.3f;
			float t2_yaw2  = pfsq * 0.15f;

			p = Quaternion::FromRVec(0, -t2_yaw, 0) * Quaternion::FromRVec(t2_pitch, 0, 0) * Quaternion::FromRVec(0, -t2_yaw2, 0);

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

		void ComputeMomentumStuff(Soldier* dood, float& dood_mass, Vec3& dood_com, Vec3& com_vel, Vec3& angular_momentum)
		{
			com_vel = Vec3();
			dood_mass = 0.0f;
			for(set<RigidBody*>::iterator iter = dood->velocity_change_bodies.begin(); iter != dood->velocity_change_bodies.end(); ++iter)
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

			for(set<RigidBody*>::iterator iter = dood->velocity_change_bodies.begin(); iter != dood->velocity_change_bodies.end(); ++iter)
			{
				RigidBody* body = *iter;
				mass_info = body->GetTransformedMassInfo();

				// linear component
				float mass = mass_info.mass;
				Vec3 vel = body->GetLinearVelocity() - com_vel;
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


		// some utility functions used by the Update to set up the inputs array for the GABrain, and make use of the outputs
		static void PushVec3(vector<float>& inputs, const Vec3& vec)
		{
			inputs.push_back(vec.x);
			inputs.push_back(vec.y);
			inputs.push_back(vec.z);
		}

		static void GetBoneDesiredLAImpulse(const CBone& bone, float inv_timestep, Vec3& force, Vec3& torque)
		{
			force = ((bone.local_com - bone.rb->GetCenterOfMass()) * inv_timestep - bone.rb->GetLinearVelocity()) * (inv_timestep * bone.rb->GetMass());

			Vec3 desired_rot    = (bone.initial_ori * Quaternion::Reverse(bone.rb->GetOrientation())).ToRVec() * -inv_timestep;
			Vec3 desired_aaccel = (desired_rot - bone.rb->GetAngularVelocity()) * inv_timestep;
			torque = Mat3(bone.rb->GetTransformedMassInfo().moi) * desired_aaccel;
		}

		static void PushBoneRelative(vector<float>& inputs, const CBone& pushme, const CBone& rel, const CJoint& joint, unsigned int n = 3)
		{
			Mat3 unrotate = rel.rb->GetOrientation().ToMat3().Transpose();
			PushVec3(inputs, unrotate * (joint.sjc->obj_b->LocalPosToWorld(joint.sjc->pos) - joint.sjc->obj_a->LocalPosToWorld(joint.sjc->pos) * 10.0f));
			//PushVec3(inputs, unrotate * pushme.rb->GetLinearVelocity()  * 0.02f);
			//PushVec3(inputs, unrotate * pushme.rb->GetAngularVelocity() * 0.02f);

			//PushVec3(inputs, (pushme.rb->GetOrientation() * Quaternion::Reverse(pushme.initial_ori)).ToRVec());

			const SkeletalJointConstraint* sjc = joint.sjc;

			Mat3 rvecmat = sjc->axes * Quaternion::Reverse(joint.a->rb->GetOrientation()).ToMat3() * joint.b->rb->GetOrientation().ToMat3() * joint.sjc->axes.Transpose();
			Vec3 rvec = Quaternion::FromRotationMatrix(rvecmat).ToRVec();

			PushVec3(inputs, rvec);

			for(const float *optr = (float*)&joint.last, *oend = optr + n, *minp = (float*)&sjc->min_torque, *maxp = (float*)&sjc->max_torque; optr != oend; ++optr, ++minp, ++maxp)
				inputs.push_back(*optr * 2.0f / max(*maxp, -*minp));

			PushVec3(inputs, sjc->net_impulse_linear);
			PushVec3(inputs, sjc->net_impulse_angular);

			Vec3 force, torque;
			GetBoneDesiredLAImpulse(pushme, 60.0f, force, torque);
			
			PushVec3(inputs, unrotate * force  * 0.02f);
			PushVec3(inputs, unrotate * torque * 0.02f);

			for(const float *rptr = (float*)&rvec, *rend = rptr + 3, *minp = (float*)&sjc->min_extents, *maxp = (float*)&sjc->max_extents; rptr != rend; ++rptr, ++minp, ++maxp)
			{
				inputs.push_back(*rptr - *maxp);
				inputs.push_back(*minp - *rptr);
			}
		}

		static void PushFootStuff(vector<float>& inputs, Dood::FootState* foot, const Vec3& pos, const Vec3& normal, float eta)
		{
			SoldierFoot* sf = (SoldierFoot*)foot;
			const RigidBody* rb = sf->body;
			Mat3 unrotate    = rb->GetOrientation().ToMat3().Transpose();
			Vec3 untranslate = rb->GetCenterOfMass();

			/*
			// contact points
			for(unsigned int j = 0; j < 3; ++j)
			{
				if(j < sf->contact_points.size())
				{
					const ContactPoint& cp = sf->contact_points[j];
					PushVec3(inputs, unrotate * (cp.pos - untranslate));
					PushVec3(inputs, unrotate * cp.normal);
				}
				else
				{
					PushVec3(inputs, Vec3());
					PushVec3(inputs, Vec3());
				}
			}
			*/

			// net force and torque
			PushVec3(inputs, unrotate * foot->net_force  * 0.2f);
			PushVec3(inputs, unrotate * foot->net_torque * 0.2f);

			// goal info
			//PushVec3(inputs, unrotate * (pos - untranslate));
			//PushVec3(inputs, unrotate * normal);
			//inputs.push_back(eta);
		}

		static void InitialSetJointTorques(float*& vptr, CJoint& joint, unsigned int n = 3)
		{
			const float* minptr = (float*)&joint.sjc->min_torque;
			const float* maxptr = (float*)&joint.sjc->max_torque;

			for(float *optr = (float*)&joint.sjc->apply_torque, *oend = optr + n; optr != oend; ++optr, ++vptr, ++minptr, ++maxptr)
				*optr = *vptr >= 0.0f ? *vptr * *maxptr : -*vptr * *minptr;

			joint.last = joint.sjc->apply_torque;
		}

		static void SetJointTorques(const float*& optr, CJoint& joint, unsigned int n = 3)
		{
			const float* minptr = (float*)&joint.sjc->min_torque;
			const float* maxptr = (float*)&joint.sjc->max_torque;

			Vec3 v = joint.sjc->apply_torque;
			float* vptr = (float*)&v;
			float* lptr = (float*)&joint.last;

			for(const float* oend = optr + n * 3 * 3; optr != oend; ++vptr, ++lptr, ++minptr, ++maxptr)
			{
				float zero = 0.0f, incr = 0.0f, decr = 0.0f;
				for(unsigned int i = 0; i < 3; ++i)
				{
					zero += max(0.0f, *(optr++));
					incr += max(0.0f, *(optr++));
					decr += max(0.0f, *(optr++));
				}
				zero /= 3.0f;
				incr /= 3.0f;
				decr /= 3.0f;

				float def  = max(0.0f, 1.0f - zero - incr - decr);

				if(zero != 0.0f || incr != 0.0f || decr != 0.0f)
				{
					float& x = *vptr;
					x  = *lptr   * def;
					x += *maxptr * incr;
					x += *minptr * decr;

					x /= zero + incr + decr;
				}
				else
					*vptr = *lptr;
			}

			joint.SetOrientedTorque(v);
		}



		void Update(Soldier* dood, const TimingInfo& time)
		{
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

			// reset all the joints and bones
			for(vector<CJoint*>::iterator iter = all_joints.begin(); iter != all_joints.end(); ++iter)
				(*iter)->Reset();

			for(vector<CBone*>::iterator iter = all_bones.begin(); iter != all_bones.end(); ++iter)
				(*iter)->Reset(inv_timestep);

			float dood_mass;
			Vec3 dood_com, com_vel, angular_momentum;
			ComputeMomentumStuff(dood, dood_mass, dood_com, com_vel, angular_momentum);

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

			// upper body stuff; mostly working
			Quaternion p, t1, t2;
			GetDesiredTorsoOris(dood, p, t1, t2);
			Quaternion yaw_ori = Quaternion::FromAxisAngle(0, 1, 0, -dood->yaw);

			DoHeadOri      ( dood, time     );
			DoArmsAimingGun( dood, time, t2 );

			pelvis.ComputeDesiredTorqueWithDefaultMoI(p,  inv_timestep);
			torso1.ComputeDesiredTorqueWithDefaultMoI(t1, inv_timestep);
			torso2.ComputeDesiredTorqueWithDefaultMoI(t2, inv_timestep);

			spine1.SetTorqueToSatisfyB();
			spine2.SetTorqueToSatisfyB();

			// per-sub-experiment goals changes
#if 0
			switch(experiment->batch % 5)
			{
				case 1: dood->yaw            += 0.1f  * timestep; break;
				case 2: dood->yaw            -= 0.1f  * timestep; break;
				case 3: dood->pitch          += 0.1f  * timestep; break;
				case 4: dood->pitch          -= 0.1f  * timestep; break;
				default: break;
			}
#endif


			// stuff info about the dood into an array, which will then be crammed through a brain... maybe something cool will happen
			vector<float> linputs, rinputs, cinputs;

			// root bone info
			Mat3 reverse_pelvis = Quaternion::Reverse(pelvis.rb->GetOrientation()).ToMat3();
			PushVec3(cinputs, reverse_pelvis * Vec3(0, 1, 0));
			//PushVec3(cinputs, reverse_pelvis * pelvis.rb->GetLinearVelocity()  * 0.02f);
			//PushVec3(cinputs, reverse_pelvis * pelvis.rb->GetAngularVelocity() * 0.02f);
			PushVec3(cinputs, reverse_pelvis * (dood_com - pelvis.rb->GetCenterOfMass()));
			PushVec3(cinputs, reverse_pelvis * com_vel * 0.02f);
			PushVec3(cinputs, reverse_pelvis * angular_momentum * 0.02f);

			Vec3 pforce, ptorque;
			GetBoneDesiredLAImpulse(pelvis, inv_timestep, pforce, ptorque);
			PushVec3(cinputs, reverse_pelvis * pforce  * 0.02f);
			PushVec3(cinputs, reverse_pelvis * ptorque * 0.02f);

			// specifying other bones relative to that
			PushBoneRelative( cinputs, torso1, pelvis, spine1    );
			//PushBoneRelative( cinputs, torso2, torso1, spine2    );
			PushBoneRelative( linputs, luleg,  pelvis, lhip      );
			PushBoneRelative( linputs, llleg,  luleg,  lknee,  1 );
			PushBoneRelative( linputs, lfoot,  llleg,  lankle    );
			PushBoneRelative( rinputs, ruleg,  pelvis, rhip      );
			PushBoneRelative( rinputs, rlleg,  ruleg,  rknee,  1 );
			PushBoneRelative( rinputs, rfoot,  rlleg,  rankle    );

			
			Vec3 zero(0, 0, 0), yvec(0, 1, 0), zvec(0, 0, 1);				// TODO: come up with actual values for this desired foot state info
			PushFootStuff(linputs, dood->feet[0], zero, yvec, -1.0f);
			PushFootStuff(rinputs, dood->feet[1], zero, yvec, -1.0f);

			// spinal column bone pos/ori goal satisfaction sensors (also used for scoring)
			Vec3 ppos  = pelvis.rb->GetOrientation().ToMat3().Transpose() * (pelvis.rb->GetCenterOfMass() - desired_pelvis_pos);
			Vec3 pori  = (Quaternion::Reverse(pelvis.rb->GetOrientation()) * p ).ToRVec();
			Vec3 t1ori = (Quaternion::Reverse(torso1.rb->GetOrientation()) * t1).ToRVec();
			Vec3 t2ori = (Quaternion::Reverse(torso2.rb->GetOrientation()) * t2).ToRVec();
			PushVec3( cinputs, ppos  );
			PushVec3( cinputs, pori  );
			//PushVec3( cinputs, t1ori );
			//PushVec3( cinputs, t2ori );

			// don't add any more sensor inputs after this line!
			static bool showed_inputs_count = false;
			if(!showed_inputs_count)
			{
				Debug(((stringstream&)(stringstream() << "#inputs: left = " << linputs.size() << "; right = " << rinputs.size() << "; center = " << cinputs.size() << endl)).str());
				showed_inputs_count = true;
			}

			for(float *iptr = cinputs.data(), *iend = iptr + cinputs.size(); iptr != iend; ++iptr)
				*iptr = tanhf(*iptr);
			for(float *iptr = linputs.data(), *iend = iptr + linputs.size(); iptr != iend; ++iptr)
				*iptr = tanhf(*iptr);
			for(float *iptr = rinputs.data(), *iend = iptr + rinputs.size(); iptr != iend; ++iptr)
				*iptr = tanhf(*iptr);

			brain->nn->arrays[0]->CopyFromVector(cinputs);
			brain->nn->arrays[1]->CopyFromVector(linputs);
			brain->nn->arrays[2]->CopyFromVector(rinputs);

			// evaluate all the things!
			brain->nn->Evaluate();

			// crop outputs list to the number of memories
			vector<float> loutputs(brain->nn->arrays[3]->size), routputs(brain->nn->arrays[4]->size);
			brain->nn->arrays[3]->CopyToVector(loutputs);
			brain->nn->arrays[4]->CopyToVector(routputs);

			// apply the outputs the brain just came up with (also, beginning of scoring stuff)
			const float* loptr = loutputs.data();
			const float* roptr = routputs.data();
			SetJointTorques(loptr, lhip);
			SetJointTorques(roptr, rhip);
			SetJointTorques(loptr, lknee, 1);
			SetJointTorques(roptr, rknee, 1);
			SetJointTorques(loptr, lankle);
			SetJointTorques(roptr, rankle);

			// scoring stuff
			++tick_age;
			if(!experiment_done)
			{
				worst = max(worst, ComputeInstantGoalCost());
				goal_error_cost += worst;

				if(tick_age >= max_tick_age)
				{
					experiment_done = true;
					experiment->GotScore(goal_error_cost, ((TestGame*)dood->game_state)->debug_text);
				}
			}
		}

		float GetBoneDeviation(const CBone& bone) const
		{
			Vec3 initial_com = bone.initial_pos + bone.initial_ori * bone.local_com;
			Vec3 dpos = bone.rb->GetCenterOfMass() - initial_com;

			Vec3 dori = (bone.initial_ori * Quaternion::Reverse(bone.rb->GetOrientation())).ToRVec();

			return dpos.ComputeMagnitude() + dori.ComputeMagnitude() * 4.0f;
		}

		float ComputeInstantGoalCost() const
		{
			static const unsigned int N = 8;

			float errors[N] =
			{
				GetBoneDeviation(torso1),
				GetBoneDeviation(pelvis),
				GetBoneDeviation(luleg),
				GetBoneDeviation(ruleg),
				GetBoneDeviation(llleg),
				GetBoneDeviation(rlleg),
				GetBoneDeviation(lfoot),
				GetBoneDeviation(rfoot),
			};

			float cost_coeffs[N];
			for(unsigned int i = 0; i < N; ++i)
				cost_coeffs[i] = 0.05f;

			float tot = 0.0f;
			for(unsigned int i = 0; i < N; ++i)
				tot += cost_coeffs[i] * errors[i];

			return 1.0f - expf(-tot);
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
		feet.push_back(new SoldierFoot(Bone::string_table["l foot"], Vec3( 0.238f, 0.000f, 0.065f)));
		feet.push_back(new SoldierFoot(Bone::string_table["r foot"], Vec3(-0.238f, 0.000f, 0.065f)));
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
		pos.y -= 0.01f;

		Dood::DoInitialPose();

		Quaternion p, t1, t2;
		imp->GetDesiredTorsoOris(this, p, t1, t2);

		posey->skeleton->GetNamedBone( "pelvis"  )->ori = p;
		posey->skeleton->GetNamedBone( "torso 1" )->ori = t1 * Quaternion::Reverse(p);
		posey->skeleton->GetNamedBone( "torso 2" )->ori = t2 * Quaternion::Reverse(t1);
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
		posey->skeleton->GetNamedBone( "l foot"  )->ori = qlist[2];
		posey->skeleton->GetNamedBone( "r leg 1" )->ori = qlist[3];
		posey->skeleton->GetNamedBone( "r leg 2" )->ori = qlist[4];
		posey->skeleton->GetNamedBone( "r foot"  )->ori = qlist[5];
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
