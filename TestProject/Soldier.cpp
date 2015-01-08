#include "StdAfx.h"
#include "Soldier.h"

#include "Gun.h"
#include "WeaponEquip.h"

#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WalkPose.h"

#define DIE_AFTER_ONE_SECOND              0

#define ENABLE_NEW_JETPACKING             0

#define MAX_TICK_AGE					  60

#define NUM_INPUTS                        172
#define NUM_OUTPUTS                       28			// double the number of joint torque axes
#define NUM_MEMORIES                      40			// must be >= NUM_OUTPUTS
#define TARGET_LENGTH                     65			// should be >= NUM_MEMORIES

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
		struct Op
		{
			unsigned char op;
			short ia, ib;

			void Read(istream& ss)
			{
				op = ReadByte (ss);
				ia = ReadInt16(ss);
				ib = ReadInt16(ss);
			}

			void Write(ostream& ss) const
			{
				WriteByte (op, ss);
				WriteInt16(ia, ss);
				WriteInt16(ib, ss);
			}
		};
		vector<Op> ops;

		vector<float> initial_mems;

		GABrain() : ops(), initial_mems(NUM_MEMORIES) { }

		void Evaluate(const vector<float>& inputs, vector<float>& outputs) const
		{
			outputs.clear();

			for(unsigned int i = 0; i < ops.size(); ++i)
			{
				const Op& o = ops[i];
				float a = LookUp(o.ia, inputs, outputs);
				float b = LookUp(o.ib, inputs, outputs);

				if((o.op & 0x1) != 0)
					a = 1.0f - a;
				if((o.op & 0x2) != 0)
					b = 1.0f - b;

				bool op_bit_one = (o.op & 0x4) != 0;
				bool op_bit_two = (o.op & 0x8) != 0;

				float result;
				if(op_bit_two)
				{
					if(op_bit_one)
					{
						result = 0.5f;//((b - a) / sqrtf(a * a + b * b)) * 0.5f + 0.5f;
						//if(result > 1 || result < 0)
						//	DEBUG();
					}
					else
						result = (a + b) * 0.5f;
				}
				else
				{
					if(op_bit_one != 0)
						result = a * b;
					else
						result = a + b - a * b;
				}
				outputs.push_back(result);
			}
		}

		float LookUp(short index, const vector<float>& inputs, vector<float>& outputs) const
		{
			if(index >= 0)
				return (unsigned)index < inputs.size() ? inputs[index] : 0.0f;
			else
			{
				index = outputs.size() + index;
				return index >= 0 ? outputs[index] : 0.0f;
			}
		}

		static GABrain CreateCrossover(const GABrain& a, const GABrain& b)
		{
#if 1
			for(unsigned int i = 0; i < NUM_MEMORIES; ++i)
			{
				// TODO: pick one parent or the other to be the 'primary' owner of this memory
				// recurse through the nodes which contribute to that node's computation
				// for every node, there is a chance that instead of using the original formula, a crossover or mutation will be made
			}
#else
			const vector<Op> *av = &a.ops, *bv = &b.ops;
			if(Random3D::RandInt() % 2 == 0)
				swap(av, bv);

			unsigned int alen = av->size(), blen = bv->size();
			
			unsigned int apart = alen, bpart = blen;
			while(apart + bpart > TARGET_LENGTH)
			{
				if(Random3D::RandInt() % 2 == 0)
				{
					if(apart > 0)
						--apart;
				}
				else
				{
					if(bpart > 0)
						--bpart;
				}
			}

			GABrain child;
			for(unsigned int i = 0; i < apart; ++i)
				child.ops.push_back(a.ops[i]);
			for(unsigned int i = b.ops.size() - bpart; i < b.ops.size(); ++i)
				child.ops.push_back(b.ops[i]);

			for(unsigned int i = 0; i < child.ops.size(); ++i)
			{
				Op& o = child.ops[i];
				if(Random3D::RandInt() % 35 == 0)
					o.op = (unsigned char)Random3D::RandInt(16);
				if(Random3D::RandInt() % 35 == 0)
					o.ia = (short)Random3D::RandInt(-(signed)i, NUM_INPUTS * 2 + NUM_MEMORIES);
				if(Random3D::RandInt() % 35 == 0)
					o.ib = (short)Random3D::RandInt(-(signed)i, NUM_INPUTS * 2 + NUM_MEMORIES);
			}

			for(unsigned int i = 0; i < child.initial_mems.size(); ++i)
				child.initial_mems[i] = Random3D::RandInt() % 2 == 0 ? a.initial_mems[i] : b.initial_mems[i];

			child.MutateMemories();

			return child;
#endif
		}

		void MutateMemories()
		{
			static const unsigned int mutation_every = 10;
			static const float        mutation_scale = 0.5f;

			for(unsigned int i = 0; i < initial_mems.size(); ++i)
			{
				float& f = initial_mems[i];
				if(Random3D::RandInt() % mutation_every == 0)
					f += Random3D::Rand(-mutation_scale, mutation_scale);
				f = max(0.0f, min(1.0f, f));
			}
		}

		void Read(istream& ss)
		{
			unsigned int num_memories = ReadUInt32(ss);
			initial_mems.resize(max((unsigned)NUM_MEMORIES, num_memories));
			for(unsigned int i = 0; i < num_memories; ++i)
				initial_mems[i] = ReadSingle(ss);
			initial_mems.resize(NUM_MEMORIES);

			ops.clear();

			unsigned int count = ReadUInt32(ss);
			for(unsigned int i = 0; i < count; ++i)
			{
				Op op;
				op.Read(ss);
				ops.push_back(op);
			}
		}

		void Write(ostream& ss) const
		{
			WriteUInt32(NUM_MEMORIES, ss);
			for(unsigned int i = 0; i < NUM_MEMORIES; ++i)
				WriteSingle(initial_mems[i], ss);

			unsigned int count = ops.size();
			WriteUInt32(count, ss);
			for(unsigned int i = 0; i < count; ++i)
				ops[i].Write(ss);
		}
	};

	struct Experiment
	{
		struct Record
		{
			GABrain brain;
			float score;

			Record() : score(0) { }

			bool operator <(const Record& r) { return score < r.score; }
		};

		unsigned int batch, genome, trial;

		vector<Record*> genepool;

		Experiment()
		{
			batch = genome = trial = 0;

			ifstream file("Files/brains", ios::in | ios::binary);
			if(!file)
				Debug("Failed to load brains!\n");
			else
			{
				unsigned int num_brains   = ReadUInt32(file);
				genepool.clear();
				for(unsigned int i = 0; i < num_brains; ++i)
				{
					Record record;
					record.brain.Read(file);

					if(file.bad())
					{
						Debug(((stringstream&)(stringstream() << "Error while trying to load brains! " << genepool.size() << " brains loaded before error" << endl)).str());
						file.close();
						return;
					}
					else
						genepool.push_back(new Record(record));
				}

				file.close();

				Debug(((stringstream&)(stringstream() << "Successfully loaded " << genepool.size() << " brains from file" << endl)).str());
			}
		}

		~Experiment()
		{
			SaveGenepool("Files/brains");

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
					genepool[i]->brain.Write(file);

				file.close();
			}
		}

		static const unsigned int num_parents = 20;
		static const unsigned int num_trials  = 18;

		GABrain NextBrain()
		{
			static const unsigned int first_gen_size = num_parents;
			static const float initial_rand = 0.0f;

			if(genepool.empty())
				for(unsigned int i = 0; i < first_gen_size; ++i)
				{
					Record* result = new Record();
					for(unsigned int i = 0; i < TARGET_LENGTH; ++i)
					{
						GABrain::Op op;
						op.op = 0x0;
						op.ia = 0;
						op.ib = 0;
						result->brain.ops.push_back(op);
					}
					genepool.push_back(result);
				}

			return genepool[genome]->brain;
		}

		void NextGeneration()
		{
			static const unsigned int children_per_pair  = 1;
			static const unsigned int mutants_per_parent = 1;

			for(unsigned int i = 0; i < genepool.size(); ++i)
				for(unsigned int j = i + 1; j < genepool.size(); ++j)
					if(*genepool[j] < *genepool[i])
						swap(genepool[i], genepool[j]);

			while(genepool.size() > num_parents)
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
			SaveGenepool(filename);

			for(unsigned int i = 0; i < actual_parents; ++i)
			{
				Record* p1 = genepool[i];
				for(unsigned int j = 0; j < mutants_per_parent; ++j)
				{
					Record* c = new Record();
					c->brain = GABrain::CreateCrossover(p1->brain, p1->brain);
					genepool.push_back(c);
				}

				for(unsigned int j = i + 1; j < actual_parents; ++j)
					for(unsigned int k = 0; k < children_per_pair; ++k)
					{
						Record* p2 = genepool[j];

						Record* c = new Record();
						c->brain = GABrain::CreateCrossover(p1->brain, p2->brain);
						genepool.push_back(c);
					}
			}

			Debug(((stringstream&)(stringstream() << "generation " << batch << "; next gen will have " << genepool.size() << " genomes" << endl)).str());
			for(unsigned int i = 0; i < actual_parents; ++i)
				Debug(((stringstream&)(stringstream() << "\tparent " << i << " score = " << genepool[i]->score << "; len = " << genepool[i]->brain.ops.size() << endl)).str());

			for(unsigned int i = 0; i < genepool.size(); ++i)
				genepool[i]->score = 0;
		}

		void GotScore(float score, string& ss_target)
		{
			float& gscore = genepool[genome]->score;
			gscore += score;

			float score_to_beat = genome >= num_parents ? genepool[num_parents - 1]->score : -1;

			{	// curly braces for scope
				stringstream ss;

				ss << "generation " << batch << ", genome " << genome << " of " << genepool.size() << ", trial " << trial << " of " << num_trials << "; cost so far = " << gscore / num_trials << endl;
				for(unsigned int i = 0; i < genome && i < num_parents; ++i)
					ss << endl << "top scorer # " << (i + 1) << " = " << genepool[i]->score;
				
				ss_target = ss.str();
			}

			bool quick = score_to_beat != -1 && gscore >= score_to_beat * num_trials;

			++trial;
			if(trial == num_trials || quick)
			{
				genepool[genome]->score /= num_trials;

				stringstream ss;
				ss << "b " << batch << " g " << genome << " score = " << gscore << "; len = " << genepool[genome]->brain.ops.size();
				if(quick)
					ss << "; fail (" << trial << " / " << num_trials << ")" << endl;
				else
					ss << "; pass" << endl;
				Debug(ss.str());

				for(unsigned int i = genome; i != 0; --i)
				{
					if(genepool[i]->score < genepool[i - 1]->score)
						swap(genepool[i], genepool[i - 1]);
					else
						break;
				}

				trial = 0;
				++genome;

				if(genome == genepool.size())
				{
					NextGeneration();

					genome = 0;
					++batch;
				}
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

			CBone() { }
			CBone(const Soldier* dood, const string& name) : name(name), rb(dood->RigidBodyForNamedBone(name)), posey(dood->posey->skeleton->GetNamedBone(name)), local_com(rb->GetLocalCoM()) { }

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

		GABrain brain;
		vector<float> memories;
		vector<float> old_inputs;
		float feet_error_cost, goal_error_cost, exertion_cost;
		bool feet_failed;

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

		Imp() :
			init(false),
			experiment_done(false),
			feet_error_cost(0),
			goal_error_cost(0),
			exertion_cost(0),
			feet_failed(true),
			timestep(0),
			inv_timestep(0),
			tick_age(0),
			max_tick_age(MAX_TICK_AGE)
		{
		}

		~Imp() { }

		void RegisterBone (CBone& bone)   { all_bones.push_back(&bone); }
		void RegisterJoint(CJoint& joint) { all_joints.push_back(&joint); }

		void InitBrain(Soldier* dood)
		{
			brain = experiment->NextBrain();
			memories = brain.initial_mems;

			old_inputs.clear();
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
			float jpn_angle = 1.0f;
			float jpn_force = 150.0f;		// 98kg * 15m/s^2 accel / 10 nozzles ~= 150N per nozzle

			jetpack_nozzles.push_back(JetpackNozzle( lshoulder, Vec3( 0.442619f, 1.576419f, -0.349652f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( lshoulder, Vec3( 0.359399f, 1.523561f, -0.366495f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( lshoulder, Vec3( 0.277547f, 1.480827f, -0.385142f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rshoulder, Vec3(-0.359399f, 1.523561f, -0.366495f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rshoulder, Vec3(-0.442619f, 1.576419f, -0.349652f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rshoulder, Vec3(-0.277547f, 1.480827f, -0.385142f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( lfoot,     Vec3( 0.237806f, 0.061778f,  0.038247f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( lfoot,     Vec3( 0.238084f, 0.063522f, -0.06296f  ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rfoot,     Vec3(-0.237806f, 0.061778f,  0.038247f ), upward, jpn_angle, jpn_force ));
			jetpack_nozzles.push_back(JetpackNozzle( rfoot,     Vec3(-0.238084f, 0.063522f, -0.06296f  ), upward, jpn_angle, jpn_force ));

			InitBrain(dood);

			desired_pelvis_pos = pelvis.rb->GetCenterOfMass();
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

		static void DebugQuatRelative(const Quaternion& bone, const Quaternion& parent)
		{
			Quaternion q = Quaternion::Reverse(parent) * bone;
			Debug(((stringstream&)(stringstream() << "\t\t\tQuaternion( " << q.w << "f, " << q.x << "f, " << q.y << "f, " << q.z << "f )," << endl)).str());
		}

		void Update(Soldier* dood, const TimingInfo& time)
		{
			if(!init)
			{
				Init(dood);
				init = true;
			}

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

#if ENABLE_NEW_JETPACKING
			float dood_mass;
			Vec3 dood_com, com_vel, angular_momentum;
			ComputeMomentumStuff(dood, dood_mass, dood_com, com_vel, angular_momentum);

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
#if 1
			switch(experiment->trial % 6)
			{
				case 1: desired_pelvis_pos.y -= 0.035f * timestep; break;
				case 2: dood->yaw            += 0.1f   * timestep; break;
				case 3: dood->yaw            -= 0.1f   * timestep; break;
				case 4: dood->pitch          += 0.1f   * timestep; break;
				case 5: dood->pitch          -= 0.1f   * timestep; break;
				default: break;
			}
#endif


			// stuff info about the dood into an array, which will then be crammed into a brain... maybe something cool will happen
			vector<float> inputs;

			struct PushVec3
			{
				PushVec3(vector<float>& inputs, const Vec3& vec)
				{
					inputs.push_back(vec.x);
					inputs.push_back(vec.y);
					inputs.push_back(vec.z);
				}
			};

			struct PushBoneRelative
			{
				PushBoneRelative(vector<float>& inputs, const CBone& pushme, const CBone& rel, const CJoint& joint)
				{
					Mat3 unrotate = rel.rb->GetOrientation().ToMat3().Transpose();
					PushVec3(inputs, unrotate * (pushme.rb->GetCenterOfMass() - rel.rb->GetCenterOfMass()));
					PushVec3(inputs, unrotate * pushme.rb->GetLinearVelocity() * 0.02f);
					PushVec3(inputs, unrotate * pushme.rb->GetAngularVelocity() * 0.02f);

					Mat3 rvecmat = joint.sjc->axes * Quaternion::Reverse(joint.a->rb->GetOrientation()).ToMat3() * joint.b->rb->GetOrientation().ToMat3() * joint.sjc->axes.Transpose();
					Vec3 rvec = Quaternion::FromRotationMatrix(rvecmat).ToRVec();

					PushVec3(inputs, rvec);
				}
			};

			// root bone info
			Mat3 reverse_pelvis = Quaternion::Reverse(pelvis.rb->GetOrientation()).ToMat3();
			PushVec3(inputs, reverse_pelvis * Vec3(0, 1, 0));
			PushVec3(inputs, reverse_pelvis * pelvis.rb->GetLinearVelocity()  * 0.02f);
			PushVec3(inputs, reverse_pelvis * pelvis.rb->GetAngularVelocity() * 0.02f);

			// specifying other bones relative to that
			PushBoneRelative( inputs, torso1, pelvis, spine1 );
			PushBoneRelative( inputs, torso2, torso1, spine2 );
			PushBoneRelative( inputs, luleg,  pelvis, lhip   );
			PushBoneRelative( inputs, llleg,  luleg,  lknee  );
			PushBoneRelative( inputs, lfoot,  llleg,  lankle );
			PushBoneRelative( inputs, ruleg,  pelvis, rhip   );
			PushBoneRelative( inputs, rlleg,  ruleg,  rknee  );
			PushBoneRelative( inputs, rfoot,  rlleg,  rankle );

			// foot stuff (contact point state info & desired values for position and normal vector, and ETA to get there)
			struct PushFootStuff
			{
				PushFootStuff(vector<float>& inputs, const Dood::FootState* foot, const Vec3& pos, const Vec3& normal, float eta)
				{
					const SoldierFoot* sf = (SoldierFoot*)foot;
					const RigidBody*   rb = sf->body;
					Mat3 unrotate    = rb->GetOrientation().ToMat3().Transpose();
					Vec3 untranslate = rb->GetCenterOfMass();

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
					
					// goal info												// TODO: change this once desired foot pos has reasonable values
					PushVec3(inputs, Vec3());//unrotate * (pos - untranslate));
					PushVec3(inputs, unrotate * normal);
					inputs.push_back(eta);
				}
			};
			Vec3 zero(0, 0, 0), yvec(0, 1, 0), zvec(0, 0, 1);				// TODO: come up with actual values for this desired foot state info
			PushFootStuff(inputs, dood->feet[0], zero, yvec, -1.0f);
			PushFootStuff(inputs, dood->feet[1], zero, yvec, -1.0f);

			// spinal column bone pos/ori goal satisfaction sensors (also used for scoring)
			Vec3 ppos  = pelvis.rb->GetOrientation().ToMat3().Transpose() * (pelvis.rb->GetCenterOfMass() - desired_pelvis_pos);
			Vec3 pori  = (Quaternion::Reverse(pelvis.rb->GetOrientation()) * p ).ToRVec();
			Vec3 t1ori = (Quaternion::Reverse(torso1.rb->GetOrientation()) * t1).ToRVec();
			Vec3 t2ori = (Quaternion::Reverse(torso2.rb->GetOrientation()) * t2).ToRVec();
			Vec3 hori  = (Quaternion::Reverse(head.rb->GetOrientation()) * yaw_ori).ToRVec();
			PushVec3( inputs, ppos  );
			PushVec3( inputs, pori  );
			PushVec3( inputs, t1ori );
			PushVec3( inputs, t2ori );
			PushVec3( inputs, hori  );

			// gun forward vector goal satisfaction sensors (ditto)
			Vec3 desired_gun_fwd = yaw_ori * zvec;
			Vec3 actual_gun_fwd  = gun_rb->GetOrientation() * zvec;
			Vec2 gunxz_d = Vec2::Normalize(Vec2(desired_gun_fwd.x, desired_gun_fwd.z));
			Vec2 gunxz_a = Vec2::Normalize(Vec2(actual_gun_fwd.x,  actual_gun_fwd.z ));
			
			// TODO: add something for the gun's up vector?
			float gunx = asinf(max(-1.0f, min(1.0f, Vec2::Dot(gunxz_a, Vec2(gunxz_d.y, -gunxz_d.x)))));
			float guny = acosf(max(-1.0f, min(1.0f, desired_gun_fwd.y * actual_gun_fwd.y)));
			inputs.push_back(gunx);
			inputs.push_back(guny);

			// don't add any more sensor inputs after this line!
			static bool showed_inputs_count = false;
			if(!showed_inputs_count)
			{
				Debug(((stringstream&)(stringstream() << "#inputs = " << inputs.size() << endl)).str());
				showed_inputs_count = true;
			}

			// final preparations for letting the brain evaluate
			inputs.resize(NUM_INPUTS * 2 + NUM_MEMORIES);

			if(!old_inputs.empty())
			{
				for(unsigned int i = 0; i < NUM_INPUTS; ++i)
				{
					inputs[NUM_INPUTS + i] = inputs[i] - old_inputs[i];
					old_inputs[i] = inputs[i];
				}
			}
			else
			{
				// input array slots for old inputs will have been default-initialized to zero
				old_inputs.resize(NUM_INPUTS);
				for(unsigned int i = 0; i < NUM_INPUTS; ++i)
					old_inputs[i] = inputs[i];
			}

			for(unsigned int i = 0; i < NUM_INPUTS * 2; ++i)
				inputs[i] = tanhf(inputs[i]) * 0.5f + 0.5f;

			memcpy(inputs.data() + NUM_INPUTS * 2, memories.data(), NUM_MEMORIES * sizeof(float));

			// evaluate all the things!
			vector<float> outputs;
			brain.Evaluate(inputs, outputs);

			// crop outputs list to the number of memories
			memories.clear();
			if(outputs.size() >= NUM_MEMORIES)
				for(unsigned int i = outputs.size() - NUM_MEMORIES; i < outputs.size(); ++i)
					memories.push_back(outputs[i]);
			else
			{
				memories.resize(NUM_MEMORIES);
				for(unsigned int i = 0; i < outputs.size(); ++i)
					memories[i] = outputs[i];
			}

			// apply the outputs the brain just came up with (also, beginning of scoring stuff)
			struct SetJointTorques
			{
				SetJointTorques(float& part_cost, const float*& optr, CJoint& joint, unsigned int n = 3)
				{
					part_cost = 0.0f;

					const float* minptr = (float*)&joint.sjc->min_torque;
					const float* maxptr = (float*)&joint.sjc->max_torque;

					Vec3 v;
					float* vptr = (float*)&v;

					for(const float* oend = optr + n * 2; optr != oend; ++vptr, ++minptr, ++maxptr)
					{
						float plus  = *(optr++);
						float minus = *(optr++);

						*vptr = plus * *maxptr + minus * *minptr;

						part_cost += plus + minus + plus * minus;
					}

					joint.SetOrientedTorque(v);
				}
			};

			const float* optr = memories.data();
			float pcosts[6];
			SetJointTorques(pcosts[0], optr, lhip);
			SetJointTorques(pcosts[1], optr, rhip);
			SetJointTorques(pcosts[2], optr, lknee, 1);
			SetJointTorques(pcosts[3], optr, rknee, 1);
			SetJointTorques(pcosts[4], optr, lankle);
			SetJointTorques(pcosts[5], optr, rankle);

			// scoring stuff
			for(unsigned int i = 0; i < 6; ++i)
				exertion_cost += pcosts[i];

			++tick_age;
			if(!experiment_done)
			{
				if(dood->feet[0]->contact_points.empty() || dood->feet[1]->contact_points.empty())
					feet_failed = false;
				if(!feet_failed)
					++feet_error_cost;

				goal_error_cost += ComputeInstantGoalCost(ppos, pori, t1ori, t2ori, hori, gunx, guny);

				if(tick_age >= max_tick_age)
				{
					experiment_done = true;
					experiment->GotScore(ComputeTotalCost(), ((TestGame*)dood->game_state)->debug_text);
				}
			}
		}

		float ComputeInstantGoalCost(const Vec3& ppos, const Vec3& pori, const Vec3& t1ori, const Vec3& t2ori, const Vec3& hori, float gunx, float guny)
		{
			float exp_coeff = 0.1f;

			float errors[7] =
			{
				ppos.ComputeMagnitudeSquared(),
				pori.ComputeMagnitudeSquared(),
				t1ori.ComputeMagnitudeSquared(),
				t2ori.ComputeMagnitudeSquared(),
				hori.ComputeMagnitudeSquared(),
				gunx * gunx,
				guny * guny
			};

			float goodness[7];
			for(unsigned int i = 0; i < 7; ++i)
				goodness[i] = expf(-exp_coeff * errors[i]);

			float use_goodness[7] = 
			{
				goodness[0],
				goodness[1],
				goodness[1] * goodness[2],
				goodness[1] * goodness[2] * goodness[3],
				goodness[1] * goodness[2] * goodness[3] * goodness[4],
				goodness[1] * goodness[2] * goodness[3] * goodness[5],
				goodness[1] * goodness[2] * goodness[3] * goodness[6]
			};

			float cost_coeffs[7] = { 0.5f, 0.2f, 0.2f, 0.2f, 0.15f, 0.2f, 0.2f };
			float tot = 0.0f;
			for(unsigned int i = 0; i < 7; ++i)
			{
				if(use_goodness[i] == 0.0f)
					DEBUG();
				tot += cost_coeffs[i] * -logf(use_goodness[i]) / exp_coeff;
			}

			return tot;
		}

		float ComputeTotalCost()
		{
			float exp_coeff = 0.025f;
			float errors[3] = { feet_error_cost, goal_error_cost, exertion_cost };
			float goodness[3];
			for(unsigned int i = 0; i < 3; ++i)
				goodness[i] = exp(-exp_coeff * errors[i]);

			float use_goodness[3] =
			{
				goodness[0],
				goodness[0] * goodness[1],
				goodness[0] * goodness[1] * goodness[2]
			};

			float cost_coeffs[3] = { 10.0f, 1.0f, 0.02f };
			float tot = 0.0f;
			for(unsigned int i = 0; i < 3; ++i)
			{
				if(use_goodness[i] == 0.0f)
					DEBUG();
				tot += cost_coeffs[i] * -logf(use_goodness[i]) / exp_coeff;
			}

			return tot;
		}
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

		yaw = Random3D::Rand(float(M_PI) * 2.0f);

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

	void Soldier::PreUpdatePoses(const TimingInfo& time)
	{
		imp->Update(this, time);
	}

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
