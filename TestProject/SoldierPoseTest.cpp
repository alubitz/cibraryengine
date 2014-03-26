#include "StdAfx.h"
#include "SoldierPoseTest.h"

#include "Soldier.h"

namespace Test
{
	/*
	 * SoldierPoseTest private implementation struct
	 */
	struct SoldierPoseTest::Imp
	{
		unsigned int num_coeffs;

		struct Genome
		{
			vector<float> coeffs;
			float         score;
			float         min, max;
			unsigned int  trials;
		} best, test;

		unsigned int iteration_counter;
		unsigned int no_improvements_counter;

		enum TestType { Mutant, RedoBest, RedoTest } test_type;

		Imp(unsigned int num_coeffs) : num_coeffs(num_coeffs), iteration_counter(0), no_improvements_counter(0)
		{
			if(unsigned int error = Load())
			{
				best.coeffs.clear();
				best.trials = 0;
				best.score = best.min = best.max = 9001;
			}
			best.coeffs.resize(num_coeffs);

			test_type = RedoBest;
		}

		~Imp() { Save(); }

		unsigned int Load()
		{
			ifstream stream("Files/magicbox", ios::in | ios::binary);
			if(!stream)
				return 1;

			BinaryChunk whole;
			whole.Read(stream);

			if(!stream)
				return 2;

			if(whole.GetName() != "MAGICBOX")
				return 3;
			else
			{
				istringstream ss(whole.data);

				best.score = ReadSingle(ss);
				best.min = ReadSingle(ss);
				best.max = ReadSingle(ss);
				best.trials = ReadUInt32(ss);
				unsigned int num_floats = ReadUInt32(ss);
				best.coeffs.clear();
				for(unsigned int i = 0; i < num_floats; ++i)
					best.coeffs.push_back(ReadSingle(ss));

				if(!ss)
					return 4;
				else
					return 0;
			}
		}

		void Save()
		{
			ofstream stream("Files/magicbox", ios::out | ios::binary);
			if(!stream)
				return;

			stringstream ss;
			WriteSingle(best.score, ss);
			WriteSingle(best.min, ss);
			WriteSingle(best.max, ss);
			WriteUInt32(best.trials, ss);
			WriteUInt32(best.coeffs.size(), ss);
			for(vector<float>::const_iterator iter = best.coeffs.begin(); iter != best.coeffs.end(); ++iter)
				WriteSingle(*iter, ss);

			BinaryChunk bc("MAGICBOX");
			bc.data = ss.str();
			bc.Write(stream);

			for(unsigned int i = 0; i < num_coeffs; ++i)
				Debug(((stringstream&)(stringstream() << "coeffs[" << i << "] = " << best.coeffs[i] << endl)).str());
		}

		void Begin(Soldier* soldier)
		{
			switch(test_type)
			{
				case RedoTest:
					break;
				case RedoBest:
				{
					test = best;
					break;
				}
				case Mutant:
				{
					if(Random3D::RandInt() % 8 == 0)
						test.coeffs[Random3D::RandInt(num_coeffs)] = 0.0f;
					else
					{
						static const unsigned int num_mutations = 3;
						static const float mutation_rate        = 0.004f;

						test = best;
						for(unsigned int i = 0; i < num_mutations; ++i)
							test.coeffs[Random3D::RandInt(num_coeffs)] += Random3D::Rand(-mutation_rate, mutation_rate);
					}

					break;
				}
			}

			soldier->magic_box_coeffs = test.coeffs;
		}

		void End(Soldier* soldier)
		{
			float score = soldier->magic_box_score;

			bool wrote = false;
			switch(test_type)
			{
				case Mutant:
				{
					++no_improvements_counter;

					if(no_improvements_counter >= min(50u, 20 + best.trials / 2) && Random3D::RandInt() % 2 == 0)
						test_type = RedoBest;
					else
					{
						test_type = RedoTest;
						test.score = test.min = test.max = score;
						test.trials = 1;
					}
					break;
				}
				case RedoBest:
				{
					best.min = min(best.min, score);
					best.max = max(best.max, score);
					best.score = (score + best.score * best.trials) / (best.trials + 1);
					++best.trials;

					if(best.trials < 10 + no_improvements_counter / 2)
						test_type = RedoBest;
					else
						test_type = Mutant;

					break;
				}
				case RedoTest:
				{
					test.min = min(test.min, score);
					test.max = max(test.max, score);
					test.score = (score + test.score * test.trials) / (test.trials + 1);
					++test.trials;

					if(test.trials < 10 || test.score < best.max)
					{
						if(test.trials >= 20)
						{
							if(test.score < best.score)
							{
								no_improvements_counter = 0;

								best = test;
								test_type = Mutant;

								Debug(((stringstream&)(stringstream() << "i = " << iteration_counter << "; avg = " << test.score << "; min = " << test.min << "; max = " << test.max << endl)).str());
								wrote = true;
							}
							else
								test_type = Mutant;
						}
						else
							test_type = RedoTest;
					}
					else
						test_type = Mutant;
				}
			}

			if(iteration_counter % 100 == 0 && !wrote)
				Debug(((stringstream&)(stringstream() << "i = " << iteration_counter << "; best = " << best.score << endl)).str());

			++iteration_counter;
		}
	};




	/*
	 * SoldierPoseTest methods
	 */
	SoldierPoseTest::SoldierPoseTest() : imp(NULL), num_inputs(39), num_outputs(9), num_coeffs((num_inputs + 1) * num_outputs) { imp = new Imp(num_coeffs); }
	SoldierPoseTest::~SoldierPoseTest()           { if(imp) { delete imp; imp = NULL; } }

	void SoldierPoseTest::Begin(Soldier* soldier) { imp->Begin(soldier); }
	void SoldierPoseTest::End(Soldier* soldier)   { imp->End(soldier); }
}
