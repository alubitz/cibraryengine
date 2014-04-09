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

			unsigned int id;

			float repro_share;
		} test;

		list<Genome> genomes;
		unsigned int next_genome;

		void ComputeGenomeWeight(Genome& g, float min, float max)
		{
			if(g.trials < 20)
				g.repro_share = 0.0f;
			else
			{
				if(min == max)
					g.repro_share = 1.0f;
				else
				{
					float s = (g.score - min) / (max - min);
					g.repro_share = 1.0f / (s * s + 0.04f);
				}
			}
		}

		unsigned int iteration_counter;

		Imp(unsigned int num_coeffs) : num_coeffs(num_coeffs), iteration_counter(0)
		{
			if(unsigned int error = Load())
			{
				Genome g;
				g.coeffs.clear();
				g.coeffs.resize(num_coeffs);

				g.trials = 0;
				g.score = g.min = g.max = 0.0f;
				g.repro_share = 1.0f;

				g.id = 0;
				genomes.push_back(g);

				g.id = 1;
				genomes.push_back(g);
			}

			next_genome = genomes.size();
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

				unsigned int count = ReadUInt32(ss);
				if(count < 2)
					return 4;
				for(unsigned int i = 0; i < count; ++i)
				{
					Genome g;
					g.score  = ReadSingle(ss);
					g.min    = ReadSingle(ss);
					g.max    = ReadSingle(ss);
					g.trials = ReadUInt32(ss);
					g.id     = i;

					g.coeffs.clear();
					unsigned int num_floats = ReadUInt32(ss);
					for(unsigned int j = 0; j < num_floats; ++j)
						g.coeffs.push_back(ReadSingle(ss));

					genomes.push_back(g);
				}

				if(!ss)
					return 5;
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

			WriteUInt32(genomes.size(), ss);
			for(list<Genome>::iterator iter = genomes.begin(); iter != genomes.end(); ++iter)
			{
				const Genome& g = *iter;

				WriteSingle(g.score,  ss);
				WriteSingle(g.min,    ss);
				WriteSingle(g.max,    ss);
				WriteUInt32(g.trials, ss);

				WriteUInt32(g.coeffs.size(), ss);
				for(vector<float>::const_iterator iter = g.coeffs.begin(); iter != g.coeffs.end(); ++iter)
					WriteSingle(*iter, ss);
			}

			BinaryChunk bc("MAGICBOX");
			bc.data = ss.str();
			bc.Write(stream);
		}

		void Begin(Soldier* soldier)
		{
			++iteration_counter;

			float gmin, gmax;
			for(list<Genome>::iterator iter = genomes.begin(); iter != genomes.end(); ++iter)
			{
				if(iter == genomes.begin())
					gmin = gmax = iter->score;
				else
				{
					gmin = min(gmin, iter->score);
					gmax = max(gmax, iter->score);
				}
			}
			float grange = gmax - gmin;

			for(list<Genome>::iterator iter = genomes.begin(); iter != genomes.end(); ++iter)
				ComputeGenomeWeight(*iter, gmin, gmax);

			if(genomes.size() > 20)
			{
				for(list<Genome>::iterator iter = genomes.begin(); iter != genomes.end();)
				{
					if(Random3D::RandInt() % 85 == 0 && (Random3D::Rand(grange) + gmin < iter->score || iter->trials > 30))
					{
						Debug(((stringstream&)(stringstream() << "deceased = " << iter->id << "; score = " << iter->score << "; n = " << iter->trials << endl)).str());
						iter = genomes.erase(iter);
					}
					else
						++iter;
				}
			}

			if(Random3D::RandInt() % (genomes.size() < 75 ? 2 : 20) == 0)
			{
				float total_repro = 0.0f;
				for(list<Genome>::iterator iter = genomes.begin(); iter != genomes.end(); ++iter)
					total_repro += iter->repro_share;

				float r1 = Random3D::Rand(total_repro);
				for(list<Genome>::iterator iter = genomes.begin(); iter != genomes.end(); ++iter)
				{
					r1 -= iter->repro_share;
					if(r1 <= 0.0f)
					{
						float r2 = Random3D::Rand(total_repro - iter->repro_share);
						for(list<Genome>::iterator jter = genomes.begin(); jter != genomes.end(); ++jter)
						{
							if(iter != jter)
							{
								r2 -= jter->repro_share;
								if(r2 <= 0.0f)
								{
									if(iter->id == jter->id)
										break;

									test = *iter;
									for(unsigned int i = 0; i < num_coeffs; ++i)
										if(Random3D::RandInt() % 2 == 0)
											test.coeffs[i] = jter->coeffs[i];

									Debug(((stringstream&)(stringstream() << "parent a = " << test.id << "; parent b = " << jter->id << "; child = " << next_genome << endl)).str());
					
									test.id = next_genome++;
									if(Random3D::RandInt() % 2 != 1000)		// a mutation is not always necessary if the parents are sufficiently different
									{
										if(Random3D::RandInt() % 8 == 0)
											test.coeffs[Random3D::RandInt(num_coeffs)] = 0.0f;
										else
										{
											static const unsigned int num_mutations = 7;
											static const float        mutation_rate = 0.01f;

											for(unsigned int i = 0; i < num_mutations; ++i)
											{
												unsigned int index = Random3D::RandInt(num_coeffs);
	#if 0
												unsigned int j;
												for(j = 0; j < 200; ++j)
												{
													unsigned int maybe_index = Random3D::RandInt(num_coeffs);
													if(test.coeffs[maybe_index] != 0.0f)
													{
														index = maybe_index;
														break;
													}
												}
	#endif										// enable to make mutations prefer to alter already-nonzero coefficients
												test.coeffs[index] += Random3D::Rand(-mutation_rate, mutation_rate);
											}
										}
									}

									test.trials = 0;
									genomes.push_back(test);

									break;
								}
							}
						}

						break;
					}
				}
			}
			else
			{
				unsigned int r = Random3D::RandInt(genomes.size());
				for(list<Genome>::iterator iter = genomes.begin(); iter != genomes.end(); ++iter, --r)
					if(r == 0)
					{
						test = *iter;
						break;
					}
			}
			
			soldier->magic_box_coeffs = test.coeffs;
			soldier->magic_box_coeffs.resize(num_coeffs);
		}

		void End(Soldier* soldier)
		{
			float score = soldier->magic_box_score;

			for(list<Genome>::reverse_iterator iter = genomes.rbegin(); iter != genomes.rend(); ++iter)
				if(iter->id == test.id)
				{
					Genome& g = *iter;
					if(g.trials > 0)
					{
						g.min = min(g.min, score);
						g.max = max(g.max, score);
						g.score = (score + g.score * g.trials) / (g.trials + 1);
					}
					else
						g.min = g.max = g.score = score;
					++g.trials;

					Debug(((stringstream&)(stringstream() << "id = " << g.id << "; avg = " << g.score << "; min = " << g.min << "; max = " << g.max << "; n = " << g.trials << endl)).str());

					break;
				}
		}


		void PrintClosingDebugInfo()
		{
			Debug(((stringstream&)(stringstream() << genomes.size() << " final genomes:" << endl)).str());
			for(list<Genome>::iterator iter = genomes.begin(); iter != genomes.end(); ++iter)
			{
				const Genome& g = *iter;
				Debug(((stringstream&)(stringstream() << "\tid = " << g.id << "; avg = " << g.score << "; min = " << g.min << "; max = " << g.max << "; n = " << g.trials << endl)).str());
			}
		}
	};




	/*
	 * SoldierPoseTest methods
	 */
	SoldierPoseTest::SoldierPoseTest() : imp(NULL), num_inputs(72), num_outputs(18), num_coeffs((num_inputs + 1) * num_outputs) { imp = new Imp(num_coeffs); }
	SoldierPoseTest::~SoldierPoseTest()                 { if(imp) { delete imp; imp = NULL; } }

	void SoldierPoseTest::Begin(Soldier* soldier)       { imp->Begin(soldier); }
	void SoldierPoseTest::End(Soldier* soldier)         { imp->End(soldier); }

	void SoldierPoseTest::PrintClosingDebugInfo() const { imp->PrintClosingDebugInfo(); }
}
