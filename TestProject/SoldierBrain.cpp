#include "StdAfx.h"
#include "SoldierBrain.h"

namespace Test
{
	using namespace CibraryEngine;
	
	/*
	 * SoldierBrain private implementation struct
	 */
	struct SoldierBrain::Imp
	{
		struct Genome
		{
			vector<float> brain;
			float score;
			bool crossover;

			Genome() : brain(), score(-1), crossover(false) { }
			Genome(unsigned int size) : brain(size), score(-1), crossover(false) { }
		};

		vector<Genome> genomes;
		unsigned int batch, active_genome, trial;
		unsigned int num_inputs, num_outputs, num_memories, brain_size;

		vector<float> memory;

		bool finished;

		string debug_text;

		Imp() :
			genomes(),
			batch(0),
			active_genome(0),
			trial(0),
			num_inputs(0),
			num_outputs(0),
			num_memories(0),
			brain_size(0),
			memory(),
			finished(false),
			debug_text("")
		{
		}

		void Load(const string& filename)
		{
			genomes.clear();

			ifstream file(filename, ios::in | ios::binary);
			if(!file)
				Debug("Unable to open brain!\n");
			else
			{
				BinaryChunk whole;
				whole.Read(file);

				if(!file)
				{
					Debug("Stream read error loading brain!\n");

					file.close();
					return;
				}
				else
				{
					file.close();

					if(whole.GetName() != "SLDBRAIN")
					{
						Debug("Brain file hasn't got the right header!\n");
						return;
					}
					else
					{
						istringstream ss(whole.data);
				
						num_inputs   = ReadUInt32(ss);
						num_outputs  = ReadUInt32(ss);
						num_memories = ReadUInt32(ss);

						if(num_inputs > 1024 || num_outputs > 1024 || num_memories > 1024)
							Debug(((stringstream&)(stringstream() << "Warning: large brain dimensions! inputs = " << num_inputs << "; outputs = " << num_outputs << "; memories = " << num_memories << endl)).str());

						brain_size = (num_inputs + num_memories) * (num_outputs + num_memories);

						unsigned int num_brains = ReadUInt32(ss);
						for(unsigned int i = 0; i < num_brains; ++i)
						{
							unsigned int check = ReadUInt32(ss);
							if(check != brain_size)
							{
								Debug("Brains stopped being aligned!\n");
								return;
							}
							else
							{
								Genome genome(brain_size);
								for(unsigned int j = 0; j < brain_size; ++j)
									genome.brain[j] = ReadSingle(ss);

								if(ss.bad())
								{
									Debug("Brains ran out mid-brain\n");
									return;
								}
								else
									genomes.push_back(genome);
							}
						}

						Debug(((stringstream&)(stringstream() << "Successfully loaded " << num_brains << " brains from file" << endl)).str());
					}
				}
			}
		}

		void Save(const string& filename, const vector<Genome>& genomes)
		{
			ofstream file(filename, ios::out | ios::binary);
			if(!file)
				Debug("File error trying to save brain!\n");
			else
			{
				stringstream ss;
			
				WriteUInt32(num_inputs,   ss);
				WriteUInt32(num_outputs,  ss);
				WriteUInt32(num_memories, ss);

				WriteUInt32(genomes.size(), ss);
				for(unsigned int i = 0; i < genomes.size(); ++i)
				{
					WriteUInt32(brain_size, ss);
					const vector<float>& brain = genomes[i].brain;
					for(unsigned int j = 0; j < brain_size; ++j)
						WriteSingle(brain[j], ss);
				}

				BinaryChunk bc("SLDBRAIN");
				bc.data = ss.str();
				bc.Write(file);

				file.close();
			}
		}

		void CreateCrossover(const Genome& parent_a, const Genome& parent_b, Genome& result)
		{
			const float* a_ptr = parent_a.brain.data();
			const float* b_ptr = parent_b.brain.data();

			float* result_ptr  = result.brain.data();
			float* results_end = result_ptr + brain_size;

			for(; result_ptr != results_end; ++result_ptr, ++a_ptr, ++b_ptr)
			{
				unsigned int r = Random3D::RandInt() % 2;
				*result_ptr = (r ? *a_ptr : *b_ptr);
				//if(r == 0)
				//	*result_ptr = (*a_ptr + *b_ptr) * 0.5f;
				//else if(r == 1)
				//	*result_ptr = *a_ptr;
				//else
				//	*result_ptr = *b_ptr;
			}
		}

		void Mutate(Genome& genome)
		{
			static const unsigned int num_mutations = 3;
			static const float        mutation_rate = 0.02f;

			// randomly modify a few elements of the coefficient matrix
			for(unsigned int i = 0; i < num_mutations; ++i)
			{	
				float& coeff = genome.brain[Random3D::RandInt(brain_size)];

				if(coeff != 0 && fabs(coeff) < mutation_rate && Random3D::RandInt() % 10 == 0)
					coeff = 0.0f;
				else
					coeff += Random3D::Rand(-mutation_rate, mutation_rate);
			}
		}

		void SwapMemoryIndices(Genome& genome)
		{
			float* brain = genome.brain.data();

			unsigned int a = Random3D::RandInt(num_memories);
			unsigned int b = (a + Random3D::RandInt(num_memories - 1)) % num_memories;

			unsigned int row_size = num_inputs  + num_memories;
			unsigned int col_size = num_outputs + num_memories;

			unsigned int ar = (a + num_outputs) * row_size, br = (b + num_outputs) * row_size;
			for(unsigned int i = 0 ; i < row_size; ++i)
				swap(brain[ar + i], brain[br + i]);
			for(unsigned int i = 0; i < col_size; ++i)
				swap(brain[i * row_size + a + num_inputs], brain[i * row_size + b + num_inputs]);
		}

		void CreateNextGen()
		{
			static const unsigned int parents             = 10;
			static const unsigned int mutants_per_parent  = 4;
			static const unsigned int crossovers_per_pair = 2;
			static const unsigned int crossovers_begin    = parents * mutants_per_parent;
			static const unsigned int generation_size     = crossovers_begin + parents * (parents - 1) * crossovers_per_pair / 2;

			static const unsigned int num_index_swaps     = 1;

			if(batch == 0)
				Debug(((stringstream&)(stringstream() << "generation size = " << generation_size << endl << endl)).str());

			float tot = 0.0f;
			for(unsigned int i = 0; i < genomes.size(); ++i)
			{
				float item_score = genomes[i].score;
				tot += item_score;
			}
			tot /= genomes.size();

			float ptot = 0.0f;
			unsigned int crossover_count = 0;
			for(unsigned int i = 0; i < parents && i < genomes.size(); ++i)
			{
				unsigned int best = i;
				for(unsigned int j = i + 1; j < genomes.size(); ++j)
					if(genomes[j].score > genomes[best].score)
						best = j;
				swap(genomes[best], genomes[i]);

				ptot += genomes[i].score;
				if(genomes[i].crossover)
					++crossover_count;
			}
			unsigned int pcount = min(parents, genomes.size());
			ptot /= pcount;

			debug_text = ((stringstream&)(stringstream() << "batch[" << batch << "] top " << pcount << " avg = " << ptot << "; full avg = " << tot << "; best = " << genomes[0].score << "; crossovers = " << crossover_count << endl)).str();
			Debug(debug_text);

			genomes.resize(max(genomes.size(), generation_size));
			for(unsigned int i = 0; i < generation_size; ++i)
				genomes[i].brain.resize(brain_size);

			// do crossovers
			unsigned int index = crossovers_begin;
			for(unsigned int i = 0; i < parents; ++i)
				for(unsigned int j = i + 1; j < parents; ++j)
					for(unsigned int k = 0; k < crossovers_per_pair; ++k, ++index)
					{
						Genome& new_genome = genomes[index];

						CreateCrossover(genomes[i], genomes[j], new_genome);
						Mutate(new_genome);
						if(num_memories > 2)
							for(unsigned int s = 0; s < num_index_swaps; ++s)
								SwapMemoryIndices(new_genome);

						new_genome.crossover = pcount > 1;
					}

			// do single-parent mutations
			for(unsigned int i = 0; i < crossovers_begin; ++i)
			{
				Genome& new_genome = genomes[i] = genomes[i % parents];

				Mutate(new_genome);
				if(num_memories > 2)
					for(unsigned int s = 0; s < num_index_swaps; ++s)
						SwapMemoryIndices(new_genome);

				new_genome.crossover = false;
			}

			genomes.resize(generation_size);
			for(unsigned int i = 0; i < generation_size; ++i)
				genomes[i].score = -1;

			active_genome = 0;
			++batch;
		}

		void NextBrain(unsigned int num_inputs_, unsigned int num_outputs_, unsigned int num_memories_, float& max_score)
		{
			if(num_inputs != num_inputs_ || num_outputs != num_outputs_)		// input or output array size mismatch; whole brain must be discarded
			{
				num_inputs   = num_inputs_;
				num_outputs  = num_outputs_;
				num_memories = num_memories_;

				genomes.clear();
			}
			else if(num_memories != num_memories_)								// memory array size mismatch; more memories can be added, or excess memories can be truncated
			{
				vector<float> new_brain;
				for(vector<Genome>::iterator iter = genomes.begin(); iter != genomes.end(); ++iter)
				{
					new_brain.clear();

					const float* brain = iter->brain.data();
					unsigned int stride = num_inputs + num_memories;

					for(unsigned int y = 0; y < num_outputs + num_memories_; ++y)
						for(unsigned int x = 0; x < num_inputs + num_memories_; ++x)
							if(x >= num_inputs + num_memories || y >= num_outputs + num_memories)
								new_brain.push_back(0);
							else
								new_brain.push_back(brain[y * stride + x]);

					iter->brain.assign(new_brain.begin(), new_brain.end());
				}

				num_memories = num_memories_;
			}

			unsigned int brain_inputs  = num_inputs  + num_memories;
			unsigned int brain_outputs = num_outputs + num_memories;
			brain_size = brain_inputs * brain_outputs;

			if(batch == 0 && active_genome == 0 && trial == 0)
				Debug(((stringstream&)(stringstream() << "brain size = " << brain_inputs << " x " << brain_outputs << " = " << brain_size << endl)).str());

			memory.resize(num_memories);
			memset(memory.data(), 0, memory.size() * sizeof(float));

			if(genomes.empty())
			{
				genomes.push_back(Genome(brain_size));

				active_genome = 0;
				trial = 0;
			}

			if(active_genome == genomes.size())
				CreateNextGen();

			finished = false;

			if(trial == 0)
				max_score = -1;
			else
				max_score = genomes[active_genome].score;
		}

		void Process(vector<float>& inputs, vector<float>& outputs)
		{
			if(!finished)
			{
				if(inputs.size() != num_inputs)
					Debug("Warning: input array size mismatch!\n");
				if(outputs.size() != num_outputs)
					Debug("Warning: output array size mismatch!\n");

				inputs.resize(num_inputs);
				inputs.insert(inputs.end(), memory.begin(), memory.end());

				outputs.resize(outputs.size() + memory.size());

				float* inputs_begin = inputs.data();
				float* inputs_end   = inputs_begin + inputs.size();
				float* output_ptr   = outputs.data();
				float* outputs_end  = output_ptr + num_outputs;
				float* brain_ptr    = genomes[active_genome].brain.data();
			
				for(; output_ptr != outputs_end; ++output_ptr)
				{
					float& o = *output_ptr = 0.0f;
					for(float* input_ptr = inputs_begin; input_ptr != inputs_end; ++input_ptr)
						o += *input_ptr * *(brain_ptr++);
					o = tanhf(o);
				}

				if(num_memories)
					memcpy(memory.data(), outputs.data() + num_outputs, memory.size() * sizeof(float));
				outputs.resize(num_outputs);
			}
		}

		void Finish(float score)
		{
			if(!finished)
			{
				static const unsigned int num_trials = 5;

				finished = true;

				// the system picks the worst score of however many trials
				float& g = genomes[active_genome].score;
				if(trial == 0)
					g = score;
				else
					g = min(g, score);

				++trial;
				if(trial == num_trials)
				{
					++active_genome;
					trial = 0;
				}
			}
		}
	};

	SoldierBrain::Imp* SoldierBrain::imp = new Imp();




	/*
	 * SoldierBrain methods
	 */
	void SoldierBrain::Load() { imp->Load("Files/brain.brain"); }
	void SoldierBrain::Save() { imp->Save("Files/brain.brain", imp->genomes); }

	void SoldierBrain::NextBrain(unsigned int num_inputs, unsigned int num_outputs, unsigned int num_memories, float& max_score) { imp->NextBrain(num_inputs, num_outputs, num_memories, max_score); }
	void SoldierBrain::Process(vector<float>& inputs, vector<float>& outputs) { imp->Process(inputs, outputs); }
	void SoldierBrain::Finish(float score) { imp->Finish(score); }

	bool SoldierBrain::IsFinished()     { return imp->finished; }
	string SoldierBrain::GetDebugText() { return imp->debug_text; }
}
