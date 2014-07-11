#include "StdAfx.h"
#include "SoldierBrain.h"

namespace Test
{
	using namespace CibraryEngine;
	
	/*
	 * SoldierBrain static fields
	 */
	vector<SoldierBrain::Genome> SoldierBrain::genomes = vector<SoldierBrain::Genome>();

	unsigned int  SoldierBrain::batch         = 0;
	unsigned int  SoldierBrain::active_genome = 0;
	unsigned int  SoldierBrain::trial         = 0;

	unsigned int  SoldierBrain::num_inputs    = 0;
	unsigned int  SoldierBrain::num_outputs   = 0;
	unsigned int  SoldierBrain::num_memories  = 0;
	unsigned int  SoldierBrain::brain_size    = 0;

	vector<float> SoldierBrain::memory = vector<float>();

	bool          SoldierBrain::finished   = false;
	string        SoldierBrain::debug_text = "";




	/*
	 * SoldierBrain methods
	 */
	void SoldierBrain::Load()
	{
		genomes.clear();

		ifstream file("Files/brain.brain", ios::in | ios::binary);
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

					brain_size = (1 + num_inputs + num_memories) * (num_outputs + num_memories);

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
	void SoldierBrain::Save()
	{

		ofstream file("Files/brain.brain", ios::out | ios::binary);
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

	void SoldierBrain::NextBrain(unsigned int num_inputs_, unsigned int num_outputs_, unsigned int num_memories_, float& max_lifetime)
	{
		static const unsigned int parents             = 15;
		static const unsigned int mutants_per_parent  = 4;
		static const unsigned int crossovers_per_pair = 1;
		static const unsigned int crossovers_begin    = parents + parents * mutants_per_parent;
		static const unsigned int generation_size     = crossovers_begin + parents * (parents - 1) * crossovers_per_pair / 2;

		static const unsigned int num_mutations       = 3;
		static const float        mutation_rate       = 0.5f;

		if(num_inputs != num_inputs_ || num_outputs != num_outputs_ || num_memories != num_memories_)
		{
			num_inputs   = num_inputs_;
			num_outputs  = num_outputs_;
			num_memories = num_memories_;

			genomes.clear();
		}

		memory.resize(num_memories);
		memset(memory.data(), 0, memory.size() * sizeof(float));

		brain_size = (1 + num_inputs + num_memories) * (num_outputs + num_memories);

		if(batch == 0 && active_genome == 0 && trial == 0)
		{
			Debug(((stringstream&)(stringstream() << "brain size = " << (1 + num_inputs + num_memories) << " x " << (num_outputs + num_memories) << " = " << brain_size << endl)).str());
			Debug(((stringstream&)(stringstream() << "generation size = " << generation_size << endl << endl)).str());
		}

		if(genomes.empty())
		{
			for(unsigned int i = 0; i < generation_size; ++i)
				genomes.push_back(Genome(brain_size));

			active_genome = 0;
			trial = 0;
		}

		if(active_genome == genomes.size())
		{
			float tot = 0.0f;
			for(unsigned int i = 0; i < generation_size; ++i)
			{
				float item_score = genomes[i].score;
				tot += item_score;
			}
			tot /= genomes.size();

			for(unsigned int i = 0; i < parents && i < genomes.size(); ++i)
			{
				unsigned int best = i;
				for(unsigned int j = i + 1; j < genomes.size(); ++j)
					if(genomes[j].score > genomes[best].score)
						best = j;
				swap(genomes[best], genomes[i]);
			}

			debug_text = ((stringstream&)(stringstream() << "batch[" << batch << "] avg = " << tot << "; best = " << genomes[0].score << endl)).str();
			Debug(debug_text);

			for(unsigned int i = parents; i < crossovers_begin; ++i)
			{
				genomes[i] = genomes[i % parents];
				for(unsigned int j = 0; j < num_mutations; ++j)
				{	
					float& coeff = genomes[i].brain[Random3D::RandInt(brain_size)];
					coeff += Random3D::Rand(-mutation_rate, mutation_rate);
				}
			}

			genomes.resize(max(genomes.size(), generation_size));

			unsigned int index = crossovers_begin;
			for(unsigned int i = 0; i < parents; ++i)
				for(unsigned int j = i + 1; j < parents; ++j)
					for(unsigned int k = 0; k < crossovers_per_pair; ++k, ++index)
					{
						genomes[index].brain.resize(brain_size);

						float* result_ptr = genomes[index].brain.data();
						float* results_end = result_ptr + brain_size;
						float* a_ptr = genomes[i].brain.data();
						float* b_ptr = genomes[j].brain.data();

						for(; result_ptr != results_end; ++result_ptr, ++a_ptr, ++b_ptr)
						{
							unsigned int r = Random3D::RandInt() % 3;
							if(r == 0)
								*result_ptr = (*a_ptr + *b_ptr) * 0.5f;
							else if(r == 1)
								*result_ptr = *a_ptr;
							else
								*result_ptr = *b_ptr;
						}
					}

			genomes.resize(generation_size);
			for(unsigned int i = 0; i < generation_size; ++i)
				genomes[i].score = -1;

			active_genome = 0;
			++batch;
		}

		finished = false;

		if(trial == 0)
			max_lifetime = -1;
		else
			max_lifetime = genomes[active_genome].score;
	}

	void SoldierBrain::Process(vector<float>& inputs, vector<float>& outputs)
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
				float& o = *output_ptr = *(brain_ptr++);
				for(float* input_ptr = inputs_begin; input_ptr != inputs_end; ++input_ptr)
					o += *input_ptr * *(brain_ptr++);
				o = tanhf(o);
			}

			if(num_memories)
				memcpy(memory.data(), outputs.data() + num_outputs, memory.size() * sizeof(float));
			outputs.resize(num_outputs);
		}
	}

	void SoldierBrain::Finish(float score)
	{
		finished = true;

		float& g = genomes[active_genome].score;
		if(trial == 0)
			g = score;
		else
			g = min(g, score);

		++trial;
		if(trial == 3)
		{
			++active_genome;
			trial = 0;
		}
	}

	bool SoldierBrain::IsFinished()     { return finished; }

	string SoldierBrain::GetDebugText() { return debug_text; }
}
