#include "StdAfx.h"
#include "SoldierBrain.h"

#define ENABLE_DEBUG_DATA_TEXTURE 1

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
			float scores[NumScoringCategories];
			bool crossover;

			Genome() : brain(), crossover(false) { }
			Genome(unsigned int size) : brain(size), crossover(false) { }
		};

		vector<Genome> genomes;
		unsigned int batch, active_genome, trial;
		unsigned int num_inputs, num_outputs, num_memories, brain_size;

		vector<float> memory;

		bool finished;

		string debug_text;
		Texture2D* debug_image;

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
			debug_text(""),
			debug_image(NULL)
		{
#if ENABLE_DEBUG_DATA_TEXTURE
			unsigned int w = 1024;
			unsigned int h = 512;
			unsigned char* data = new unsigned char[w * h * 4];
			memset(data, 0, w * h * 4 * sizeof(unsigned char));

			unsigned char* alpha_ptr = data + 3;
			unsigned char* alpha_end = alpha_ptr + w * h * 4;
			for(; alpha_ptr != alpha_end; alpha_ptr += 4)
				*alpha_ptr = 255;

			debug_image = new Texture2D(w, h, data, false, true);
#endif
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
			memcpy(result.brain.data(), parent_a.brain.data(), brain_size * sizeof(float));

			ShuffleMemoryIndices(result);

			float* result_ptr  = result.brain.data();
			float* results_end = result_ptr + brain_size;
			const float* b_ptr = parent_b.brain.data();

			for(; result_ptr != results_end; ++result_ptr, ++b_ptr)
			{
#if 1
				float lerp_b = Random3D::Rand(), lerp_a = 1.0f - lerp_b;
				*result_ptr = (lerp_a * *result_ptr) + (lerp_b * *b_ptr);
#else
				unsigned char lerp_b = Random3D::RandInt() % 2, lerp_a = 1 - lerp_b;
				*result_ptr = (lerp_a * *result_ptr) + (lerp_b * *b_ptr);
#endif
			}

			ShuffleMemoryIndices(result);
		}

		void Mutate(Genome& genome, unsigned int num_mutations)
		{
			unsigned int w = num_inputs + num_memories;

			// randomly modify a few elements of the coefficient matrix
			for(unsigned int i = 0; i < num_mutations; ++i)
			{	
				unsigned int index = Random3D::RandInt(brain_size);
				unsigned int x = index % w;
				unsigned int y = index / w;

				float& coeff = genome.brain[index];

				if(x >= num_inputs || y >= num_outputs)
				{
					if(Random3D::Rand() * 0.2f > fabs(coeff))
						coeff = 0.0f;
					else
						coeff = Random3D::Rand(-2.0f, 2.0f);
				}
			}
		}

		void SwapMemoryIndices(float* brain, unsigned int a, unsigned int b, unsigned int row_size, unsigned int col_size)
		{
			unsigned int ar = (a + num_outputs) * row_size, br = (b + num_outputs) * row_size;
			for(unsigned int i = 0 ; i < row_size; ++i)
				swap(brain[ar + i], brain[br + i]);
			for(unsigned int i = 0; i < col_size; ++i)
				swap(brain[i * row_size + a + num_inputs], brain[i * row_size + b + num_inputs]);
		}

		void ShuffleMemoryIndices(Genome& genome)
		{
			if(num_memories >= 2)
			{
				float* brain = genome.brain.data();

				unsigned int row_size = num_inputs  + num_memories;
				unsigned int col_size = num_outputs + num_memories;

				unsigned int n_minus_one = num_memories - 1;

				for(unsigned int a = n_minus_one; a > 0; --a)
				{
					unsigned int b = Random3D::RandInt(a + 1);
					if(a != b)
						SwapMemoryIndices(brain, a, b, row_size, col_size);
				}
			}
		}

		void DoScoreStatistics(unsigned int pcount)
		{
			unsigned int crossover_count = 0;

			float tot[NumScoringCategories], ptot[NumScoringCategories];
			for(unsigned int i = 0; i < NumScoringCategories; ++i)
				tot[i] = ptot[i] = 0.0f;

			for(unsigned int i = 0; i < genomes.size(); ++i)
			{
				const float* item_scores = genomes[i].scores;
				for(unsigned int j = 0; j < NumScoringCategories; ++j)
					tot[j] += item_scores[j];
			}

			for(unsigned int i = 0; i < pcount; ++i)
			{
				for(unsigned int j = 0; j < NumScoringCategories; ++j)
					ptot[j] += genomes[i].scores[j];
				if(genomes[i].crossover)
					++crossover_count;
			}

			for(unsigned int i = 0; i < NumScoringCategories; ++i)
			{
				tot[i]  /= float(genomes.size());
				ptot[i] /= float(pcount);
			}

			stringstream ss;
			ss << "batch[" << batch << "]; crossovers = " << crossover_count << endl;
			ss << '\t' << "top " << pcount << " avg = { ";
			for(unsigned int i = 0; i < NumScoringCategories; ++i)
				if(i == 0)
					ss << ptot[i];
				else
					ss << ", " << ptot[i];
			ss << " }" << endl;
			ss << '\t' << "full avg = { ";
			for(unsigned int i = 0; i < NumScoringCategories; ++i)
				if(i == 0)
					ss << tot[i];
				else
					ss << ", " << tot[i];
			ss << " }" << endl;

			Debug(ss.str());
		}

		void CreateNextGen()
		{
			static const unsigned int crossover_mutations     = 2;
			static const unsigned int single_parent_mutations = 10;

			static const unsigned int parent_categories[] =
			{
				5, 5, 5, 5, 5,
				0, 1, 2, 3, 4,
				0, 1, 2, 3, 4,
				0, 1, 2, 3, 4
			};

			static const unsigned int parents             = sizeof(parent_categories) / sizeof(unsigned int);
			static const unsigned int mutants_per_parent  = 19;
			static const unsigned int crossovers_per_pair = 2;
			static const unsigned int crossovers_begin    = parents * mutants_per_parent;
			static const unsigned int generation_size     = crossovers_begin + parents * (parents - 1) * crossovers_per_pair / 2;

			if(batch == 0)
				Debug(((stringstream&)(stringstream() << "generation size = " << generation_size << endl << endl)).str());

			unsigned int pcount = min(parents, genomes.size());
			for(unsigned int i = 0; i < pcount; ++i)
			{
				unsigned int category = parent_categories[i];

				unsigned int best = i;
				for(unsigned int j = i + 1; j < genomes.size(); ++j)
					if(genomes[j].scores[category] > genomes[best].scores[category])
						best = j;
				swap(genomes[best], genomes[i]);
			}

			DoScoreStatistics(pcount);

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
						Mutate(new_genome, crossover_mutations);

						new_genome.crossover = pcount > 1;
					}

			// do single-parent offspring; each parent spawns one perfect clone; the rest of its offspring are mutants
			for(unsigned int i = 0; i < crossovers_begin; ++i)
			{
				Genome& new_genome = genomes[i] = genomes[i % parents];
				if(i >= parents)
					Mutate(new_genome, single_parent_mutations);

				new_genome.crossover = false;
			}

			genomes.resize(generation_size);

			active_genome = 0;
			++batch;
		}

		void NextBrain(unsigned int num_inputs_, unsigned int num_outputs_, unsigned int num_memories_)
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
				Debug(((stringstream&)(stringstream() << "resizing brain matrix to change from " << num_memories << " memories (old) to " << num_memories_ << " memories (new)" << endl)).str());

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
			{
				Debug(((stringstream&)(stringstream() << "inputs = " << num_inputs << "; outputs = " << num_outputs << "; memories = " << num_memories << endl)).str());
				Debug(((stringstream&)(stringstream() << "brain size = " << brain_inputs << " x " << brain_outputs << " = " << brain_size << endl)).str());
			}

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

		void Finish(const float* scores)
		{
			if(!finished)
			{
				static const unsigned int num_trials = 5;

				debug_text = (((stringstream&)(stringstream() << "batch " << batch << "\ngenome " << active_genome << " / " << genomes.size() << "\ntrial " << trial << " / " << num_trials)).str());

				finished = true;

				float* g = genomes[active_genome].scores;
				if(trial == 0)
					for(unsigned int i = 0; i < NumScoringCategories; ++i)
						g[i]  = scores[i];
				else
					for(unsigned int i = 0; i < NumScoringCategories; ++i)
						g[i] *= scores[i];

				++trial;
				if(trial == num_trials)
				{
					++active_genome;
					trial = 0;

					for(unsigned int i = 0; i < NumScoringCategories; ++i)
						g[i] = 100.0f * powf(g[i], 1.0f / num_trials);

					UpdateScoreTextureData(g, genomes[active_genome].crossover);
				}
			}
		}

		void UpdateScoreTextureData(const float* scores, bool crossover)
		{
#if ENABLE_DEBUG_DATA_TEXTURE
			unsigned char* data = debug_image->byte_data;

			unsigned int x = batch;
			if((signed)x < debug_image->width)
			{
				unsigned int h = debug_image->height / SoldierBrain::NumScoringCategories;

				for(unsigned int i = 0; i < SoldierBrain::NumScoringCategories; ++i)
				{
					float score = max(0.0f, min(1.0f, scores[i] / 100.0f));

					unsigned int y = (unsigned int)(score * h) + i * h;
					if((signed)y < debug_image->height)
					{
						unsigned int pixel_index = debug_image->width * y + x;
						unsigned char* byte_ptr = data + pixel_index * 4;

						*byte_ptr = (unsigned char)min(255, (int)*byte_ptr + 32);
						++byte_ptr;
						*byte_ptr = (unsigned char)min(255, (int)*byte_ptr + 8);
						++byte_ptr;
						*byte_ptr = (unsigned char)min(255, (int)*byte_ptr + 1);
					}
					else
						DEBUG();
				}
			}
#endif
		}
	};

	SoldierBrain::Imp* SoldierBrain::imp = new Imp();




	/*
	 * SoldierBrain methods
	 */
	void SoldierBrain::Load() { imp->Load("Files/brain.brain"); }
	void SoldierBrain::Save() { imp->Save("Files/brain.brain", imp->genomes); }

	void SoldierBrain::NextBrain(unsigned int num_inputs, unsigned int num_outputs, unsigned int num_memories) { imp->NextBrain(num_inputs, num_outputs, num_memories); }
	void SoldierBrain::Process(vector<float>& inputs, vector<float>& outputs)	{ imp->Process(inputs, outputs); }
	void SoldierBrain::Finish(const float* scores)								{ imp->Finish(scores); }

	bool SoldierBrain::IsFinished()												{ return imp->finished; }
	string SoldierBrain::GetDebugText()											{ return imp->debug_text; }

	Texture2D* SoldierBrain::GetDebugImage()									{ return imp->debug_image; }
}
