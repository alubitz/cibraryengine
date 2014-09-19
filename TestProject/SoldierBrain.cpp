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
			unsigned int parent_a, parent_b;
			bool clone;
			bool converted;
			float scores[NumScoringCategories];

			Genome()                  : brain(),     parent_a(0), parent_b(0), clone(false), converted(false) { }
			Genome(unsigned int size) : brain(size), parent_a(0), parent_b(0), clone(false), converted(false) { }
		};

		vector<Genome> genomes;
		unsigned int batch, active_genome, trial;
		unsigned int num_inputs, num_outputs, num_memories, brain_size;

		vector<float> memory;

		vector<bool> allowed_coeffs;

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

			unsigned int section_h = h / NumScoringCategories;
			for(unsigned int i = 0; i < NumScoringCategories; ++i)
			{
				alpha_ptr = data + (((i + 1) * section_h - 1) * w * 4) + 2;
				alpha_end = alpha_ptr + w * 4;
				for(; alpha_ptr != alpha_end; alpha_ptr += 4)
					*alpha_ptr = 128;
			}

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
								genome.parent_a = ReadUInt32(ss);
								genome.parent_b = ReadUInt32(ss);

								unsigned char cc = ReadByte(ss);
								genome.clone     = (cc & 0x01) != 0;
								genome.converted = (cc & 0x02) != 0;

								for(float *brain_ptr = genome.brain.data(), *brain_end = brain_ptr + brain_size; brain_ptr != brain_end; ++brain_ptr)
									*brain_ptr = ReadSingle(ss);

								if(ss.bad())
								{
									Debug("Brains ran out mid-brain\n");
									return;
								}
								else
									genomes.push_back(genome);
							}
						}

						// check that the range of parents in the loaded set of brains matches the current number of parents; if not, make changes!
						for(unsigned int i = 0; i < genomes.size(); ++i)
						{
							unsigned int* genome_parents[2] = { &genomes[i].parent_a, &genomes[i].parent_b };
							for(unsigned int j = 0; j < 2; ++j)
							{
								unsigned int& parent = *genome_parents[j];
								if(parent >= parents)
								{
									parent = Random3D::RandInt(parents);
									genomes[i].converted = true;
								}
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

					const Genome& g = genomes[i];
					WriteUInt32(g.parent_a, ss);
					WriteUInt32(g.parent_b, ss);
					WriteByte((g.clone ? 0x01 : 0x00) | (g.converted ? 0x02 : 0x00), ss);
					for(const float *brain_ptr = g.brain.data(), *brain_end = brain_ptr + brain_size; brain_ptr != brain_end; ++brain_ptr)
						WriteSingle(*brain_ptr, ss);
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
#if 0
				float lerp_b = Random3D::Rand(), lerp_a = 1.0f - lerp_b;
				*result_ptr = (lerp_a * *result_ptr) + (lerp_b * *b_ptr);
#else
				unsigned char lerp_b = Random3D::RandInt() % 2, lerp_a = 1 - lerp_b;
				*result_ptr = (lerp_a * *result_ptr) + (lerp_b * *b_ptr);
#endif
			}

			ShuffleMemoryIndices(result);

			EnforceAllowedCoeffs(result);
		}

		void EnforceAllowedCoeffs(Genome& genome)
		{
			float* brain = genome.brain.data();
			for(unsigned int i = 0; i < brain_size; ++i)
				if(!allowed_coeffs[i])
					brain[i] = 0.0f;
		}

		void Mutate(Genome& genome, unsigned int num_mutations)
		{
			static const unsigned int max_tries = 5;

			// randomly modify a few elements of the coefficient matrix
			for(unsigned int i = 0; i < num_mutations; ++i)
			{
				// try up to some max number of tries, or until we get a coefficient that's allowed to be nonzero
				for(unsigned int j = 0; j < max_tries; ++j)
				{
					unsigned int index = Random3D::RandInt(brain_size);

					float& coeff = genome.brain[index];
					if(allowed_coeffs[index])
					{
						if(coeff != 0.0f && Random3D::RandInt() % 2 == 0)
							coeff = 0.0f;
						else
							coeff = Random3D::Rand(-2.0f, 2.0f);

						break;
					}
					else
						coeff = 0.0f;
				}
			}
		}

		void DoPerNeuronMutations(Genome& genome)
		{
			unsigned int brain_w = num_inputs  + num_memories;
			unsigned int brain_h = num_outputs + num_memories;

			float* brain = genome.brain.data();

			vector<unsigned int> input_indices;
			for(unsigned int y = 0; y < brain_h; ++y)
			{
				float* row = brain + y * brain_w;
				for(unsigned int x = 0; x < brain_w; ++x)
					if(row[x] != 0.0f)
					{
						if(Random3D::RandInt() % 20 == 0)
							row[x] = 0.95f * row[x] + 0.05f * Random3D::Rand(-2.0f, 2.0f);

						input_indices.push_back(x);
					}

				unsigned int num_input_indices = input_indices.size();
				if(num_input_indices > 8)
				{
					if(Random3D::RandInt() % 10  != 0)
						row[input_indices[Random3D::RandInt() % num_input_indices]] = 0.0f;
					if(num_input_indices > 12)
					{
						if(Random3D::RandInt() % 20 != 0)
							row[input_indices[Random3D::RandInt() % num_input_indices]] = 0.0f;
						if(num_input_indices > 20)
							if(Random3D::RandInt() % 30 != 0)
								row[input_indices[Random3D::RandInt() % num_input_indices]] = 0.0f;
					}
				}

				if(num_input_indices < 5 && Random3D::RandInt() % 10 == 0)
					row[Random3D::RandInt() % brain_w] = Random3D::Rand(-2.0f, 2.0f);

				if(Random3D::RandInt() % 30 == 0)
					row[Random3D::RandInt() % brain_w] = Random3D::Rand(-2.0f, 2.0f);
			}

			EnforceAllowedCoeffs(genome);
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
			struct RecordTypeSelStuff
			{
				struct CategoryStats
				{
					list<float> scores;

					float avg, sd;
					float min, lq, median, uq, max;

					void ComputeStats()
					{
						unsigned int count = scores.size();
						unsigned int m1  = count / 2;
						unsigned int m2  = count % 2 == 0 ? m1 + 1 : m1;
						unsigned int lq1 = m1 / 2;
						unsigned int lq2 = lq1 % 2 == 0 ? lq1 + 1 : lq1;
						unsigned int uq1 = (m2 + count) / 2;
						unsigned int uq2 = (count - m2) % 2 == 1 ? uq1 + 1: uq1;		// x % 2 == 1 instead of (x+1) % 2 == 0

						min = max = avg = sd = median = lq = uq = 0.0f;

						scores.sort();
						unsigned int i = 0;
						for(list<float>::iterator iter = scores.begin(); iter != scores.end(); ++iter, ++i)
						{
							float f = *iter;
							
							if(i == 0)
								min = f;
							if(i == count - 1)
								max = f;

							if(i == m1  || i == m2)
								median += f;
							if(i == lq1 || i == lq2)
								lq += f;
							if(i == uq1 || i == uq2)
								uq += f;

							avg += f;
						}

						if(m1 != m2)
							median /= 2;
						if(lq1 != lq2)
							lq /= 2;
						if(uq1 != uq2)
							uq /= 2;

						avg /= count;

						for(list<float>::iterator iter = scores.begin(); iter != scores.end(); ++iter)
							sd += fabs(*iter - avg);
						sd /= count;
					}

					void Print(stringstream& ss, const string& label)
					{
						ss << "\t\t" << label << " n = " << scores.size();
						switch(scores.size())
						{
							case 0:
								ss << endl;
								break;
							case 1:
								ss << "; score = " << avg << endl;
								break;
							default:
								ss << "; avg = " << avg << "; sd = " << sd << "; min = " << min << "; lq = " << lq << "; median = " << median << "; uq = " << uq << "; max = " << max << endl;
								break;
						}						
					}
				};
			
				CategoryStats all_genomes[NumScoringCategories][5];
				CategoryStats sel_genomes[NumScoringCategories][5];

				RecordTypeSelStuff()
				{
					for(unsigned int i = 0; i < NumScoringCategories; ++i)
						for(unsigned int j = 0; j < 5; ++j)
						{
							all_genomes[i][j] = CategoryStats();
							sel_genomes[i][j] = CategoryStats();
						}
				}

				void RecordScore(unsigned int category, unsigned int sel_type, float score, bool is_selected)
				{
					if(is_selected)
					{
						sel_genomes[category][0].scores.push_back(score);
						sel_genomes[category][sel_type].scores.push_back(score);
					}
					all_genomes[category][0].scores.push_back(score);
					all_genomes[category][sel_type].scores.push_back(score);
				}

				void ComputeStats()
				{
					for(unsigned int i = 0; i < NumScoringCategories; ++i)
						for(unsigned int j = 0; j < 5; ++j)
						{
							all_genomes[i][j].ComputeStats();
							sel_genomes[i][j].ComputeStats();
						}
				}

				void Display(unsigned int batch)
				{
					static const string labels[5][2] =
					{
						{ "all genomes:   ", "sel genomes:   " },
						{ "all xcats:     ", "sel xcats:     " },
						{ "all crossovers:", "sel crossovers:" },
						{ "all mutants:   ", "sel mutants:   " },
						{ "all clones:    ", "sel clones:    " }
					};

					stringstream ss;
					ss << "generation " << batch << ":" << endl;
					for(unsigned int i = 0; i < NumScoringCategories; ++i)
					{
						ss << '\t' << "cat " << i << ":" << endl;
						for(unsigned int j = 0; j < 5; ++j)
							all_genomes[i][j].Print(ss, labels[j][0]);
						for(unsigned int j = 0; j < 5; ++j)
							sel_genomes[i][j].Print(ss, labels[j][1]);
					}
					Debug(ss.str());
				}
			} records;

			for(unsigned int i = 0; i < genomes.size(); ++i)
			{
				const Genome& g = genomes[i];

				unsigned int sel_type;
				if(g.parent_a != g.parent_b)
				{
					if(g.parent_a % NumScoringCategories != g.parent_b % NumScoringCategories)
						sel_type = 1;
					else
						sel_type = 2;
				}
				else if(g.clone)
					sel_type = 4;
				else
					sel_type = 3;

				if(g.converted)
				{
					for(unsigned int j = 0; j < NumScoringCategories; ++j)
						records.RecordScore(j,  3,        g.scores[j],  i < pcount);
				}
				else
				{
					unsigned int pa = g.parent_a, pb = g.parent_b;
					unsigned int ca = pa % NumScoringCategories, cb = pb % NumScoringCategories;

					records.RecordScore    (ca, sel_type, g.scores[ca], i < pcount && i % NumScoringCategories == ca);
					if(ca != cb)
						records.RecordScore(cb, sel_type, g.scores[cb], i < pcount && i % NumScoringCategories == cb);
				}
			}

			records.ComputeStats();

			records.Display(batch);
		}

		static const unsigned int parents             = NumScoringCategories * 15;
		static const unsigned int mutants_per_parent  = 4;
		static const unsigned int crossovers_per_pair = 2;
		static const unsigned int crossovers_begin    = parents * mutants_per_parent;
		static const unsigned int generation_size     = crossovers_begin + parents * (parents - 1) * crossovers_per_pair / 2;

		static const unsigned int num_trials          = 5;

		void CreateNextGen()
		{
			static const unsigned int crossover_mutations     = 1;
			static const unsigned int single_parent_mutations = 1;

			unsigned int pcount = min(parents, genomes.size());
			for(unsigned int i = 0; i < pcount; ++i)
			{
				unsigned int category = i % NumScoringCategories;

				unsigned int best = i;
				for(unsigned int j = i + 1; j < genomes.size(); ++j)
					if((genomes[j].parent_a == i || genomes[j].parent_b == i || genomes[j].converted) && genomes[j].scores[category] > genomes[best].scores[category])
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

						DoPerNeuronMutations(new_genome);
						Mutate(new_genome, crossover_mutations);

						new_genome.parent_a = i;
						new_genome.parent_b = j;
						new_genome.clone = new_genome.converted = false;
					}

			// do single-parent offspring; each parent spawns one perfect clone; the rest of its offspring are mutants
			for(unsigned int i = 0; i < crossovers_begin; ++i)
			{
				unsigned int parent = i % parents;
				Genome& new_genome = genomes[i] = genomes[parent];
				if(i >= parents)
				{
					DoPerNeuronMutations(new_genome);
					Mutate(new_genome, single_parent_mutations);

					new_genome.clone = false;
				}
				else
					new_genome.clone = true;

				new_genome.converted = false;

				new_genome.parent_a = new_genome.parent_b = parent;
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

				SetAllowedCoeffs();

				Debug(((stringstream&)(stringstream() << "generation size = " << generation_size << endl)).str());
				Debug(((stringstream&)(stringstream() << "num trials = " << num_trials << endl)).str());

				Debug("\n");
			}

			memory.resize(num_memories);
			memset(memory.data(), 0, memory.size() * sizeof(float));

			if(genomes.empty())
			{
				for(unsigned int i = 0; i < parents; ++i)
				{
					Genome g(brain_size);
					g.parent_a = g.parent_b = i;
					g.clone = false;
					g.converted = true;

					genomes.push_back(g);
				}

				active_genome = 0;
				trial = 0;
			}

			if(active_genome == genomes.size())
				CreateNextGen();

			finished = false;
		}

		void SetAllowedCoeffs()
		{
			unsigned int w = num_inputs + num_memories;

			allowed_coeffs.clear();
			allowed_coeffs.resize(brain_size, true);

			//for(unsigned int x = 0; x < num_inputs; ++x)
			//	for(unsigned int y = 0; y < num_outputs; ++y)
			//		allowed_coeffs[y * w + x] = false;

			unsigned int allowed_count = 0;
			for(unsigned int i = 0; i < brain_size; ++i)
				if(allowed_coeffs[i])
					++allowed_count;
			Debug(((stringstream&)(stringstream() << "allowed count = " << allowed_count << " / " << brain_size << endl)).str());
		}

		void Process(vector<float>& inputs, vector<float>& outputs)
		{
			if(!finished)
			{
				if(inputs.size() != num_inputs)
					Debug(((stringstream&)(stringstream() << "Warning: input array size mismatch! Expected " << num_inputs << " but was given " << inputs.size() << endl)).str());
				if(outputs.size() != num_outputs)
					Debug(((stringstream&)(stringstream() << "Warning: output array size mismatch! Expected " << num_outputs << " but was given " << outputs.size() << endl)).str());

				unsigned int brain_inputs  = num_inputs  + num_memories;
				unsigned int brain_outputs = num_outputs + num_memories;

				inputs.resize(num_inputs);
				inputs.insert(inputs.end(), memory.begin(), memory.end());
				
				outputs.resize(brain_outputs);

				float* inputs_begin = inputs.data();
				float* inputs_end   = inputs_begin + brain_inputs;
				float* output_ptr   = outputs.data();
				float* outputs_end  = output_ptr + brain_outputs;
				float* brain_ptr    = genomes[active_genome].brain.data();
			
				for(; output_ptr != outputs_end; ++output_ptr)
				{
					float& o = *output_ptr = 0.0f;
					for(float* input_ptr = inputs_begin; input_ptr != inputs_end; ++input_ptr, ++brain_ptr)
						o += *input_ptr * *brain_ptr;
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
				debug_text = (((stringstream&)(stringstream() << "batch " << batch << "\ngenome " << active_genome << " / " << genomes.size() << "\ntrial " << trial << " / " << num_trials)).str());

				finished = true;

				Genome& g = genomes[active_genome];

				float* gscores = g.scores;
				if(trial == 0)
					for(unsigned int i = 0; i < NumScoringCategories; ++i)
						gscores[i]  = scores[i];
				else
					for(unsigned int i = 0; i < NumScoringCategories; ++i)
						gscores[i] += scores[i];

				++trial;
				if(trial == num_trials)
				{
					for(unsigned int i = 0; i < NumScoringCategories; ++i)
						gscores[i] = 100.0f * gscores[i] / num_trials;

					UpdateScoreTextureData(gscores, active_genome);

					++active_genome;
					trial = 0;
				}
			}
		}

		void UpdateScoreTextureData(const float* scores, unsigned int genome_index)
		{
#if ENABLE_DEBUG_DATA_TEXTURE
			unsigned char* data = debug_image->byte_data;

			unsigned int batch_h = debug_image->height / NumScoringCategories - 1;
			unsigned int batch_w = (unsigned int)ceil(float(generation_size) / batch_h) + 1;

			unsigned int section_x = genome_index / batch_h;
			unsigned int section_y = genome_index % batch_h;

			unsigned int x = batch * batch_w + section_x;
			if((signed)x < debug_image->width)
			{
				for(unsigned int i = 0; i < NumScoringCategories; ++i)
				{
					unsigned int y = section_y + (batch_h + 1) * i;
					if((signed)y < debug_image->height)
					{
						unsigned int pixel_index = debug_image->width * y + x;
						unsigned char* byte_ptr = data + pixel_index * 4;

						float score_frac = max(0.0f, min(1.0f, scores[i] / 100.0f));

						float red   = score_frac * 0.75f + 0.25f;
						float green = score_frac * score_frac * score_frac;
						float blue  = green * green * green * green;

						*byte_ptr = (unsigned char)(255.0f * red);
						++byte_ptr;
						*byte_ptr = (unsigned char)(255.0f * green);
						++byte_ptr;
						*byte_ptr = (unsigned char)(255.0f * blue);
					}
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
