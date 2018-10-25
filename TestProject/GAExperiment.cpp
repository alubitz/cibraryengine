#include "StdAfx.h"
#include "GAExperiment.h"

#define NUM_ELITES						20

#define CROSSOVERS_PER_PAIR				2

#define MUTATION_COUNT					50
#define MUTATION_SCALE					0.001f

#define TRIALS_PER_SUBTEST				20

#define TARGET_PROGRAM_SIZE				40

#define NUM_BONE_INPUTS					70
#define NUM_BONE_MEM					0		// was 1
#define NUM_BONE_OUTPUTS				5		// was 3

#define FLOAT_CONSTANT_RANGE			2.0f

#define FORCE_FIRST_GEN_MODIFICATION	0


namespace Test
{
	using namespace CibraryEngine;
	/*
	 * GPOp methods
	 */
	unsigned int GPOp::Write(ostream& s)
	{
		WriteUInt32(input_weights.size(), s);
		for(unsigned int i = 0; i < input_weights.size(); ++i)
			WriteSingle(input_weights[i], s);

		WriteUInt32(mem_weights.size(), s);
		for(unsigned int i = 0; i < mem_weights.size(); ++i)
			WriteSingle(mem_weights[i], s);

		WriteUInt32(scratch_weights.size(), s);
		for(unsigned int i = 0; i < scratch_weights.size(); ++i)
			WriteSingle(scratch_weights[i], s);

		WriteUInt32(parent_weights.size(), s);
		for(unsigned int i = 0; i < parent_weights.size(); ++i)
			WriteSingle(parent_weights[i], s);

		WriteUInt32(child_weights.size(), s);
		for(unsigned int i = 0; i < child_weights.size(); ++i)
			WriteSingle(child_weights[i], s);

		return 0;
	}

	unsigned int GPOp::Read(istream& s)
	{
		input_weights.resize(ReadUInt32(s));
		for(unsigned int i = 0; i < input_weights.size(); ++i)
			input_weights[i] = ReadSingle(s);

		mem_weights.resize(ReadUInt32(s));
		for(unsigned int i = 0; i < mem_weights.size(); ++i)
			mem_weights[i] = ReadSingle(s);

		scratch_weights.resize(ReadUInt32(s));
		for(unsigned int i = 0; i < scratch_weights.size(); ++i)
			scratch_weights[i] = ReadSingle(s);

		parent_weights.resize(ReadUInt32(s));
		for(unsigned int i = 0; i < parent_weights.size(); ++i)
			parent_weights[i] = ReadSingle(s);

		child_weights.resize(ReadUInt32(s));
		for(unsigned int i = 0; i < child_weights.size(); ++i)
			child_weights[i] = ReadSingle(s);

		return s.bad() ? 1 : 0;
	}


	/*
	 * GACandidate methods
	 */
	GACandidate::GACandidate(unsigned int id) : id(id), p1(0), p2(0), scratch_ops(), output_ops(), mem_ops(), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		memset(mutations, 0, sizeof(mutations));
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& other) : id(id), p1(other.id), p2(other.id), scratch_ops(), output_ops(), mem_ops(), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		memcpy(mutations, other.mutations, sizeof(mutations));		// or should it memset(0)?
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& p1, const GACandidate& p2) : id(id), p1(p1.id), p2(p2.id), scratch_ops(), output_ops(), mem_ops(), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		scratch_ops = Crossover(p1.scratch_ops, p2.scratch_ops);
		output_ops = Crossover(p1.output_ops, p2.output_ops);
		mem_ops = Crossover(p1.mem_ops, p2.mem_ops);

		memset(mutations, 0, sizeof(mutations));

		Randomize(MUTATION_COUNT, MUTATION_SCALE);

#if FORCE_FIRST_GEN_MODIFICATION
		if(p1.p1 == 0 && p1.p2 == 0 || p2.p1 == 0 && p2.p2 == 0)
		{
		}
#endif
	}

	GACandidate::~GACandidate() { }

	void GACandidate::Randomize(unsigned int count, float scale)
	{
		float minx = -FLOAT_CONSTANT_RANGE;
		float maxx =  FLOAT_CONSTANT_RANGE;
		float rscale = scale * (maxx - minx);

		bool ok;
		do
		{
			ok = false;

			unsigned int tot = scratch_ops.size() + output_ops.size() + mem_ops.size();
			unsigned int r = Random3D::BigRand(tot);

			GPOp* target = nullptr;

			if(r < scratch_ops.size())
				target = &scratch_ops[r];
			else
			{
				r -= scratch_ops.size();
				if(r < output_ops.size())
					target = &output_ops[r];
				else
				{
					r -= output_ops.size();
					target = &mem_ops[r];
				}
			}

			tot = target->input_weights.size() + target->mem_weights.size() + target->scratch_weights.size() + target->parent_weights.size() + target->child_weights.size();
			r = Random3D::BigRand(tot);

			float* fptr = nullptr;
			if(r < target->input_weights.size())
				fptr = &target->input_weights[r];
			else
			{
				r -= target->input_weights.size();
				if(r < target->mem_weights.size())
					fptr = &target->mem_weights[r];
				else
				{
					r -= target->mem_weights.size();
					if(r < target->scratch_weights.size())
						fptr = &target->scratch_weights[r];
					else
					{
						r -= target->scratch_weights.size();
						if(r < target->parent_weights.size())
							fptr = &target->parent_weights[r];
						else
						{
							r -= target->parent_weights.size();
							fptr = &target->child_weights[r];
						}
					}
				}
			}

			*fptr = max(minx, min(maxx, *fptr + Random3D::Rand(-rscale, rscale)));
			++mutations[0];
			ok = true;

		} while(!ok || Random3D::RandInt() % count != 0);
	}

	vector<GPOp> GACandidate::Crossover(const vector<GPOp>& a, const vector<GPOp>& b)
	{
		vector<GPOp> results(max(a.size(), b.size()));

		vector<float> dummy;

		for(unsigned int i = 0; i < results.size(); ++i)
		{
			if(i < a.size() && i < b.size())
			{
				results[i].input_weights   = Crossover(a[i].input_weights  , b[i].input_weights  );
				results[i].mem_weights     = Crossover(a[i].mem_weights    , b[i].mem_weights    );
				results[i].scratch_weights = Crossover(a[i].scratch_weights, b[i].scratch_weights);
				results[i].parent_weights  = Crossover(a[i].parent_weights , b[i].parent_weights );
				results[i].child_weights   = Crossover(a[i].child_weights  , b[i].child_weights  );
			}
			else if(i < a.size())
			{
				results[i].input_weights   = Crossover(a[i].input_weights  , dummy);
				results[i].mem_weights     = Crossover(a[i].mem_weights    , dummy);
				results[i].scratch_weights = Crossover(a[i].scratch_weights, dummy);
				results[i].parent_weights  = Crossover(a[i].parent_weights , dummy);
				results[i].child_weights   = Crossover(a[i].child_weights  , dummy);
			}
			else
			{
				results[i].input_weights   = Crossover(dummy, b[i].input_weights  );
				results[i].mem_weights     = Crossover(dummy, b[i].mem_weights    );
				results[i].scratch_weights = Crossover(dummy, b[i].scratch_weights);
				results[i].parent_weights  = Crossover(dummy, b[i].parent_weights );
				results[i].child_weights   = Crossover(dummy, b[i].child_weights  );
			}
		}

		return results;
	}

	vector<float> GACandidate::Crossover(const vector<float>& a, const vector<float>& b)
	{
		vector<float> results(max(a.size(), b.size()));

		for(unsigned int i = 0; i < results.size(); ++i)
			results[i] = Crossover(i < a.size() ? a[i] : 0.0f, i < b.size() ? b[i] : 0.0f);

		return results;
	}

	float GACandidate::Crossover(float a, float b) { return a + Random3D::Rand() * (b - a); }

	void GACandidate::Resize()
	{
		scratch_ops.resize(TARGET_PROGRAM_SIZE);
		output_ops.resize(NUM_BONE_OUTPUTS);
		mem_ops.resize(NUM_BONE_MEM);

		for(unsigned int i = 0; i < scratch_ops.size(); ++i)
		{
			GPOp& op = scratch_ops[i];
			op.input_weights.resize(NUM_BONE_INPUTS);
			op.mem_weights.resize(NUM_BONE_MEM);
			op.scratch_weights.resize(i);
			op.parent_weights.resize(i);
			op.child_weights.resize(i);
		}

		for(unsigned int i = 0; i < mem_ops.size(); ++i)
		{
			GPOp& op = mem_ops[i];
			op.input_weights.resize(NUM_BONE_INPUTS);		// can assign directly from input to mem
			op.mem_weights.resize(NUM_BONE_MEM);			// can assign directly from mem to mem
			op.scratch_weights.resize(scratch_ops.size());
			op.parent_weights.resize(scratch_ops.size());
			op.child_weights.resize(scratch_ops.size());
		}

		for(unsigned int i = 0; i < output_ops.size(); ++i)
		{
			GPOp& op = output_ops[i];
			op.input_weights.resize(0);						// can NOT assign directly from input to output
			op.mem_weights.resize(0);						// can NOT assign directly from mem to output
			op.scratch_weights.resize(scratch_ops.size());
			op.parent_weights.resize(scratch_ops.size());
			op.child_weights.resize(scratch_ops.size());
		}
	}

	string GACandidate::GetText() const
	{
		stringstream ss;
		ss << '(' << id << ", p " << p1 << ", " << p2 << ") score = " << score / TRIALS_PER_SUBTEST << " (";
		for(unsigned int i = 0; i < score_parts.size(); ++i)
		{
			if(i != 0)
				ss << ", ";
			ss << score_parts[i] / TRIALS_PER_SUBTEST;
		}
		ss << ')';

		return ss.str();
	}

	unsigned int GACandidate::Write(ostream& s)
	{
		stringstream ss;

		WriteUInt32(scratch_ops.size(), ss);
		for(unsigned int i = 0; i < scratch_ops.size(); ++i)
			scratch_ops[i].Write(ss);

		WriteUInt32(output_ops.size(), ss);
		for(unsigned int i = 0; i < output_ops.size(); ++i)
			output_ops[i].Write(ss);

		WriteUInt32(mem_ops.size(), ss);
		for(unsigned int i = 0; i < mem_ops.size(); ++i)
			mem_ops[i].Write(ss);
		
		BinaryChunk chunk("BONENNS_");
		chunk.data = ss.str();
		chunk.Write(s);

		return 0;
	}

	unsigned int GACandidate::Read(istream& s, GACandidate*& result, unsigned int id)
	{
		result = nullptr;

		BinaryChunk chunk;
		chunk.Read(s);

		if(chunk.GetName() != "BONENNS_")
			return 1;

		istringstream ss(chunk.data);

		GACandidate* candidate = new GACandidate(id);
		
		candidate->scratch_ops.resize(ReadUInt32(ss));
		for(unsigned int i = 0; i < candidate->scratch_ops.size(); ++i)
			if(unsigned int error = candidate->scratch_ops[i].Read(ss))
			{
				delete candidate;
				return 1;
			}
		
		candidate->output_ops.resize(ReadUInt32(ss));
		for(unsigned int i = 0; i < candidate->output_ops.size(); ++i)
			if(unsigned int error = candidate->output_ops[i].Read(ss))
			{
				delete candidate;
				return 1;
			}

		candidate->mem_ops.resize(ReadUInt32(ss));
		for(unsigned int i = 0; i < candidate->mem_ops.size(); ++i)
			if(unsigned int error = candidate->mem_ops[i].Read(ss))
			{
				delete candidate;
				return 1;
			}

		if(s.bad())
		{
			delete candidate;
			return 2;
		}
		else
		{
			result = candidate;
			return 0;
		}
	}




	/*
	 * GAExperiment methods
	 */
	GAExperiment::GAExperiment(const string& filename) : mutex(), batch(0), next_id(1), subtests(), elites(), candidates(), tokens_not_started(), tokens_busy()
	{
		// load the saved best brain if possible
		ifstream file(filename, ios::in | ios::binary);
		if(!!file)
		{
			unsigned int num_genomes = ReadUInt32(file);
			for(unsigned int i = 0; i < num_genomes; ++i)
			{
				GACandidate* loadme = nullptr;
				if(unsigned int error = GACandidate::Read(file, loadme, next_id++))
				{
					Debug(((stringstream&)(stringstream() << "Error " << error << " loading GACandidate" << endl)).str());
					break;
				}
				else
				{
					loadme->Resize();
					candidates.push_back(loadme);
				}
			}

			if(!candidates.empty())
			{
				Debug(((stringstream&)(stringstream() << "Successfully loaded " << candidates.size() << " brains from genepool" << endl)).str());
				DebugGenerationStats();

				// make sure sizes match
			}

			file.close();
		}

		if(candidates.empty())
			MakeFirstGeneration();

		GenerateSubtestList();

		GenerateTrialTokens();

		Debug(((stringstream&)(stringstream() << "first generation will contain " << candidates.size() << " candidates (" << tokens_not_started.size() << " tokens)" << endl)).str());
	}

	GAExperiment::~GAExperiment()
	{
		subtests.clear();
		elites.clear();
		tokens_not_started.clear();
		tokens_busy.clear();

		for(unsigned int i = 0; i < candidates.size(); ++i)
			delete candidates[i];
		candidates.clear();
	}

	void GAExperiment::GenerateSubtestList()
	{
		static const unsigned int num_subtests = 4;
		static const float scalar = 0.05f;

		vector<float> floats;
		for(unsigned int j = 0; j < num_subtests / 2; ++j)
		{
			float x = scalar * float(j) / (float(num_subtests / 2) - 1);
			floats.push_back(x);
			floats.push_back(-x);
		}
		while(floats.size() < num_subtests)
			floats.push_back(0.0f);					// i.e. if the number of subtests is not a multiple of 2

		vector<float[6]> axes(num_subtests);
		for(unsigned int i = 0; i < 6; ++i)
		{
			vector<float> axis_floats = floats;
			for(unsigned int j = 0; j < num_subtests; ++j)
			{
				swap(axis_floats[axis_floats.size() - 1], axis_floats[Random3D::RandInt(axis_floats.size())]);
				axes[j][i] = axis_floats[axis_floats.size() - 1];
				axis_floats.pop_back();
			}
		}

		subtests.clear();
		for(unsigned int i = 0; i < num_subtests; ++i)
		{
			const Vec3* axes_vec = (Vec3*)&axes[i];
			subtests.push_back(GASubtest(i, axes_vec[0], axes_vec[1]));
			//subtests.push_back(GASubtest(i, Vec3(), Vec3()));
		}
	}

	void GAExperiment::GenerateTrialTokens()
	{
		assert(tokens_not_started.empty());
		assert(tokens_busy.empty());

		for(unsigned int left = 0, right = candidates.size() - 1; left < right; left++, right--)
			swap(candidates[left], candidates[right]);

		for(unsigned int i = 0; i < candidates.size(); ++i)
		{
			GACandidate* candidate = candidates[i];

			assert(candidate->tokens_not_finished.empty());
			assert(candidate->tokens_busy.empty());

			candidate->score = 0.0f;
			candidate->score_parts.clear();
			candidate->time_spent = 0;
			candidate->aborting = false;

			for(unsigned int j = 0; j < subtests.size(); ++j)
				for(unsigned int k = 0; k < TRIALS_PER_SUBTEST; ++k)
				{
					GATrialToken token;
					token.candidate = candidate;
					token.subtest = subtests[j];
					token.trial = k;

					candidate->tokens_not_finished.insert(token);
					tokens_not_started.push_back(token);
				}
		}
	}

	void GAExperiment::MakeFirstGeneration()
	{
		unsigned int count = NUM_ELITES * NUM_ELITES;
		for(unsigned int i = 0; i < count; ++i)
		{
			GACandidate* candidate = new GACandidate(next_id++);

			candidate->Resize();
			
			if(i != 0)
				candidate->Randomize(MUTATION_COUNT * 2, MUTATION_SCALE * 2);

			candidates.push_back(candidate);
		}
	}

	void GAExperiment::MakeNextGeneration()
	{
		++batch;

		// delete old candidates that didn't make it into the elites
		set<GACandidate*> dead;
		for(unsigned int i = 0; i < candidates.size(); ++i)
			dead.insert(candidates[i]);
		for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
			dead.erase(*iter);

		for(set<GACandidate*>::iterator iter = dead.begin(); iter != dead.end(); ++iter)
			delete *iter;
		dead.clear();
		candidates.clear();

		// elites of the last generation are included in the next one
		for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
			candidates.push_back(*iter);

		// create 2 crossovers for each pair of elites
		for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
			for(list<GACandidate*>::iterator jter = iter; jter != elites.end(); ++jter)
				if(iter != jter)
				{
					for(unsigned int n = 0; n < CROSSOVERS_PER_PAIR; ++n)
					{
						GACandidate* candidate = new GACandidate(next_id++, **iter, **jter);
						candidates.push_back(candidate);
					}
				}

		elites.clear();
	}

	GATrialToken GAExperiment::GetNextTrial()
	{
		unique_lock<std::mutex> lock(mutex);

		if(tokens_not_started.empty())
			return GATrialToken();

		GATrialToken result = *tokens_not_started.rbegin();
		tokens_not_started.pop_back();

		result.candidate->tokens_busy.insert(result);
		tokens_busy.insert(result);

		//Debug(((stringstream&)(stringstream() << "returning trial: candidate = " << result.candidate->id << "; trial = " << result.trial << endl)).str()); 

		return result;
	}

	void GAExperiment::TrialFinished(GATrialToken token, float score, const vector<float>& score_parts, unsigned int time_spent)
	{
		unique_lock<std::mutex> lock(mutex);

		GACandidate& candidate = *token.candidate;
		candidate.score += score;

		if(candidate.score_parts.empty())
			candidate.score_parts.resize(score_parts.size());
		else if(score_parts.empty())
			assert(candidate.score_parts.size() == score_parts.size() || score_parts.empty());

#if FORCE_FIRST_GEN_MODIFICATION
		if(candidate.p1 == 0 && candidate.p2 == 0)
		{
			score += 10000;
			candidate.score += 10000;
			if(!candidate.score_parts.empty())
				candidate.score_parts[0] += 10000;
		}
#endif

		for(unsigned int i = 0; i < score_parts.size(); ++i)
			candidate.score_parts[i] += score_parts[i];

		candidate.time_spent += time_spent;

		candidate.tokens_busy.erase(token);
		candidate.tokens_not_finished.erase(token);

		tokens_busy.erase(token);

		if(!candidate.aborting)
		{
			if(elites.size() >= NUM_ELITES && candidate.score >= (*elites.rbegin())->score)
				candidate.aborting = true;
		}

		if(candidate.tokens_not_finished.empty())
		{
			stringstream ss;
			ss << candidate.GetText();

			list<GACandidate*>::iterator insert_where;
			for(insert_where = elites.begin(); insert_where != elites.end(); ++insert_where)
				if(!candidate.aborting && (*insert_where)->score > candidate.score)
					break;

			elites.insert(insert_where, token.candidate);

			if(*elites.begin() == token.candidate)
				ss << "; new best!";
			else if(*elites.rbegin() == token.candidate && elites.size() > NUM_ELITES)
				ss << "; fail (" << candidate.time_spent << " / x)";		// log early-abort savings

			ss << endl;
			Debug(ss.str());

			if(elites.size() > NUM_ELITES && *elites.rbegin() != token.candidate)
				elites.pop_back();

			stringstream ss2;
			ss2 << "batch " << batch << endl;
			for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
			{
				const GACandidate& c = **iter;
				if(*iter == token.candidate)
				{
					if(elites.size() > NUM_ELITES && &c == *elites.rbegin())
						ss2 << endl;
					ss2 << ">   ";
				}
				else
					ss2 << "    ";
				ss2 << c.GetText() << endl;
			}
			debug_text = ss2.str();

			if(elites.size() > NUM_ELITES)
				elites.pop_back();
		}

		if(tokens_busy.empty() & tokens_not_started.empty())
		{
			time_t raw_time;
			time(&raw_time);
			tm now = *localtime(&raw_time);
			string filename = ((stringstream&)(stringstream() << "Files/Brains/genepool-" << now.tm_year + 1900 << "-" << now.tm_mon + 1 << "-" << now.tm_mday << "-" << now.tm_hour << "-" << now.tm_min << "-" << now.tm_sec)).str();

			SaveElites(filename, true);
			SaveElites("Files/Brains/genepool");

			DebugGenerationStats();
			
			MakeNextGeneration();
			GenerateTrialTokens();

			Debug(((stringstream&)(stringstream() << "next generation will contain " << candidates.size() << " candidates (" << tokens_not_started.size() << " tokens)" << endl)).str());
		}
	}

	float GAExperiment::GetEarlyFailThreshold()
	{
		unique_lock<std::mutex> lock(mutex);

		if(elites.size() < NUM_ELITES)
			return -1;
		else
			return (*elites.rbegin())->score;
	}

	void GAExperiment::SaveElites(const string& filename, bool verbose)
	{
		ofstream file(filename, ios::out | ios::binary);
		if(!file)
			Debug("Failed to save brain!\n");
		else
		{
			WriteUInt32(elites.size(), file);

			for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
				(**iter).Write(file);

			file.close();

			if(verbose)
			{
				stringstream ss;
				ss << "Genepool saved to \"" << filename << "\"" << endl;
				ss << "batch " << batch << " elites:" << endl;

				for(list<GACandidate*>::iterator iter = elites.begin(); iter != elites.end(); ++iter)
				{
					const GACandidate& c = **iter;
					ss << '\t' << c.GetText() << "; mutations: [";
					for(unsigned int i = 0; i < sizeof(c.mutations) / sizeof(unsigned short); ++i)
					{
						if(i != 0)
							ss << ", ";
						ss << c.mutations[i];
					}
					ss << "]" << endl;
				}

				Debug(ss.str());
			}
		}
	}

	void GAExperiment::DebugGenerationStats() const
	{
		vector<GACandidate*> analyze_these;
		if(!elites.empty())
			analyze_these.assign(elites.begin(), elites.end());
		else if(!candidates.empty())
			analyze_these.assign(candidates.begin(), candidates.end());
		else
			return;

		//stringstream ss;
		//
		//unsigned int count = GACandidate::NumIndexedParams();
		//for(unsigned int i = 0; i < count; ++i)
		//{
		//	float tot = 0.0f;
		//	float min, max;
		//	for(unsigned int j = 0; j < analyze_these.size(); ++j)
		//	{
		//		float value = ((float*)&analyze_these[j]->params_prefix)[i + 1];
		//		tot += value;
		//		if(j == 0 || value < min)
		//			min = value;
		//		if(j == 0 || value > max)
		//			max = value;
		//	}
		//
		//	float avg = tot / analyze_these.size();
		//	ss << "\tcomponent " << i << ": MIN = " << GACandidate::min_values.GetIndexedParam(i) << ", min = " << min << ", avg = " << avg << ", max = " << max << ", MAX = " << GACandidate::max_values.GetIndexedParam(i) << endl;
		//}
		//
		//Debug(ss.str());
	}

	string GAExperiment::GetDebugText()
	{
		return debug_text;
	}
}
