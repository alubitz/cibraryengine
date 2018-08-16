#include "StdAfx.h"
#include "GAExperiment.h"

#define NUM_ELITES						10

#define CROSSOVERS_PER_PAIR				2

#define MUTATION_COUNT					10
#define MUTATION_SCALE					0.5f

#define TRIALS_PER_SUBTEST				20

#define FORCE_FIRST_GEN_MODIFICATION	0


namespace Test
{
	using namespace CibraryEngine;

	/*
	 * GACandidate methods
	 */
	const GACandidate GACandidate::min_values = GACandidate::InitMin();
	const GACandidate GACandidate::max_values = GACandidate::InitMax();
	GACandidate::GACandidate(unsigned int id) : id(id), p1(0), p2(0), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		memset(&params_prefix, 0, (unsigned char*)&params_suffix - (unsigned char*)&params_prefix);
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& other) : id(id), p1(other.id), p2(other.id), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		memcpy(&params_prefix, &other.params_prefix, (unsigned char*)&params_suffix - (unsigned char*)&params_prefix);
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& p1, const GACandidate& p2) : id(id), p1(p1.id), p2(p2.id), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		const float* aptr = &p1.params_prefix;
		const float* bptr = &p2.params_prefix;
		for(float *optr = &params_prefix, *oend = &params_suffix; optr != oend; ++optr, ++aptr, ++bptr)
			*optr = Crossover(*aptr, *bptr);

		Randomize(MUTATION_COUNT, MUTATION_SCALE);
	}

	GACandidate::~GACandidate() { }

	GACandidate GACandidate::InitMin()
	{
		GACandidate result(0);
		result.bone_t_weights[0] = 1.0f;		// carapace weight is always 1
		for(unsigned int i = 1; i < 12; ++i)
			result.bone_t_weights[i] = 0.0f;
		for(unsigned int i = 0; i < 9; ++i)
			result.foot_t_absorbed[i] = 0.0f;
		for(unsigned int i = 0; i < 9; ++i)
			result.foot_t_matching[i] = 0.0f;
		for(unsigned int i = 0; i < 6; ++i)
			result.leg_fixed_xfrac[i] = 0.0f;
		return result;
	}

	GACandidate GACandidate::InitMax()
	{
		GACandidate result(0);
		
		result.bone_t_weights[0] = 1.0f;		// carapace weight is always 1
		
		result.bone_t_weights[1] = result.bone_t_weights[2] = result.bone_t_weights[3] = result.bone_t_weights[6] = result.bone_t_weights[9] = 2.0f;
		
		//for(unsigned int i = 1; i < 12; ++i)
		//	result.bone_t_weights[i] = 2.0f;
		for(unsigned int i = 0; i < 9; ++i)
			result.foot_t_absorbed[i] = 2.0f;
		for(unsigned int i = 0; i < 9; ++i)
			result.foot_t_matching[i] = 1.0f;
		//for(unsigned int i = 0; i < 6; ++i)
		//	result.leg_fixed_xfrac[i] = 1.0f;
		
		return result;
	}

	unsigned int GACandidate::NumIndexedParams() { return (&min_values.params_suffix - &min_values.params_prefix) - 1; }
	float& GACandidate::GetIndexedParam(unsigned int n)				{ return (&params_prefix)[n + 1]; }
	const float& GACandidate::GetIndexedParam(unsigned int n) const	{ return (&params_prefix)[n + 1]; }

	void GACandidate::Randomize(unsigned int count, float scale)
	{
		bool ok;
		do
		{
			unsigned int offset = Random3D::RandInt() % NumIndexedParams();
			float* ptr = &GetIndexedParam(offset);
			float minx = min_values.GetIndexedParam(offset);
			float maxx = max_values.GetIndexedParam(offset);
			
			if(minx != maxx)
			{
				float rscale = scale * (maxx - minx);
				*ptr = min(maxx, max(minx, *ptr + Random3D::Rand(-rscale, rscale)));

				ok = true;
			}

		} while(!ok || Random3D::RandInt() % count != 0);
	}

	float GACandidate::Crossover(float a, float b) { return a + Random3D::Rand() * (b - a); }

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

		unsigned int count = NumIndexedParams();
		WriteUInt32(count, ss);
		for(unsigned int i = 0; i < count; ++i)
			WriteSingle(GetIndexedParam(i), ss);
		
		BinaryChunk chunk("IDKOPS01");
		chunk.data = ss.str();
		chunk.Write(s);

		return 0;
	}

	unsigned int GACandidate::Read(istream& s, GACandidate*& result, unsigned int id)
	{
		result = nullptr;

		BinaryChunk chunk;
		chunk.Read(s);

		if(chunk.GetName() != "IDKOPS01")
			return 1;

		istringstream ss(chunk.data);

		GACandidate* candidate = new GACandidate(id);
		unsigned int count = ReadUInt32(ss);
		unsigned int mycount = NumIndexedParams();
		for(unsigned int i = 0; i < count; ++i)
		{
			float f = ReadSingle(ss);
			if(i < mycount)
				candidate->GetIndexedParam(i) = f;
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
					candidates.push_back(loadme);
			}

			if(!candidates.empty())
			{
				Debug(((stringstream&)(stringstream() << "Successfully loaded " << candidates.size() << " brains from genepool" << endl)).str());
				DebugGenerationStats();
			}

			file.close();
		}

		if(candidates.empty())
			MakeFirstGeneration();

		GenerateSubtestList();

		GenerateTrialTokens();
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
		subtests.clear();
		subtests.push_back(GASubtest());
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
		for(unsigned int i = 0; i < NUM_ELITES * NUM_ELITES; ++i)
		{
			GACandidate* candidate = new GACandidate(next_id++);
			
#if 0
			candidate->bone_t_weights[0] = 1.0f;
			for(unsigned int j = 1; j < 12; ++j)
				candidate->bone_t_weights[j] = 0.0f;
			for(unsigned int j = 0; j < 9; ++j)
				candidate->foot_t_absorbed[j] = 1.0f;
			//for(unsigned int j = 0; j < 6; ++j)
			//	candidate->leg_fixed_xfrac[j] = 0.0f;
			if(i >= NUM_ELITES)
				candidate->Randomize(MUTATION_COUNT * 2, MUTATION_SCALE);
#else
			for(unsigned int i = 0; i < GACandidate::NumIndexedParams(); ++i)
				candidate->GetIndexedParam(i) = Random3D::Rand(GACandidate::min_values.GetIndexedParam(i), GACandidate::max_values.GetIndexedParam(i));
#endif

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
					ss << '\t' << (*iter)->GetText() << endl;

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

		stringstream ss;

		unsigned int count = GACandidate::NumIndexedParams();
		for(unsigned int i = 0; i < count; ++i)
		{
			float tot = 0.0f;
			float min, max;
			for(unsigned int j = 0; j < analyze_these.size(); ++j)
			{
				float value = ((float*)&analyze_these[j]->params_prefix)[i + 1];
				tot += value;
				if(j == 0 || value < min)
					min = value;
				if(j == 0 || value > max)
					max = value;
			}

			float avg = tot / analyze_these.size();
			ss << "\tcomponent " << i << ": MIN = " << GACandidate::min_values.GetIndexedParam(i) << ", min = " << min << ", avg = " << avg << ", max = " << max << ", MAX = " << GACandidate::max_values.GetIndexedParam(i) << endl;
		}

		Debug(ss.str());
	}

	string GAExperiment::GetDebugText()
	{
		return debug_text;
	}
}
