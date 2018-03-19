#include "StdAfx.h"
#include "GAExperiment.h"

#define NUM_ELITES			10

#define MUTATION_COUNT		20
#define MUTATION_SCALE		0.01f

#define TRIALS_PER_SUBTEST	10


namespace Test
{
	using namespace CibraryEngine;

	/*
	 * GACandidate methods
	 */
	GACandidate::GACandidate(unsigned int id) : id(id), p1(0), p2(0), score(0.0f), aborting(false), tokens_busy(), tokens_not_finished()
	{
		memset(kp, 0, sizeof(kp));
		memset(ki, 0, sizeof(ki));
		memset(kd, 0, sizeof(kd));
		memset(initial, 0, sizeof(initial));
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& other) : id(id), p1(other.id), p2(other.id), score(0.0f), aborting(false), tokens_busy(), tokens_not_finished()
	{
		memcpy(kp, other.kp, sizeof(kp));
		memcpy(ki, other.ki, sizeof(ki));
		memcpy(kd, other.kd, sizeof(kd));
		memcpy(initial, other.initial, sizeof(initial));
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& p1, const GACandidate& p2) : id(id), p1(p1.id), p2(p2.id), score(0.0f), aborting(false), tokens_busy(), tokens_not_finished()
	{
		for(unsigned int i = 0; i < 21; ++i)
		{
			kp[i] = Crossover(p1.kp[i], p2.kp[i]);
			ki[i] = Crossover(p1.ki[i], p2.ki[i]);
			kd[i] = Crossover(p1.kd[i], p2.kd[i]);
			initial[i] = Crossover(p1.initial[i], p2.initial[i]);
		}

		Randomize(MUTATION_COUNT, MUTATION_SCALE);
	}

	void GACandidate::Randomize(unsigned int count, float scale)
	{
		do
		{
			float* ptr;
			float minx;
			float maxx;

			unsigned int index = Random3D::RandInt() % 21;
			switch(Random3D::RandInt() % 4)
			{
				case 0:
					ptr = kp + index;
					minx = 0;
					maxx = 100;
					break;
				case 1:
					ptr = ki + index;
					minx = 0;
					maxx = 100;
					break;
				case 2:
					ptr = kd + index;
					minx = 0;
					maxx = 20;
					break;
				case 3:
					ptr = initial + index;
					minx = -20;
					maxx =  20;
					break;
			}

			float rscale = (maxx - minx) * scale;
			*ptr = max(minx, min(maxx, *ptr + Random3D::Rand(-rscale, rscale)));

		} while(Random3D::RandInt() % count == 0);
	}

	float GACandidate::Crossover(float a, float b) { return a + Random3D::Rand() * (b - a); }

	unsigned int GACandidate::Write(ostream& s)
	{
		stringstream ss;
		for(unsigned int i = 0; i < 21; ++i)
		{
			WriteSingle(kp[i], ss);
			WriteSingle(ki[i], ss);
			WriteSingle(kd[i], ss);
			WriteSingle(initial[i], ss);
		}
		
		BinaryChunk chunk("IDKPID01");
		chunk.data = ss.str();
		chunk.Write(s);

		return 0;
	}

	unsigned int GACandidate::Read(istream& s, GACandidate*& result, unsigned int id)
	{
		result = nullptr;

		BinaryChunk chunk;
		chunk.Read(s);

		if(chunk.GetName() != "IDKPID01")
			return 1;

		istringstream ss(chunk.data);

		GACandidate* candidate = new GACandidate(id);
		for(unsigned int i = 0; i < 21; ++i)
		{
			candidate->kp[i] = ReadSingle(ss);
			candidate->ki[i] = ReadSingle(ss);
			candidate->kd[i] = ReadSingle(ss);
			candidate->initial[i] = ReadSingle(ss);
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
				GACandidate* loadme = NULL;
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

				// TODO: display stats?
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
			if(i >= NUM_ELITES)
				candidate->Randomize(MUTATION_COUNT * 2, MUTATION_SCALE);
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
					for(unsigned int n = 0; n < 2; ++n)
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

	void GAExperiment::TrialFinished(GATrialToken token, float score)
	{
		unique_lock<std::mutex> lock(mutex);

		GACandidate& candidate = *token.candidate;
		candidate.score += score;

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
			ss << "(" << candidate.id << ", p " << candidate.p1 << ", " << candidate.p2 << ") score = " << candidate.score;

			list<GACandidate*>::iterator insert_where;
			for(insert_where = elites.begin(); insert_where != elites.end(); ++insert_where)
				if(!candidate.aborting && (*insert_where)->score > candidate.score)
					break;

			elites.insert(insert_where, token.candidate);

			if(*elites.begin() == token.candidate)
				ss << "; new best!";
			else if(*elites.rbegin() == token.candidate && elites.size() > NUM_ELITES)
				ss << "; fail";		// TODO: log early-abort savings

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
				ss2 << "(" << c.id << ", p " << c.p1 << ", " << c.p2 << ") " << c.score << endl;
			}
			debug_text = ss2.str();

			if(elites.size() > NUM_ELITES)
				elites.pop_back();
		}

		if(tokens_busy.empty() & tokens_not_started.empty())
		{
			stringstream ss;
			ss << "batch " << batch << " elites:" << endl;
			
			Debug(ss.str());

			time_t raw_time;
			time(&raw_time);
			tm now = *localtime(&raw_time);
			string filename = ((stringstream&)(stringstream() << "Files/Brains/genepool-" << now.tm_year + 1900 << "-" << now.tm_mon + 1 << "-" << now.tm_mday << "-" << now.tm_hour << "-" << now.tm_min << "-" << now.tm_sec)).str();

			SaveElites(filename, true);
			SaveElites("Files/Brains/genepool");
			
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
					ss << "\t(" << c.id << ", p " << c.p1 << ", " << c.p2 << ") score = " << c.score << endl;
				}

				Debug(ss.str());
			}
		}

		// TODO: save to file(s)
	}

	string GAExperiment::GetDebugText()
	{
		return debug_text;
	}
}
