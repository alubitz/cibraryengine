#include "StdAfx.h"
#include "GAExperiment.h"

#define NUM_ELITES			20

#define CROSSOVERS_PER_PAIR	1

#define MUTATION_COUNT		4
#define MUTATION_SCALE		0.001f

#define TRIALS_PER_SUBTEST	10


namespace Test
{
	using namespace CibraryEngine;

	/*
	 * GACandidate methods
	 */
	GACandidate::GACandidate(unsigned int id) : id(id), p1(0), p2(0), ops(), score(0.0f), aborting(false), tokens_busy(), tokens_not_finished()
	{
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& other) : id(id), p1(other.id), p2(other.id), ops(other.ops), score(0.0f), aborting(false), tokens_busy(), tokens_not_finished()
	{
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& p1, const GACandidate& p2) : id(id), p1(p1.id), p2(p2.id), ops(), score(0.0f), aborting(false), tokens_busy(), tokens_not_finished()
	{
		const GACandidate* a = &p1;
		const GACandidate* b = &p2;

		if(Random3D::RandInt() % 2 == 0)
			swap(a, b);

		unsigned int an = a->ops.empty() ? 0 : Random3D::RandInt(a->ops.size());
		unsigned int bn = b->ops.empty() ? 0 : Random3D::RandInt(b->ops.size());

		ops = a->ops.substr(0, an) + b->ops.substr(b->ops.size() - bn);

		Randomize(MUTATION_COUNT, MUTATION_SCALE);
	}

	void GACandidate::Randomize(unsigned int count, float scale)
	{
		bool ok;
		do
		{
			if(ops.empty())
			{
				ops += (char)Random3D::RandInt(256);
				ok = true;
			}
			else
			{
				unsigned int pos = Random3D::RandInt() % ops.size();
				switch(Random3D::RandInt() % 2)
				{
					case 0:		// insert
					{
						string after = ops.substr(pos);
						ops = ops.substr(0, pos);

						unsigned int len = Random3D::RandInt(1, 10);
						for(unsigned int i = 0; i < len; ++i)
							ops += (char)Random3D::RandInt(256);
						ops += after;

						ok = true;

						break;
					}

					case 1:		// mutate
					{
						ops = ops.substr(0, pos - 1) + (char)Random3D::RandInt(256) + ops.substr(pos);
						ok = true;
						break;
					}

					case 2:		// remove
						ops = ops.substr(0, pos - 1) + ops.substr(pos + 1);
						ok = true;
						break;
				}
			}

		} while(!ok || Random3D::RandInt() % count == 0);
	}

	float GACandidate::Crossover(float a, float b) { return a + Random3D::Rand() * (b - a); }

	unsigned int GACandidate::Write(ostream& s)
	{
		stringstream ss;
		for(unsigned int i = 0; i < 3; ++i)
			WriteString4(ops, ss);
		
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
		for(unsigned int i = 0; i < 3; ++i)
			candidate->ops = ReadString4(ss);

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
				stringstream ss;
				ss << "Successfully loaded " << candidates.size() << " brains from genepool" << endl;

				/*for(unsigned int i = 0; i < 3; ++i)
				{
					float tot = 0.0f;
					float min, max;
					for(unsigned int j = 0; j < candidates.size(); ++j)
					{
						float value = ((float*)&candidates[j]->foot_fudge)[i];
						tot += value;
						if(j == 0 || value < min)
							min = value;
						if(j == 0 || value > max)
							max = value;
					}

					ss << "\tcomponent " << i << ": min = " << min << "; max = " << max << "; avg = " << tot / candidates.size() << endl;
				}

				Debug(ss.str());*/
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
			//if(i >= NUM_ELITES)
			//	candidate->Randomize(MUTATION_COUNT * 2, MUTATION_SCALE);
			for(unsigned int j = 0; j < 500; ++j)
				candidate->ops += (char)Random3D::RandInt(256);
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
					ss << "\t(" << c.id << ", p " << c.p1 << ", " << c.p2 << ") score = " << c.score << "; genome length = " << c.ops.size() << endl;
				}

				Debug(ss.str());
			}
		}
	}

	string GAExperiment::GetDebugText()
	{
		return debug_text;
	}
}
