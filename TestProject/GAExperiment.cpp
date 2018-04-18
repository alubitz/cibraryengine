#include "StdAfx.h"
#include "GAExperiment.h"

#include "MultiLayerBrain.h"

#define NUM_ELITES			10

#define CROSSOVERS_PER_PAIR	2

#define MUTATION_COUNT		10
#define MUTATION_SCALE		0.5f

#define TRIALS_PER_SUBTEST	20


namespace Test
{
	using namespace CibraryEngine;

	/*
	 * GACandidate methods
	 */
	GACandidate::GACandidate(unsigned int id) : id(id), p1(0), p2(0), brains(), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished() { }

	GACandidate::GACandidate(unsigned int id, const GACandidate& other) : id(id), p1(other.id), p2(other.id), brains(), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		for(unsigned int i = 0; i < other.brains.size(); ++i)
			brains.push_back(new MultiLayerBrain(*other.brains[i]));
	}

	GACandidate::GACandidate(unsigned int id, const GACandidate& p1, const GACandidate& p2) : id(id), p1(p1.id), p2(p2.id), brains(), score(0.0f), time_spent(0), aborting(false), tokens_busy(), tokens_not_finished()
	{
		unsigned int num_brains = min(p1.brains.size(), p2.brains.size());
		for(unsigned int bindex = 0; bindex < num_brains; ++bindex)
		{
			const MultiLayerBrain& b1 = *p1.brains[bindex];
			const MultiLayerBrain& b2 = *p2.brains[bindex];

			unsigned int num_layers = min(b1.layer_sizes.size(), b2.layer_sizes.size());
			vector<unsigned int> layer_sizes(num_layers);
			for(unsigned int i = 0; i < num_layers; ++i)
				layer_sizes[i] = min(b1.layer_sizes[i], b2.layer_sizes[i]);

			MultiLayerBrain* brain = new MultiLayerBrain(layer_sizes);
		
			for(unsigned int i = 1; i < num_layers; ++i)
			{
				for(unsigned int j = 0; j < layer_sizes[i]; ++j)
					for(unsigned int k = 0; k < layer_sizes[i - 1]; ++k)
						brain->matrices[i - 1][j * layer_sizes[i - 1] + k] = Crossover(b1.matrices[i - 1][j * b1.layer_sizes[i - 1] + k], b2.matrices[i - 1][j * b2.layer_sizes[i - 1] + k]);
			}

			brains.push_back(brain);
		}

		Randomize(MUTATION_COUNT, MUTATION_SCALE);
	}

	GACandidate::~GACandidate()
	{
		for(unsigned int i = 0; i < brains.size(); ++i)
			delete brains[i];
		brains.clear();
	}

	void GACandidate::Randomize(unsigned int count, float scale)
	{
		//unsigned int total_coeffs = 0;
		//for(unsigned int i = 0; i < brain->matrices.size(); ++i)
		//	total_coeffs += brain->matrices[i].size();

		set<float*> nonzeroes;
		for(unsigned int bindex = 0; bindex < brains.size(); ++bindex)
		{
			MultiLayerBrain* brain = brains[bindex];
			for(unsigned int mindex = 0; mindex < brain->matrices.size(); ++mindex)
			{
				//if(bindex == 2 ||  bindex == 1 && mindex == 2)
					for(unsigned int j = 0; j < brain->matrices[mindex].size(); ++j)
						if(brain->matrices[mindex][j] != 0)
							nonzeroes.insert(brain->matrices[mindex].data() + j);
			}
		}

		bool ok;
		do
		{
			//// equal distribution to each coefficient (so smaller matrices don't get disporportionately many mutations)
			//unsigned int r = Random3D::BigRand(total_coeffs);
			//for(unsigned int i = 0; i < brain->matrices.size(); ++i)
			//{
			//	vector<float>& matrix = brain->matrices[i];
			//	if(r < matrix.size())
			//	{
			//		matrix[r] += Random3D::Rand(-scale, scale);
			//
			//		ok = true;
			//		break;
			//	}
			//	else
			//		r -= matrix.size();
			//}
			ok = false;

			unsigned int action = Random3D::RandInt(10);
			if(action == 0 || nonzeroes.size() < 50)
			{
				unsigned int bindex = Random3D::RandInt(brains.size());
				MultiLayerBrain& brain = *brains[bindex];
				unsigned int mindex = Random3D::RandInt(brain.matrices.size());
				//if(bindex == 2 ||  bindex == 1 && mindex == 2)
				{
					vector<float>& matrix = brain.matrices[mindex];
					unsigned int eindex = Random3D::RandInt(matrix.size());

					//if(mindex == 1)
					//{
					//	unsigned int square = bindex == 0 ? 30 : bindex == 1 ? 40 : 15;
					//	unsigned int x = eindex % square, y = eindex / square;
					//	if(x < square / 2 && y >= square / 2 || x >= square / 2 && y < square / 2)
					//		continue;
					//}

					float* ptr = matrix.data() + eindex;
					*ptr += Random3D::Rand(-scale, scale);
					nonzeroes.insert(ptr);
					ok = true;
				}
			}
			else
			{
				unsigned int r = Random3D::BigRand(nonzeroes.size());
				for(set<float*>::iterator iter = nonzeroes.begin(); iter != nonzeroes.end(); ++iter)
				{
					if(r == 0)
					{
						switch(action)
						{
							case 1:
								**iter = 0.0f;
								nonzeroes.erase(*iter);
								break;

							case 2:
								**iter *= 0.5f;
								break;

							default:
								**iter += Random3D::Rand(-scale, scale);
								break;
						}

						ok = true;
						break;
					}
					else
						--r;
				}

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

		WriteUInt32(brains.size(), ss);
		for(unsigned int i = 0; i < brains.size(); ++i)
			if(unsigned int error = brains[i]->Write(ss))
			{
				Debug(((stringstream&)(stringstream() << "MultiLayerBrain::Write[" << i << "] returned error " << error << endl)).str());
				return error;
			}
		
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
		unsigned int num_brains = ReadUInt32(ss);
		for(unsigned int i = 0; i < num_brains; ++i)
		{
			MultiLayerBrain* brain;
			if(unsigned int error = MultiLayerBrain::Read(ss, brain, id))
			{
				Debug(((stringstream&)(stringstream() << "MultiLayerBrain::Read[" << i << "] returned error " << error << endl)).str());
				return 2 + error;
			}
			else
				candidate->brains.push_back(brain);
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
		static const unsigned int signal_up		= 30;//20;
		static const unsigned int signal_down	= 10;//20;
		static const unsigned int signal_self	= 20;

		vector<unsigned int> sizes_a;
		sizes_a.push_back(126);
		sizes_a.push_back(30);		//20
		sizes_a.push_back(30);		//20
		sizes_a.push_back(signal_up + signal_self);

		vector<unsigned int> sizes_b;
		sizes_b.push_back(95 + signal_up);
		sizes_b.push_back(40);
		sizes_b.push_back(40);
		sizes_b.push_back(6 + signal_down);

		vector<unsigned int> sizes_c;
		sizes_c.push_back(6 + signal_down + signal_self);
		sizes_c.push_back(15);
		sizes_c.push_back(15);
		sizes_c.push_back(5);

		for(unsigned int i = 0; i < NUM_ELITES * NUM_ELITES; ++i)
		{
			GACandidate* candidate = new GACandidate(next_id++);
			
			candidate->brains.push_back(new MultiLayerBrain(sizes_a));
			candidate->brains.push_back(new MultiLayerBrain(sizes_b));
			candidate->brains.push_back(new MultiLayerBrain(sizes_c));

			//if(i >= NUM_ELITES)
			//	candidate->Randomize(MUTATION_COUNT * 2, MUTATION_SCALE);
			//if(i >= NUM_ELITES)
			{
				for(unsigned int j = 0; j < candidate->brains.size(); ++j)
				{
					MultiLayerBrain& brain = *candidate->brains[j];
					for(unsigned int k = 0; k < brain.matrices.size(); ++k)
					{
						vector<float>& matrix = brain.matrices[k];
						for(unsigned int m = 0; m < brain.layer_sizes[k] + brain.layer_sizes[k + 1]; ++m)
							matrix[Random3D::RandInt(matrix.size())] = Random3D::Rand(-0.01f, 0.01f);
					}
				}
			}

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
		else
			assert(candidate.score_parts.size() == score_parts.size());

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
					ss << '\t' << c.GetText() << "; nonzeros { ";// << endl;
					for(unsigned int bindex = 0; bindex < c.brains.size(); ++bindex)
					{
						if(bindex != 0)
							ss << "; ";
						ss << "brain[" << bindex << "]: ";

						const MultiLayerBrain& brain = *c.brains[bindex];
						const vector<vector<float>>& matrices = brain.matrices;
						for(unsigned int i = 0; i < matrices.size(); ++i)
						{
							const vector<float>& matrix = matrices[i];
							unsigned int nonzero = 0;
							for(unsigned int j = 0; j < matrix.size(); ++j)
								if(matrix[j] != 0)
									++nonzero;
							if(i != 0)
								ss << ", ";
							ss << nonzero << " / " << matrix.size();
						}
					}
					ss << " }" << endl;
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

		/*stringstream ss;

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

		Debug(ss.str());*/
	}

	string GAExperiment::GetDebugText()
	{
		return debug_text;
	}
}
