#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;

	struct GASubtest
	{
		// TODO: add fields here?

		bool operator ==(const GASubtest& other) const { return true; } 
		bool operator < (const GASubtest& other) const { return false; }
	};

	struct GACandidate;

	struct GATrialToken
	{
		GACandidate* candidate;
		GASubtest subtest;
		unsigned int trial;

		GATrialToken() : candidate(nullptr), subtest(), trial(0) { }

		bool operator <(const GATrialToken& other) const { return candidate < other.candidate || candidate == other.candidate && (subtest < other.subtest || subtest == other.subtest && trial < other.trial); }
	};

	struct GACandidate
	{
		static const GACandidate min_values;
		static const GACandidate max_values;

		unsigned int id, p1, p2;

		float params_prefix;
		float bone_t_weights[12];
		float foot_t_absorbed[9];
		float foot_t_matching[9];
		float leg_fixed_xfrac[6];
		float params_suffix;

		float score;
		unsigned int time_spent;
		bool aborting;

		set<GATrialToken> tokens_busy;
		set<GATrialToken> tokens_not_finished;

		GACandidate(unsigned int id);
		GACandidate(unsigned int id, const GACandidate& other);
		GACandidate(unsigned int id, const GACandidate& p1, const GACandidate& p2);

		void Randomize(unsigned int count, float scale);

		static float Crossover(float a, float b);

		unsigned int Write(ostream& s);
		static unsigned int Read(istream& s, GACandidate*& result, unsigned int id);

		static GACandidate InitMin();
		static GACandidate InitMax();

		float& GetIndexedParam(unsigned int n);
		const float& GetIndexedParam(unsigned int n) const;

		static unsigned int NumIndexedParams();
	};

	class GAExperiment
	{
		private:

			mutex mutex;

			unsigned int batch;
			unsigned int next_id;

			string debug_text;

			vector<GASubtest> subtests;

			list<GACandidate*> elites;
			vector<GACandidate*> candidates;

			list<GATrialToken> tokens_not_started;
			set<GATrialToken> tokens_busy;

			void GenerateSubtestList();
			void GenerateTrialTokens();

			void MakeFirstGeneration();
			void MakeNextGeneration();

			void SaveElites(const string& filename, bool verbose = false);
			void DebugGenerationStats() const;

		public:

			GAExperiment(const string& filename);
			~GAExperiment();

			GATrialToken GetNextTrial();

			void TrialFinished(GATrialToken token, float score, unsigned int time_spent);

			float GetEarlyFailThreshold();

			string GetDebugText();
	};
}
