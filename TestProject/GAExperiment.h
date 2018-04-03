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
		unsigned int id, p1, p2;

		string ops;

		float score;
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

		public:

			GAExperiment(const string& filename);
			~GAExperiment();

			GATrialToken GetNextTrial();

			void TrialFinished(GATrialToken token, float score);

			float GetEarlyFailThreshold();

			string GetDebugText();
	};
}
