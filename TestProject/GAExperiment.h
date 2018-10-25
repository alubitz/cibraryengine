#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	struct GASubtest
	{
		unsigned int id;
		Vec3 x1, x2;

		GASubtest() : id(0), x1(), x2() { }
		GASubtest(unsigned int id, const Vec3& x1, const Vec3& x2) : id(id), x1(x1), x2(x2) { }

		bool operator ==(const GASubtest& other) const { return memcmp(this, &other, sizeof(GASubtest)) == 0; }
		bool operator < (const GASubtest& other) const { return memcmp(this, &other, sizeof(GASubtest)) < 0; }
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

	struct GPOp
	{
		vector<float> input_weights;
		vector<float> mem_weights;
		vector<float> scratch_weights;
		vector<float> parent_weights;
		vector<float> child_weights;

		GPOp() : input_weights(), scratch_weights(), parent_weights(), child_weights() { }

		unsigned int Write(ostream& s);
		unsigned int Read(istream& s);

	};

	struct GACandidate
	{
		unsigned int id, p1, p2;

		unsigned short mutations[1];

		vector<GPOp> scratch_ops;
		vector<GPOp> output_ops;
		vector<GPOp> mem_ops;

		float score;
		vector<float> score_parts;

		unsigned int time_spent;
		bool aborting;
		set<GATrialToken> tokens_busy;
		set<GATrialToken> tokens_not_finished;

		GACandidate(unsigned int id);
		GACandidate(unsigned int id, const GACandidate& other);							// clone
		GACandidate(unsigned int id, const GACandidate& p1, const GACandidate& p2);		// crossover
		~GACandidate();

		void Randomize(unsigned int count, float scale);
		static vector<GPOp> Crossover(const vector<GPOp>& a, const vector<GPOp>& b);
		static vector<float> Crossover(const vector<float>& a, const vector<float>& b);
		static float Crossover(float a, float b);

		void Resize();

		string GetText() const;

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
			void DebugGenerationStats() const;

		public:

			GAExperiment(const string& filename);
			~GAExperiment();

			GATrialToken GetNextTrial();

			void TrialFinished(GATrialToken token, float score, const vector<float>& score_parts, unsigned int time_spent);

			float GetEarlyFailThreshold();

			string GetDebugText();
	};
}
