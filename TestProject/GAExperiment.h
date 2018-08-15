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

	struct GPOp;

	struct GPOperand
	{
		unsigned char src;			// 0 integer constant, 1 node strict inputs, 2 node scratch memory, 3 node old persistent memory, 4 parent scratch memory, 5 sum of children's scratch memory, 6 brain connection, 7 float constants table
		unsigned char index;

		GPOperand() : src(0), index(0) { }
		GPOperand(unsigned char src, unsigned char index) : src(src), index(index) { }

		void Randomize(bool brain, unsigned int my_index, const vector<GPOp>& ops);

		bool SourceIsScratch() const { return src == 2 || src == 4 || src ==  5 || src == 6; }

		string SourceToString() const;
	};

	struct GPOp
	{
		bool brain;					// if true, applies to brain node; else applies to all rigid body nodes
		unsigned char opcode;		// +, -, *, /, tanh (fixes nan/inf), average, compare (fixes nan/inf), sqrt, square, pow
		unsigned char dst_class;	// 0 scratch memory, 1 persistent memory, 2 strict output
		unsigned char dst_index;
		GPOperand arg1;
		GPOperand arg2;				// depending on the opcode, may be unused

		GPOp() : brain(false), opcode(0), dst_class(0), dst_index(0), arg1(), arg2() { }

		void Randomize(unsigned int my_index, const vector<GPOp>& ops);

		static bool IsUnary(unsigned char opcode) { return opcode == 4 || opcode  == 7 || opcode == 8; }

		string ToString() const;
	};

	struct GACandidate
	{
		unsigned int id, p1, p2;

		unsigned short mutations[12];

		float constants[64];
		vector<GPOp> ops;

		bool compiled_flag;
		vector<GPOp> compiled;

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
		static float Crossover(float a, float b);
		void Compile();
		void GetLastOutputs(const vector<GPOp>& ops, vector<unsigned int>& last_outputs) const;
		bool RemoveUnreferencedScratchSteps(const vector<GPOp>& ops_in, vector<GPOp>& ops_out, const vector<unsigned int>& last_outputs) const;

		string GetText() const;

		unsigned int Write(ostream& s);
		static unsigned int Read(istream& s, GACandidate*& result, unsigned int id);

		static string CompiledToString(unsigned int id, const vector<GPOp>& ops);

		static unsigned int GetInputCoverage(const vector<GPOp>& ops);
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
