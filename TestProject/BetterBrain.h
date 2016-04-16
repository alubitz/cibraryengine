#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;
	using namespace CibraryEngine;

	struct BetterBrain
	{
		struct Array
		{
			unsigned int size;
			float* ptr;

			Array(unsigned int size) : size(size), ptr(new float[size]) { }
			~Array() { if(ptr) { delete[] ptr; ptr = NULL; } }

			Array(const Array& other) : size(other.size), ptr(new float[size]) { memcpy(ptr, other.ptr, size * sizeof(float)); }
			void operator =(const Array& other) { if(this != &other) { this->~Array(); *this = other; } }

			void CopyFromVector(const vector<float>& src);
			void CopyToVector(vector<float>& dest) const;			// doesn't resize!

			void SetZero();
		};

		struct Matrix
		{
			unsigned int w, h, wh;
			float* ptr;

			bool mutate;

			Matrix(unsigned int w, unsigned int h) : w(w), h(h), wh(w * h), ptr(new float[wh]), mutate(false) { memset(ptr, 0, wh * sizeof(float)); }
			~Matrix() { if(ptr) { delete[] ptr; ptr = NULL; } }

			Matrix(const Matrix& other) : w(other.w), h(other.h), wh(w * h), ptr(new float[wh]), mutate(other.mutate) { memcpy(ptr, other.ptr, wh * sizeof(float)); }
			void operator =(const Matrix& other) { if(this != &other) { this->~Matrix(); *this = other; } }

			void Mutate(unsigned int odds_against, float scale);

			void Write(ostream& o) const;
			static Matrix* Read(istream& i);

			// copies elements that are within the range of both matrices (no resizing)
			void CopySharedElements(const Matrix& other);
		};

		struct Op
		{
			unsigned int mat;
			vector<unsigned int> from, to;

			Op() : mat(0), from(), to() { }
			Op(unsigned int mat, const vector<unsigned int>& from, const vector<unsigned int>& to) : mat(mat), from(from), to(to) { }

			void Write(ostream& o) const;
			static Op Read(istream& i);
		};

		vector<Array*>  arrays;
		vector<Matrix*> mats;
		vector<Op>      ops;

		BetterBrain() : arrays(), mats(), ops() { }
		~BetterBrain();

		BetterBrain(const BetterBrain& other);
		void operator =(const BetterBrain& other) { if(this != &other) { this->~BetterBrain(); *this = other; } }

		void Evaluate();

		void MutateAll(unsigned int odds_against, float scale);

		unsigned int AddArray(unsigned int size);
		unsigned int AddMatrixBySize(unsigned int w, unsigned int h);
		unsigned int AddMatrixAndOp(const vector<unsigned int>& from, const vector<unsigned int>& to);			// returns matrix index
		unsigned int AddMatrixAndOp(unsigned int from, unsigned int to);										// returns matrix index
		
		unsigned int AddOpCreateMatrix(Op& op);																	// returns matrix index
		void AddOpExistingMatrix(unsigned int mat, const vector<unsigned int>& from, const vector<unsigned int>& to);
		void AddOpExistingMatrix(unsigned int mat, unsigned int from, unsigned int to);

		void Write(ostream& o) const;
		static BetterBrain* Read(istream& i);
	};
}
