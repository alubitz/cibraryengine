#include "StdAfx.h"
#include "BetterBrain.h"

namespace Test
{
	/*
	 * BetterBrain::Array methods
	 */
	void BetterBrain::Array::CopyFromVector(const vector<float>& src) { memcpy(ptr, src.data(),  min(size, src.size())  * sizeof(float)); }
	void BetterBrain::Array::CopyToVector(vector<float>& dest) const  { memcpy(dest.data(), ptr, min(size, dest.size()) * sizeof(float)); }
	void BetterBrain::Array::SetZero()                                { memset(ptr, 0, size * sizeof(float)); }



	/*
	 * BetterBrain::Matrix methods
	 */
	void BetterBrain::Matrix::Mutate(unsigned int odds_against, float scale)
	{
		for(float *mptr = ptr, *mend = mptr + wh; mptr != mend; ++mptr)
			if(Random3D::RandInt() % odds_against == 0)
				*mptr += Random3D::Rand(-scale, scale);
	}

	void BetterBrain::Matrix::Write(ostream& o) const
	{
		WriteUInt32(w, o);
		WriteUInt32(h, o);

		for(float *p = ptr, *pend = p + wh; p != pend; ++p)
			WriteSingle(*p, o);
	}

	BetterBrain::Matrix* BetterBrain::Matrix::Read(istream& i)
	{
		unsigned int w = ReadUInt32(i);
		unsigned int h = ReadUInt32(i);

		Matrix* result = new Matrix(w, h);

		for(float *p = result->ptr, *pend = p + result->wh; p != pend; ++p)
			*p = ReadSingle(i);

		return result;
	}

	void BetterBrain::Matrix::CopySharedElements(const Matrix& other)
	{
		unsigned int minw = min(w, other.w);
		unsigned int minh = min(h, other.h);

		for(float *rowa = ptr, *rowb = other.ptr, *rend = rowa + w * minh; rowa != rend; rowa += w, rowb += other.w)
			for(float *aptr = rowa, *bptr = rowb, *aend = rowa + minw; aptr != aend; ++aptr, ++bptr)
				*aptr = *bptr;
	}



	/*
	 * BetterBrain::Op methods
	 */
	void BetterBrain::Op::Write(ostream& o) const
	{
		WriteUInt32(mat,         o);
		WriteUInt32(from.size(), o);
		WriteUInt32(to.size(),   o);

		for(unsigned int i = 0; i < from.size(); ++i)
			WriteUInt32(from[i], o);
		for(unsigned int i = 0; i < to.size(); ++i)
			WriteUInt32(to[i], o);
	}

	BetterBrain::Op BetterBrain::Op::Read(istream& i)
	{
		Op op;
		op.mat = ReadUInt32(i);

		unsigned int num_froms = ReadUInt32(i);
		unsigned int num_tos   = ReadUInt32(i);

		op.from.clear();
		op.to.clear();

		for(unsigned int j = 0; j < num_froms; ++j)
			op.from.push_back(ReadUInt32(i));
		for(unsigned int j = 0; j < num_tos; ++j)
			op.to.push_back(ReadUInt32(i));

		return op;
	}




	/*
	 * BetterBrain methods
	 */
	BetterBrain::BetterBrain(const BetterBrain& other) : arrays(other.arrays.size()), mats(other.mats.size()), ops(other.ops.size())
	{
		for(unsigned int i = 0; i < arrays.size(); ++i)
			arrays[i] = new Array(*other.arrays[i]);
		for(unsigned int i = 0; i < mats.size(); ++i)
			mats[i] = new Matrix(*other.mats[i]);
		for(unsigned int i = 0; i < ops.size(); ++i)
			ops[i] = other.ops[i];
	}

	BetterBrain::~BetterBrain()
	{
		for(unsigned int i = 0; i < arrays.size(); ++i)
			delete arrays[i];
		arrays.clear();

		for(unsigned int i = 0; i < mats.size(); ++i)
			delete mats[i];
		mats.clear();

		ops.clear();
	}

	void BetterBrain::Evaluate()
	{
		vector<float> inputs, outputs;
		for(unsigned int i = 0; i < ops.size(); ++i)
		{
			const Op& op = ops[i];

			unsigned int num_inputs = 0, num_outputs = 0;
			for(unsigned int j = 0; j < op.from.size(); ++j)
				num_inputs += arrays[op.from[j]]->size;
			for(unsigned int j = 0; j < op.to.size(); ++j)
				num_outputs += arrays[op.to[j]]->size;

			const Matrix& mat = *mats[op.mat];
			if(mat.w == num_outputs && mat.h == num_inputs)
			{
				inputs.resize(num_inputs);
				float* ip = inputs.data();
				for(unsigned int j = 0; j < op.from.size(); ++j)
				{
					const Array& from = *arrays[op.from[j]];
					memcpy(ip, from.ptr, from.size * sizeof(float));
					ip += from.size;
				}
				outputs.resize(num_outputs);

				const float *ibegin = inputs.data(),  *iend = ibegin + num_inputs;
				const float *mbegin = mat.ptr,        *mend = mbegin + mat.wh;
				float       *obegin = outputs.data(), *oend = obegin + num_outputs;

				const float *iptr, *mptr = mbegin;
				float *optr;
				for(optr = obegin; optr != oend; ++optr)
				{
					*optr = 0.0f;
					for(iptr = ibegin; iptr != iend; ++iptr, ++mptr)
						*optr += *iptr * *mptr;
					*optr = tanhf(*optr);
				}

				optr = obegin;
				for(unsigned int j = 0; j < op.to.size(); ++j)
				{
					Array& to = *arrays[op.to[j]];
					memcpy(to.ptr, optr, to.size * sizeof(float));
					optr += to.size;
				}
			}
			else
				Debug(((stringstream&)(stringstream() << "ops[" << i << "] expected a " << num_outputs << " x " << num_inputs << " matrix, but instead got one which is " << mat.w << " x " << mat.h << endl)).str());
		}
	}

	void BetterBrain::MutateAll(unsigned int odds_against, float scale)
	{
		for(unsigned int i = 0; i < mats.size(); ++i)
			if(mats[i]->mutate)
				mats[i]->Mutate(odds_against, scale);
	}

	unsigned int BetterBrain::AddArray(unsigned int size)
	{
		arrays.push_back(new Array(size));
		return arrays.size() - 1;
	}
	unsigned int BetterBrain::AddMatrixBySize(unsigned int w, unsigned int h)
	{
		mats.push_back(new Matrix(w, h));
		return mats.size() - 1;
	}

	unsigned int BetterBrain::AddMatrixAndOp(const vector<unsigned int>& from, const vector<unsigned int>& to)
	{
		unsigned int atot = 0, btot = 0;

		for(unsigned int i = 0; i < from.size(); ++i)
			atot += arrays[from[i]]->size;
		for(unsigned int i = 0; i < to.size(); ++i)
			btot += arrays[to[i]]->size;

		mats.push_back(new Matrix(btot, atot));
		ops.push_back(Op(mats.size() - 1, from, to));
		return mats.size() - 1;
	}

	unsigned int BetterBrain::AddMatrixAndOp(unsigned int from, unsigned int to)
	{
		vector<unsigned int> froms, tos;

		froms.push_back(from);
		tos.push_back(to);

		return AddMatrixAndOp(froms, tos);
	}

	void BetterBrain::AddOpExistingMatrix(unsigned int mat, const vector<unsigned int>& from, const vector<unsigned int>& to) { ops.push_back(Op(mat, from, to)); }

	void BetterBrain::AddOpExistingMatrix(unsigned int mat, unsigned int from, unsigned int to)
	{
		vector<unsigned int> froms, tos;

		froms.push_back(from);
		tos.push_back(to);

		AddOpExistingMatrix(mat, froms, tos);
	}

	unsigned int BetterBrain::AddOpCreateMatrix(Op& op) { return op.mat = AddMatrixAndOp(op.from, op.to); }

	void BetterBrain::Write(ostream& o) const
	{
		BinaryChunk bc("BTRBRAIN");
		
		stringstream ss;
		
		WriteUInt32(arrays.size(), ss);
		WriteUInt32(mats.size(),   ss);
		WriteUInt32(ops.size(),    ss);

		for(unsigned int i = 0; i < arrays.size(); ++i)
			WriteUInt32(arrays[i]->size, ss);

		for(unsigned int i = 0; i < mats.size(); ++i)
			mats[i]->Write(ss);

		for(unsigned int i = 0; i < ops.size(); ++i)
			ops[i].Write(ss);

		WriteUInt32(7890, ss);

		bc.data = ss.str();
		bc.Write(o);
	}

	BetterBrain* BetterBrain::Read(istream& i)
	{
		BinaryChunk bc;
		bc.Read(i);

		if(bc.GetName() != "BTRBRAIN")
			return NULL;

		istringstream ss(bc.data);

		unsigned int num_arrays = ReadUInt32(ss);
		unsigned int num_mats   = ReadUInt32(ss);
		unsigned int num_ops    = ReadUInt32(ss);

		BetterBrain* result = new BetterBrain();

		for(unsigned int i = 0; i < num_arrays; ++i)
			result->arrays.push_back(new Array(ReadUInt32(ss)));

		for(unsigned int i = 0; i < num_mats; ++i)
			result->mats.push_back(Matrix::Read(ss));

		for(unsigned int i = 0; i < num_ops; ++i)
			result->ops.push_back(Op::Read(ss));

		unsigned int derpflag = ReadUInt32(ss);
		if(derpflag != 7890)
		{
			DEBUG();

			delete result;
			return NULL;
		}

		return result;
	}
}
