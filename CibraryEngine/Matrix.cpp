#include "StdAfx.h"
#include "Matrix.h"
#include "Vector.h"
#include "Quaternion.h"

#include "Serialize.h"

namespace CibraryEngine
{
	/*
	 * Mat3 method
	 */
	static void row_swap3(float*& r1, float*& r2)
	{
		float* temp = r1;
		r1 = r2;
		r2 = temp;
	}
	static void row_mult3(float* row, float coeff)
	{
		for(int i = 0; i < 6; ++i)
			row[i] *= coeff;
	}
	static void row_combine3(float* from, float* to, float coeff)
	{
		for(int i = 0; i < 6; ++i)
			to[i] += from[i] * coeff;
	}

	Mat3 Mat3::Invert(const Mat3& a)
	{
		const float* v = a.values;
		float ra[] = {	v[0],	v[1],	v[2],	1, 0, 0 };
		float rb[] = {	v[3],	v[4],	v[5],	0, 1, 0 };
		float rc[] = {	v[6],	v[7],	v[8],	0, 0, 1 };

		float *r[] = { ra, rb, rc };

		// working on first column...
		for(int i = 0; i < 2; ++i)
			for(int j = 0; j < 2 - i; ++j)
				if(fabs(r[j][0]) < fabs(r[j + 1][0]))
					row_swap3(r[j], r[j + 1]);
		row_mult3(r[0], 1.0f / r[0][0]);
		if(r[1][0] != 0)
			row_combine3(r[0], r[1], -r[1][0]);
		if(r[2][0] != 0)
			row_combine3(r[0], r[2], -r[2][0]);

		// working on second column
		if(fabs(r[1][1]) < fabs(r[2][1]))
			row_swap3(r[1], r[2]);
		row_mult3(r[1], 1.0f / r[1][1]);
		if(r[0][1] != 0)
			row_combine3(r[1], r[0], -r[0][1]);
		if(r[2][1] != 0)
			row_combine3(r[1], r[2], -r[2][1]);

		// working on third column
		if(r[2][2] == 0)
			return Mat3();					// SINGULAR MATRIX!
		row_mult3(r[2], 1.0f / r[2][2]);
		if(r[0][2] != 0)
			row_combine3(r[2], r[0], -r[0][2]);
		if(r[1][2] != 0)
			row_combine3(r[2], r[1], -r[1][2]);

		return Mat3(
			r[0][3],	r[0][4],	r[0][5],
			r[1][3],	r[1][4],	r[1][5],
			r[2][3],	r[2][4],	r[2][5]
		);
	}




	/*
	 * Mat4 methods
	 */
	Mat4 Mat4::FromQuaternion(const Quaternion& q) { return Mat4::FromMat3(q.ToMat3()); }
	Mat4 Mat4::FromPositionAndOrientation(const Vec3& pos, const Quaternion& ori) { return Mat4::FromPositionAndOrientation(pos, ori.ToMat3()); }
	Mat4 Mat4::FromPosOriScale(const Vec3& pos, const Quaternion& ori, float scale) { return Mat4::FromPosOriScale(pos, ori.ToMat3(), scale); }


	static void row_swap4(float*& r1, float*& r2)
	{
		float* temp = r1;
		r1 = r2;
		r2 = temp;
	}
	static void row_mult4(float* row, float coeff)
	{
		for(int i = 0; i < 8; ++i)
			row[i] *= coeff;
	}
	static void row_combine4(float* from, float* to, float coeff)
	{
		for(int i = 0; i < 8; ++i)
			to[i] += from[i] * coeff;
	}

	Mat4 Mat4::Invert(const Mat4& a)
	{
		const float* v = a.values;
		float ra[] = {	v[0],	v[1],	v[2],	v[3],	1, 0, 0, 0 };
		float rb[] = {	v[4],	v[5],	v[6],	v[7],	0, 1, 0, 0 };
		float rc[] = {	v[8],	v[9],	v[10],	v[11],	0, 0, 1, 0 };
		float rd[] = {	v[12],	v[13],	v[14],	v[15],	0, 0, 0, 1 };

		float *r[] = { ra, rb, rc, rd };

		// working on first column...
		for(int i = 0; i < 3; ++i)
			for(int j = 0; j < 3 - i; ++j)
				if(fabs(r[j][0]) < fabs(r[j + 1][0]))
					row_swap4(r[j], r[j + 1]);
		row_mult4(r[0], 1.0f / r[0][0]);
		if(r[1][0] != 0)
			row_combine4(r[0], r[1], -r[1][0]);
		if(r[2][0] != 0)
			row_combine4(r[0], r[2], -r[2][0]);
		if(r[3][0] != 0)
			row_combine4(r[0], r[3], -r[3][0]);

		// working on second column
		for(int i = 0; i < 2; ++i)
			for(int j = 1; j < 3 - i; ++j)
				if(fabs(r[j][1]) < fabs(r[j + 1][1]))
					row_swap4(r[j], r[j + 1]);
		row_mult4(r[1], 1.0f / r[1][1]);
		if(r[0][1] != 0)
			row_combine4(r[1], r[0], -r[0][1]);
		if(r[2][1] != 0)
			row_combine4(r[1], r[2], -r[2][1]);
		if(r[3][1] != 0)
			row_combine4(r[1], r[3], -r[3][1]);

		// working on third column
		if(fabs(r[2][2]) < fabs(r[3][2]))
			row_swap4(r[2], r[3]);
		row_mult4(r[2], 1.0f / r[2][2]);
		if(r[0][2] != 0)
			row_combine4(r[2], r[0], -r[0][2]);
		if(r[1][2] != 0)
			row_combine4(r[2], r[1], -r[1][2]);
		if(r[3][2] != 0)
			row_combine4(r[2], r[3], -r[3][2]);

		// working on fourth column
		if(r[3][3] == 0)
			return Mat4();					// SINGULAR MATRIX!
		row_mult4(r[3], 1.0f / r[3][3]);
		if(r[0][3] != 0)
			row_combine4(r[3], r[0], -r[0][3]);
		if(r[1][3] != 0)
			row_combine4(r[3], r[1], -r[1][3]);
		if(r[2][3] != 0)
			row_combine4(r[3], r[2], -r[2][3]);

		return Mat4(
			r[0][4],	r[0][5],	r[0][6],	r[0][7],
			r[1][4],	r[1][5],	r[1][6],	r[1][7],
			r[2][4],	r[2][5],	r[2][6],	r[2][7],
			r[3][4],	r[3][5],	r[3][6],	r[3][7]
		);
	}

	void Mat4::Decompose(Vec3& translation, Quaternion& orientation, Vec3& scale) const
	{
		translation = TransformVec3_1(0, 0, 0);
		
		Vec3 x = TransformVec3_0(1, 0, 0), y = TransformVec3_0(0, 1, 0), z = TransformVec3_0(0, 0, 1);

		scale.x = x.ComputeMagnitude();
		scale.y = y.ComputeMagnitude();
		scale.z = z.ComputeMagnitude();

		x /= scale.x;
		y /= scale.y;
		z /= scale.z;

		orientation = Quaternion::FromRotationMatrix(Mat3(x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z));
	}

	void Mat4::Decompose(Vec3& translation, Quaternion& orientation) const
	{
		translation = TransformVec3_1(0, 0, 0);
		
		Vec3 x = TransformVec3_0(1, 0, 0), y = TransformVec3_0(0, 1, 0), z = TransformVec3_0(0, 0, 1);
		orientation = Quaternion::FromRotationMatrix(Mat3(x.x, x.y, x.z, y.x, y.y, y.z, z.x, z.y, z.z));
	}




	/*
	 * Matrix serialization functions
	 */
	void WriteMat3(const Mat3& mat, ostream& stream)
	{
		for(int i = 0; i < 9; ++i)
			WriteSingle(mat.values[i], stream);
	}
	void WriteMat4(const Mat4& mat, ostream& stream)
	{
		for(int i = 0; i < 16; ++i)
			WriteSingle(mat.values[i], stream);
	}

	Mat3 ReadMat3(istream& stream)
	{
		float values[9];
		for(int i = 0; i < 9; ++i)
			values[i] = ReadSingle(stream);
		return Mat3(values);
	}

	Mat4 ReadMat4(istream& stream)
	{
		float values[16];
		for(int i = 0; i < 16; ++i)
			values[i] = ReadSingle(stream);
		return Mat4(values);
	}
}
