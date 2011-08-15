#pragma once

#include "StdAfx.h"
#include "Disposable.h"
#include "MathTypes.h"
#include "VertexBuffer.h"

namespace CibraryEngine
{
	using namespace std;

	/** Class containing a vertex buffer and associatinbg with it a material index... */
	struct MaterialModelPair
	{
		/** The vertex buffer */
		VertexBuffer* vbo;
		/** The material index... I don't think this does anything yet? */
		unsigned int material_index;			// index into an array of materials in the model class

		MaterialModelPair() : vbo(NULL), material_index(0) { }
	};

	class Skeleton;

	/** A model for skeletal animation purposes, which uses SkinVInfo vertex infos */
	class SkinnedModel : public Disposable
	{
		protected:

			void InnerDispose() { }

		public:

			/** Collection of material model pairs within this model */
			vector<MaterialModelPair> material_model_pairs;
			/** Collection of material names */
			vector<string> material_names;

			/** Skeleton prototype for this model */
			Skeleton* skeleton;

			SkinnedModel(vector<MaterialModelPair> material_model_pairs, vector<string> material_names, Skeleton* skeleton);

			static SkinnedModel* WrapVertexBuffer(VertexBuffer* model, string material_name);
	};

	struct VTNTT
	{
		Vec3 x;
        Vec3 uvw;
        Vec3 n;
        Vec3 tan_1; 
        Vec3 tan_2;

		VTNTT() { }
		VTNTT(Vec3 x, Vec3 uvw, Vec3 n) : x(x), uvw(uvw), n(n) { }

	};

	struct SkinVInfo : public VTNTT
	{
		unsigned char indices[4];
        unsigned char weights[4];

		SkinVInfo();
		SkinVInfo(Vec3 x, Vec3 uvw, Vec3 n);
		SkinVInfo(VTNTT original);
		SkinVInfo(Vec3 x, Vec3 uvw, Vec3 n, unsigned char* indices, unsigned char* weights);
	};

	void AddTriangleVertexInfo(VertexBuffer* vbo, VTNTT a, VTNTT b, VTNTT c);
	void AddTriangleVertexInfo(VertexBuffer* vbo, SkinVInfo a, SkinVInfo b, SkinVInfo c);
	VTNTT GetVTNTT(VertexBuffer* vbo, int index);
	SkinVInfo GetSkinVInfo(VertexBuffer* vbo, int index);
}
