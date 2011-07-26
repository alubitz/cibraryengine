#pragma once

#include "StdAfx.h"

#include "Content.h"

#include "MathTypes.h"

namespace CibraryEngine
{
	using namespace std;

	/** Top-level vertex buffer interface */
	class VertexBufferI : public Disposable
	{
		private:

			bool built;

		protected:

			/** Abstract function to build the vertex buffer */
			virtual void Build() = 0;

			virtual void InnerDispose();

			/** Marks the vertex buffer as built */
			void Validate();
			/** Marks the vertex buffer as needing to be built */
			void Invalidate();

			/** Abstract function to clean up any resources left over by the vertex buffer */
			virtual void GLCleanup() = 0;

		public:

			VertexBufferI() : built(false) { }

			/** Abstract function to draw this vertex buffer */
			virtual void Draw() = 0;

			/** Builds the vertex buffer if it needs to be built */
			void BuildAsNeeded();

			/** Returns whether the vertex buffer has been built */
			bool IsValid();
	};

	// vertex buffer template
	/** Template class for vertex buffers containing vertex infos of a specific type */
	template <typename T> class VertexBuffer : public VertexBufferI
	{
		public:

			vector<T> vertex_infos;

			VertexBuffer() : VertexBufferI(), vertex_infos() { }
	};

	/** Class containing a vertex buffer and associatinbg with it a material index... */
	struct MaterialModelPair
	{
		/** The vertex buffer */
		VertexBufferI* vbo;
		/** The material index... I don't think this does anything yet? */
		unsigned int material_index;			// index into an array of materials in the model class

		MaterialModelPair() : vbo(NULL), material_index(0) { }
	};

	/** Template content class for 3d models using a specific type of vertex buffer */
	template <typename T> class Model : public Disposable
	{
		protected:

			/** The vertex buffer */
			T* vbo;

			void InnerDispose() { }

		public:

			/** Initializes a model */
			Model(T* vbo) : vbo(vbo) { }

			/** Returns the vertex buffer, loading the model and generating the vertex buffer if necessary */
			T* GetVBO() { return vbo; }
	};

	class VUVNTTCVertexBuffer;

	/** A model which has VUVNTTC vertex infos */
	class VTNModel : public Model<VUVNTTCVertexBuffer>
	{
		public:

			VTNModel(VUVNTTCVertexBuffer* vbo) : Model<VUVNTTCVertexBuffer>(vbo) { }
	};

	class SkinVInfoVertexBuffer;
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

			static SkinnedModel* CopyVTNModel(VTNModel* model, string material_name);
	};
}
