#pragma once

#include "StdAfx.h"

#include "Model.h"
#include "VertexInfo.h"
#include "UberModel.h"

namespace CibraryEngine
{
	/** ContentLoader for VTNModel assets */
	struct ModelLoader : public ContentTypeHandler<VTNModel>
	{
		ModelLoader(ContentMan* man);

		VTNModel* Load(ContentMetadata& what);
		void Unload(VTNModel* content, ContentMetadata& meta);
	};

	/** ContentLoader for SkinnedModel assets */
	struct SkinnedModelLoader : public ContentTypeHandler<SkinnedModel>
	{
		SkinnedModelLoader(ContentMan* man);

		SkinnedModel* Load(ContentMetadata& what);
		void Unload(SkinnedModel* content, ContentMetadata& meta);
	};

	/** Loads a .OBJ model file, returning 0 if ok or an int error code otherwise */
	int LoadOBJ(string filename, VUVNTTCVertexBuffer* vbo);

	int SaveOBJ(string filename, VUVNTTCVertexBuffer* vbo);
	int SaveOBJ(string filename, SkinVInfoVertexBuffer* vbo);

	/** Loads a .AAM model file, returning 0 if ok or an int error code otherwise */
	int LoadAAM(string filename, VUVNTTCVertexBuffer* vbo);
	/** Saves a .AAM model file, returning 0 if ok or an int error code otherwise */
	int SaveAAM(string filename, VUVNTTCVertexBuffer* vbo, bool overwrite = true);		// is overwrite even used?

	/** Loads a .AAK skin file, returning 0 if ok or an int error code otherwise */
	int LoadAAK(string filename, vector<MaterialModelPair>& material_model_pairs, vector<string>& material_names, Skeleton*& skeleton);
	/** Saves a .AAK skin file, returning 0 if ok or an int error code otherwise */
	int SaveAAK(string filename, SkinnedModel* model, bool overwrite = true);			// is overwrite even used?

	class KeyframeAnimation;

	/** Loads a .AAA animation file, returning 0 if ok or an int error code otherwise */
	int LoadAAA(string filename, KeyframeAnimation& anim);
	/** Saves a .AAA animation file, returning 0 if ok or an int error code otherwise */
	int SaveAAA(string filename, KeyframeAnimation& anim);
}
