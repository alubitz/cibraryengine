#pragma once

#include "StdAfx.h"
#include "Vector.h"
#include "Quaternion.h"
#include "Sphere.h"

#include "Content.h"

#include "Disposable.h"

namespace CibraryEngine
{
	using namespace std;

	class BinaryChunk;
	struct MaterialModelPair;

	class Skeleton;
	class SkinnedModel;
	class SkinnedCharacter;
	struct SkinnedCharacterRenderInfo;
	
	class Material;
	class SceneRenderer;

	struct Mat4;

	class CollisionShape;

	/** A really awesome model format I came up with */
	class UberModel : public Disposable
	{
		protected:

			vector<Material*> cached_materials;

			void InnerDispose();

		public:

			// first some structs...

			/** A struct compactly storing bone influence data */
			struct CompactBoneInfluence
			{
				unsigned char indices[4];
				unsigned char weights[4];
				CompactBoneInfluence()
				{
					indices[0] = indices[1] = indices[2] = indices[3] = weights[1] = weights[2] = weights[3] = 0;
					weights[0] = 255;
				}
			};

			/** Indices into the vertices, texcoords, and normals arrays */
			struct VTN
			{
				unsigned int v, t, n;
			};

			struct Point
			{
				unsigned int material;
				VTN vtn;
			};

			struct Edge
			{
				unsigned int material;
				VTN a, b;
			};

			struct Triangle
			{
				unsigned int material;
				VTN a, b, c;
			};

			struct Bone
			{
				string name;
				unsigned int parent;
				Vec3 pos;
				Quaternion ori;
			};

			struct Special
			{
				Vec3 pos;
				Vec3 normal;
				float radius;

				unsigned int bone;			// special points can only be in one bone

				string info;
			};




			/** A level-of-detail of an UberModel */
			class LOD : public Disposable
			{
				protected:

					void InnerDispose();

				public:

					string lod_name;

					vector<Vec3> vertices;
					vector<Vec3> texcoords;
					vector<Vec3> normals;

					vector<CompactBoneInfluence> bone_influences;

					vector<Point> points;
					vector<Edge> edges;
					vector<Triangle> triangles;

					vector<MaterialModelPair>* vbos;

					LOD();

					vector<MaterialModelPair>* GetVBOs();
					void InvalidateVBOs();
			};




			vector<LOD*> lods;
			vector<string> materials;
			vector<Bone> bones;
			vector<Special> specials;

			Sphere bounding_sphere;

			UberModel();

			/** Gets a bounding sphere for this model in local coords; skeletal animation might exceed this */
			Sphere GetBoundingSphere();

			Skeleton* CreateSkeleton();

			void LoadCachedMaterials(Cache<Material>* mat_cache, bool force = false);

			void Vis(SceneRenderer* renderer, int lod, const Mat4& xform, SkinnedCharacterRenderInfo* char_render_info);
			void Vis(SceneRenderer* renderer, int lod, const Mat4& xform, SkinnedCharacterRenderInfo* char_render_info, vector<Material*>* use_materials);
	};

	struct UberModelLoader : public ContentTypeHandler<UberModel>
	{
		UberModelLoader(ContentMan* man);

		UberModel* Load(ContentMetadata& what);
		void Unload(UberModel* content, ContentMetadata& meta);

		static unsigned int LoadZZZ(UberModel*& model, const string& filename);
		static unsigned int SaveZZZ(UberModel* model, const string& filename);

		static UberModel* CopySkinnedModel(SkinnedModel* skinny);

		static void AddSkinnedModel(UberModel* uber, SkinnedModel* skinny, const string& lod_name);
	};

	int ba_saveUberModel(lua_State* L);
}
