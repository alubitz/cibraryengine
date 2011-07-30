#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace CibraryEngine;

	struct DSNMaterialNodeData
	{
		SkinVInfoVertexBuffer* model;
		Mat4 xform;

		Sphere bs;

		Texture1D* bone_matrices;
		int bone_count;

		DSNMaterialNodeData(SkinVInfoVertexBuffer* model, Mat4 xform, Sphere bs);
		DSNMaterialNodeData(SkinVInfoVertexBuffer* model, Mat4 xform, Sphere bs, Texture1D* bone_matrices, int bone_count);

		void Draw();
	};

	class DSNMaterial : public Material
	{
		private:

			vector<DSNMaterialNodeData*> node_data;
			SceneRenderer* scene;

		protected:

			void InnerDispose();

		public:

			ShaderProgram* shader;

			Texture2D* diffuse;
			Texture2D* specular;
			Texture2D* normal;

			DSNMaterial(ShaderProgram* shader, Texture2D* diffuse, Texture2D* specular, Texture2D* normal);

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(RenderNode node);

			void Cleanup(RenderNode node);

			bool Equals(Material* material);

			static Texture1D* default_bone_matrices;
	};
}
