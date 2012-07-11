#pragma once

#include "StdAfx.h"

#include "Material.h"
#include "Matrix.h"
#include "Sphere.h"

namespace CibraryEngine
{
	struct VertexBuffer;

	class Texture1D;
	class Texture2D;
	class ShaderProgram;

	class SceneRenderer;

	struct DSNMaterialNodeData
	{
		VertexBuffer* model;
		Mat4 xform;

		Sphere bs;

		Texture1D* bone_matrices;
		int bone_count;
		float precision;

		DSNMaterialNodeData(VertexBuffer* model, Mat4 xform, Sphere bs);
		DSNMaterialNodeData(VertexBuffer* model, Mat4 xform, Sphere bs, Texture1D* bone_matrices, int bone_count, float precision = 4096.0f);

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
			ShaderProgram* shadow_shader;

			Texture2D* diffuse;
			Texture2D* specular;
			Texture2D* normal;

			BlendStyle blend_style;

			DSNMaterial(ShaderProgram* shader, ShaderProgram* shadow_shader, Texture2D* diffuse, Texture2D* specular, Texture2D* normal, BlendStyle blend_style);

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(RenderNode node);

			void Cleanup(RenderNode node);

			bool Equals(Material* material);

			static Texture1D* default_bone_matrices;
	};
}
