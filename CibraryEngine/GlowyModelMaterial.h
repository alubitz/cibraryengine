#pragma once

#include "StdAfx.h"

#include "Matrix.h"
#include "Material.h"

namespace CibraryEngine
{
	struct VertexBuffer;

	class Texture2D;
	class Texture3D;
	class ShaderProgram;

	struct GlowyModelMaterialNodeData
	{
		VertexBuffer* model;
		Mat4 xform;

		GlowyModelMaterialNodeData(VertexBuffer* model, const Mat4& xform);

		void Draw();
	};

	class GlowyModelMaterial : public Material
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			GlowyModelMaterial(Texture2D* texture, ShaderProgram* shader);
			GlowyModelMaterial(Texture3D* texture, ShaderProgram* shader);

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(const RenderNode& node);

			void Cleanup(const RenderNode& node);

			bool Equals(const Material* material) const;
	};
}
