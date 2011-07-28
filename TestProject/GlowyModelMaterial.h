#pragma once

#include "../CibraryEngine/DebugLog.h"

#include "../CibraryEngine/MathTypes.h"
#include "../CibraryEngine/GraphicsTypes.h"

namespace Test
{
	using namespace CibraryEngine;

	struct GlowyModelMaterialNodeData
	{
		VertexBufferI* model;
		Mat4 xform;

		GlowyModelMaterialNodeData(VertexBufferI* model, Mat4 xform);

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
			void Draw(RenderNode node);

			void Cleanup(RenderNode node);

			bool Equals(Material* material);
	};
}
