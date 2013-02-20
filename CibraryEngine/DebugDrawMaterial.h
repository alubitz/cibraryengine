#pragma once

#include "StdAfx.h"

#include "Vector.h"
#include "Material.h"

namespace CibraryEngine
{
	struct DebugDrawMaterialNodeData
	{
		Vec3 p1, p2;
		Vec3 color;

		DebugDrawMaterialNodeData(const Vec3& p1, const Vec3& p2) : p1(p1), p2(p2), color(1.0f, 1.0f, 1.0f) { }
		DebugDrawMaterialNodeData(const Vec3& p1, const Vec3& p2, const Vec3& color) : p1(p1), p2(p2), color(color) { }
	};

	class DebugDrawMaterial : public Material
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			DebugDrawMaterial();

			void BeginDraw(SceneRenderer* renderer);
			void EndDraw();
			void Draw(RenderNode node);

			void Cleanup(RenderNode node);

			bool Equals(Material* other);

			static DebugDrawMaterial* GetDebugDrawMaterial();



			DebugDrawMaterialNodeData* New(const Vec3& p1, const Vec3& p2);
			DebugDrawMaterialNodeData* New(const Vec3& p1, const Vec3& p2, const Vec3& color);

			void EmptyRecycleBin();
	};
}
