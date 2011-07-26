#pragma once

#include "StdAfx.h"
#include "Content.h"

namespace CibraryEngine
{
	class SceneRenderer;
	class RenderNode;

	/** What kind of blending a Material uses */
	enum BlendStyle
	{
		/** Material is opaque */
		Opaque = 0,
		/** Material uses additive blending */
		Additive,
		/** Material uses alpha-opacity */
		Alpha
	};

	// like an interface
	// the fields should be left mostly-constant once they are initialized
	/** Class representing a material for rendering */
	struct Material : public Disposable
	{
		protected:

			virtual void InnerDispose() { }

		public:

			/** What kind of blending to use */
			BlendStyle blend_style;
			/** Whether lighting is applied to this */
			bool apply_lighting;

			/** A unique identifying integer for this material */
			unsigned int mclass_id;

			Material(unsigned int mclass_id) : mclass_id(mclass_id) { }
			Material(unsigned int mclass_id, BlendStyle blend_style, bool apply_lighting) : blend_style(blend_style), apply_lighting(apply_lighting), mclass_id(mclass_id) { }

			virtual ~Material() { }

			/** Abstract function called when the first of a sequence of RenderNodes using this material is drawn */
			virtual void BeginDraw(SceneRenderer* renderer) = 0;
			/** Abstract function called when the last of a sequence of RenderNodes using this material is drawn */
			virtual void EndDraw() = 0;
			/** Abstract function called when a RenderNode using this material is drawn */
			virtual void Draw(RenderNode node) = 0;

			/** Abstract function to clean up any resources created while drawing stuff */
			virtual void Cleanup(RenderNode node) = 0;

			/** Abstract function to determine whether on material equals another; implementations should check mclass_id! */
			virtual bool Equals(Material* other) = 0;



			// stuff for Content class :|
			void Load(ContentMan* content) { }
			bool IsLoaded() { return true; }
			void RecommendUnload() { }
	};
}