#pragma once

namespace CibraryEngine
{
	class Material;

	// plain data; no inheritance to mess with
	/** Data type for quasi-atomic things to be drawn in the scene */
	class RenderNode
	{
		public:

			/** Material this RenderNode is to be drawn with */
			Material*	material;
			/** Pointer to data the Material will use to figure out how to draw this RenderNode */
			void*		data;
			/** Distance of the relevant drawn objects, for sorting */
			float		distance;

			RenderNode(Material* material, void* data, float distance) : material(material), data(data), distance(distance) { }
	};
}