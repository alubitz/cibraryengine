#include "StdAfx.h"
#include "SceneRenderer.h"
#include "Material.h"
#include "RenderNode.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	// Comparison type for render nodes
	struct RNDistanceComp { bool operator() (RenderNode& lhs, RenderNode& rhs) { return lhs.distance < rhs.distance; } };

	void SceneRenderer::Render()
	{
		BeginRender();

		GLDEBUG();

		RenderOpaque();
		RenderTranslucent();

		GLDEBUG();
	}

	void SceneRenderer::BeginRender()
	{
		// group the models by material
		material_model_lists = map<Material*, vector<RenderNode> >();
		for(vector<RenderNode>::iterator iter = objects.begin(); iter != objects.end(); iter++)
		{
			RenderNode model = *iter;
			if (material_model_lists.find(model.material) == material_model_lists.end())
				material_model_lists[model.material] = vector<RenderNode>();
			material_model_lists[model.material].push_back(model);
		}

		translucent_items = map<Material*, vector<RenderNode> >();
		opaque_items = map<Material*, vector<RenderNode> >();
		for(map<Material*, vector<RenderNode> >::iterator iter = material_model_lists.begin(); iter != material_model_lists.end(); iter++)
		{
			Material* mat = iter->first;

			if (mat->blend_style == Opaque)
				opaque_items[mat] = material_model_lists[mat];
			else
				translucent_items[mat] = material_model_lists[mat];
		}

		sorted_translucent_items = list<RenderNode>();
		for(map<Material*, vector<RenderNode> >::iterator iter = translucent_items.begin(); iter != translucent_items.end(); iter++)
			sorted_translucent_items.insert(sorted_translucent_items.end(), iter->second.begin(), iter->second.end());

		sorted_translucent_items.sort<RNDistanceComp>(RNDistanceComp());
	}

	void SceneRenderer::RenderOpaque()
	{
		GLDEBUG();

		for(map<Material*, vector<RenderNode> >::iterator iter = opaque_items.begin(); iter != opaque_items.end(); iter++)
		{
			Material* mat = iter->first;
			vector<RenderNode>& node_list = iter->second;

			mat->BeginDraw(this);

			for(vector<RenderNode>::iterator jter = node_list.begin(); jter != node_list.end(); jter++)
				mat->Draw(*jter);

			mat->EndDraw();
		}

		GLDEBUG();
	}

	void SceneRenderer::RenderTranslucent()
	{
		GLDEBUG();

		// translucent stuff is sorted, now get to drawing it!
		Material* current_mat = NULL;
		for(list<RenderNode>::iterator iter = sorted_translucent_items.begin(); iter != sorted_translucent_items.end(); iter++)
		{
			RenderNode model = *iter;

			assert (model.material != NULL);

			// only end the current material and start the next if they are DIFFERENT materials
			if (current_mat == NULL || !model.material->Equals(current_mat))
			{
				if (current_mat != NULL)
					current_mat->EndDraw();							// if the previous material was null, that means we're on the first material

				model.material->BeginDraw(this);

				current_mat = model.material;
			}

			current_mat->Draw(model);
		}

		// Finish the draw operation on the currently active material... 
		// If nothing's been drawn there is nothing to do... anyway the material is null
		if(current_mat != NULL)
			current_mat->EndDraw();

		GLDEBUG();
	}

	void SceneRenderer::RenderShadowVolumes(Vec4 origin)
	{
		// TODO: implement this
	}

	void SceneRenderer::Cleanup()
	{
		for(vector<RenderNode>::iterator iter = objects.begin(); iter != objects.end(); iter++)
			iter->material->Cleanup(*iter);

		material_model_lists.clear();
		opaque_items.clear();
		translucent_items.clear();
		sorted_translucent_items.clear();
	}
}
