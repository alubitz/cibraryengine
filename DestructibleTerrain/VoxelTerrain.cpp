#include "StdAfx.h"

#include "VoxelTerrain.h"
#include "VoxelMaterial.h"

namespace DestructibleTerrain
{
	using namespace CibraryEngine;

	/*
	 * TerrainLeaf methods
	 */
	TerrainLeaf::TerrainLeaf() : value() { }
	TerrainLeaf::TerrainLeaf(float value) : value(value) { }




	/*
	 * VoxelTerrain methods
	 */
	VoxelTerrain::VoxelTerrain(VoxelMaterial* material, int dim_x, int dim_y, int dim_z) :
		data(),
		material(material),
		model(NULL)
	{
		dim[0] = dim_x;
		dim[1] = dim_y;
		dim[2] = dim_z;

		min_xyz = Vec3(-1.0f, -1.0f, -1.0f);
		step_xyz = Vec3(2.0f / (dim[0] - 1), 2.0f / (dim[1] - 1), 2.0f / (dim[2] - 1));

		for(int x = 0; x < dim[0]; x++)
			for(int y = 0; y < dim[1]; y++)
				for(int z = 0; z < dim[2]; z++)
				{
					data.push_back(TerrainLeaf(Random3D::Rand(-1, 1)));
				}
	}

	VoxelTerrain::~VoxelTerrain()
	{
		if(model != NULL)
		{
			model->Dispose(); 
			delete model; 
			model = NULL;
		}
	}

	TerrainLeaf& VoxelTerrain::Element(int x, int y, int z) { return data[x * dim[1] * dim[2] + y * dim[2] + z]; }

	void VoxelTerrain::Vis(SceneRenderer *renderer)
	{
		if(model == NULL)
		{
			model = new VertexBuffer(Triangles);

			model->AddAttribute("gl_Position", Float, 4);
			model->AddAttribute("gl_Color", Float, 4);

			for(int x = 1; x < dim[0]; x++)
				for(int y = 1; y < dim[1]; y++)
					for(int z = 1; z < dim[2]; z++)
					{
						float vert_values[] =
						{
							Element(x - 1, y - 1, z - 1).value,	
							Element(x - 1, y - 1, z).value,
							Element(x - 1, y, z - 1).value,	
							Element(x - 1, y, z).value,
							Element(x, y - 1, z - 1).value,	
							Element(x, y - 1, z).value,
							Element(x, y, z - 1).value,	
							Element(x, y, z).value
						};

						float edge_splits[12];
						for(int i = 0; i < 12; i++)
							edge_splits[i] = -1;

						unsigned short edge_split_mask = 0;

						// don't want to have to pass vert_vlaues and edge_splits to this function
						struct
						{
							float* edge_splits;
							float* vert_values;
							unsigned short* edge_split_mask;

							void operator() (int v1, int v2, int edge) 
							{
								if(vert_values[v1] * vert_values[v2] < 0)
								{
									edge_splits[edge] = -vert_values[v1] / (vert_values[v2] - vert_values[v1]);

									stringstream ss;
									ss << "found a zero on some edge; t = " << edge_splits[edge] << endl;
									Debug(ss.str());

									*edge_split_mask |= (0x1 << edge);
								}
								else
									Debug("didn't find a zero on some edge\n");
							}
						} find_zero;
						find_zero.edge_splits = edge_splits;
						find_zero.vert_values = vert_values;
						find_zero.edge_split_mask = &edge_split_mask;

						find_zero(0, 1, 0);
						find_zero(0, 2, 1);
						find_zero(0, 4, 2);
						find_zero(1, 3, 3);
						find_zero(1, 5, 4);
						find_zero(2, 3, 5);
						find_zero(2, 6, 6);
						find_zero(3, 7, 9);
						find_zero(4, 5, 7);
						find_zero(4, 6, 8);
						find_zero(5, 7, 10);
						find_zero(6, 7, 11);

						int total = 0;
						int cur = edge_split_mask;
						while(cur != 0)
						{
							if(cur & 0x1)
								total++;
							cur >>= 1;
						}

						stringstream ss;
						ss << "Edge split mask = " << edge_split_mask << "; total edges = " << total << endl;
						Debug(ss.str());
					}
		}

		renderer->objects.push_back(RenderNode(material, new VoxelMaterialNodeData(model, Mat4::Scale(step_xyz) * Mat4::Translation(min_xyz)), 0));
	}
}
