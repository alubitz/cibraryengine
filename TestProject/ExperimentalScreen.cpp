#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "../CibraryEngine/HardwareAcceleratedComputation.h"

namespace Test
{
	/*
	 * ExperimentalScreen implementation
	 */
	struct ExperimentalScreen::Imp
	{
		HardwareAcceleratedComputation* comp;
		VertexBuffer* input_vbo;
		VertexBuffer* output_vbo;

		struct BackButton : public AutoMenuItem
		{
			BackButton(ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		vector<AutoMenuItem*> auto_menu_items;

		Imp(ExperimentalScreen* menu) : comp(NULL), input_vbo(NULL), output_vbo(NULL), auto_menu_items()
		{
			ContentMan* content = menu->content;

			int index = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Experimental Screen", index++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------",	index++, false));
			auto_menu_items.push_back(new BackButton(content, index));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				menu->AddItem(auto_menu_items[i]);

			Shader* shader = content->GetCache<Shader>()->Load("hardware_comp-v");

			vector<const GLchar*> varying_names;
			varying_names.push_back("gl_Position");
			varying_names.push_back("derp");

			input_vbo = new VertexBuffer(Points);
			input_vbo->AddAttribute("gl_Vertex", Float, 4);
			input_vbo->SetNumVerts(7);
			float* vert_ptr = input_vbo->GetFloatPointer("gl_Vertex");
			float positions[] =
			{
				1,	2,	3,	4,
				5,	6,	7,	8,
				9,	0,	1,	2,
				7,	7,	4,	4,
				9,	9,	9,	1,
				0,	0,	0,	12,
				8,	7,	6,	5
			};

			for(float *read_ptr = positions, *write_ptr = vert_ptr, *write_end = vert_ptr + 4 * input_vbo->GetNumVerts(); write_ptr != write_end; ++read_ptr, ++write_ptr)
				*write_ptr = *read_ptr;

			input_vbo->BuildVBO();

			output_vbo = new VertexBuffer(Points);
			output_vbo->AddAttribute("gl_Position",	Float, 4);
			output_vbo->AddAttribute("derp",		Float, 4);

			comp = new HardwareAcceleratedComputation(shader, map<string, string>(), output_vbo);
		}

		void Destroy()
		{
			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
			{
				auto_menu_items[i]->Dispose();
				delete auto_menu_items[i];
			}
			auto_menu_items.clear();

			if(comp)		{ delete comp;			comp = NULL; }
			if(input_vbo)	{ delete input_vbo;		input_vbo = NULL; }
			if(output_vbo)	{ delete output_vbo;	output_vbo = NULL; }
		}

		void Process()
		{
			comp->Process(input_vbo, output_vbo);
			output_vbo->UpdateDataFromGL();

			unsigned int num_verts = output_vbo->GetNumVerts();
			Debug(((stringstream&)(stringstream() << "Got data for " << num_verts << " verts!\n")).str());

			vector<string> attrib_names = output_vbo->GetAttributes();
			for(unsigned int i = 0; i < attrib_names.size(); ++i)
			{
				string attrib_name = attrib_names[i];

				float* data = output_vbo->GetFloatPointer(attrib_name);
				unsigned int num_floats = num_verts * output_vbo->GetAttribNPerVertex(attrib_name);
				for(unsigned int j = 0; j < num_floats; ++j)
				{
					float datum = data[j];
					switch(j % 4)
					{
						case 0:
							Debug(((stringstream&)(stringstream() << '\t' << attrib_name << "[" << j / 4 << "] = (" << datum << ", ")).str());
							break;

						case 1:
						case 2:
							Debug(((stringstream&)(stringstream() << datum << ", ")).str());
							break;

						case 3:
							Debug(((stringstream&)(stringstream() << datum << ")" << endl)).str());
							break;
					}
				}
			}
		}
	};




	/*
	 * ExperimentalScreen methods
	 */
	ExperimentalScreen::ExperimentalScreen(ProgramWindow* win, ProgramScreen* previous) : MenuScreen(win, previous), imp(NULL) { }

	void ExperimentalScreen::Activate()
	{
		MenuScreen::Activate();

		if(!imp)
			imp = new Imp(this);
	}

	void ExperimentalScreen::Deactivate()
	{
		MenuScreen::Deactivate();

		if(imp) { imp->Destroy(); delete imp; imp = NULL; }
	}

	void ExperimentalScreen::Draw(int width, int height)
	{
		MenuScreen::Draw(width, height);

		imp->Process();
	}
}
