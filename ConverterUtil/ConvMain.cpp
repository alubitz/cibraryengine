#include "StdAfx.h"

#include "UnrealImport.h"

using namespace std;
using namespace CibraryEngine;
using namespace ConverterUtil;

string GetFileExtension(string filename)
{
	size_t last_dot = filename.find_last_of(".");
	
	if(last_dot == string::npos)
		return string();
	else
	{
		filename = filename.substr(last_dot + 1);

		string result;
		for(unsigned int i = 0; i < filename.length(); i++)
			if(filename[i] >= 'a' && filename[i] <= 'z')
				result += (filename[i] + 'A' - 'a');
			else
				result += filename[i];

		return result;
	}
}

UberModel* model = NULL;

bool changes = false;
bool running = true;

Vec3 min_xyz, max_xyz;
unsigned int num_verts = 0;

void UpdateStatus();

void DisplayStatus()
{
	cout << endl;

	if(num_verts == 0)
		cout << "Model is empty" << endl;
	else
	{
		cout << "Model contains " << num_verts << " vertices" << endl;
		cout << "Bounding box: (" << min_xyz.x << ", " << min_xyz.y << ", " << min_xyz.z << ") - (" << max_xyz.x << ", " << max_xyz.y << ", " << max_xyz.z << ")" << endl;
	}
}

void ShowMainMenu()
{
	cout << endl;
	cout << "Menu:" << endl;
	cout << "---------------------------------------------------" << endl;

	
	if(changes)
	{
		cout << "1. Discard changes" << endl;
		cout << "2. Save model" << endl;
	}
	else if(num_verts > 0)
		cout << "1. New model" << endl;

	if(num_verts > 0)
		cout << "3. Merge model" << endl;
	else
		cout << "3. Open model" << endl;

	if(num_verts > 0)
	{
		cout << "4. Translate model" << endl;
		cout << "5. Scale model" << endl;
		cout << "6. Rotate model" << endl;
	}

	cout << "7. Exit" << endl;
	cout << endl;
}

void DiscardChanges()
{
	if(model != NULL)
	{
		model->Dispose();
		delete model;
		model = NULL;
	}

	changes = false;
	UpdateStatus();
}

void SaveModel()
{
	if(model != NULL)
	{
		cout << "Enter filename to save as: ";

		string input;
		cin >> input;

		string ext = GetFileExtension(input);

		if(ext.compare("ZZZ") == 0)
		{
			unsigned int result = UberModelLoader::SaveZZZ(model, input);
			if(result)
				cout << "ERROR! SaveZZZ returned with status " << result << "!" << endl;
			else
				changes = false;
		}
		else
			cout << "Can't export to that format!" << endl;
	}
}

void ImportModel()
{
	cout << "Enter filename to load: ";

	string input;
	cin >> input;
	
	string ext = GetFileExtension(input);

	if(ext.compare("PSK") == 0)
	{
		SkinnedModel* skinny = new SkinnedModel(vector<MaterialModelPair>(), vector<string>(), new Skeleton());
		if(int load_result = LoadPSK(input, skinny, 1.0f))
			cout << "ERROR! LoadPSK returned with status " << load_result << "!" << endl;
		else
		{
			if(model == NULL)
				model = UberModelLoader::CopySkinnedModel(skinny);
			else
				UberModelLoader::AddSkinnedModel(model, skinny, "");

			changes = true;
			UpdateStatus();
		}
	}
	else if(ext.compare("ZZZ") == 0)
	{
		if(model == NULL)
		{
			if(unsigned int load_result = UberModelLoader::LoadZZZ(model, input))
				cout << "ERROR! LoadZZZ returned with status " << load_result << "!" << endl;
			else
			{
				changes = true;
				UpdateStatus();
			}
		}
		else
			cout << "Can't merge a ZZZ model" << endl;
	}
	else
		cout << "Can't import that type of file" << endl;
}

void UpdateStatus()
{
	num_verts = 0;

	bool first_vert = true;;
		
	if(model != NULL)
	{
		for(unsigned int i = 0; i < model->lods.size(); i++)
		{
			UberModel::LOD* lod = model->lods[i];
			
			vector<MaterialModelPair>* pairs = lod->GetVBOs();

			for(vector<MaterialModelPair>::iterator iter = pairs->begin(); iter != pairs->end(); iter++)
			{
				VertexBuffer* vbo = iter->vbo;

				float* vertex_data = vbo->GetFloatPointer("gl_Vertex");

				unsigned int count = vbo->GetNumVerts();
				num_verts += count;
				
				for(unsigned int j = 0; j < count; j++)
				{
					float x = *(vertex_data++), y = *(vertex_data++), z = *(vertex_data++);

					if(first_vert)
					{
						min_xyz = max_xyz = Vec3(x, y, z);
						first_vert = false;
					}
					else
					{
						min_xyz.x = min(x, min_xyz.x);
						min_xyz.y = min(y, min_xyz.y);
						min_xyz.z = min(z, min_xyz.z);

						max_xyz.x = max(x, max_xyz.x);
						max_xyz.y = max(y, max_xyz.y);
						max_xyz.z = max(z, max_xyz.z);
					}
				}
			}
		}
	}
}

void TranslateModel()
{
	string input;

	cout << "Enter translation vector... " << endl;

	cout << "x = ";
	cin >> input;
	float x = (float)atof(input.c_str());

	cout << "y = ";
	cin >> input;
	float y = (float)atof(input.c_str());

	cout << "z = ";
	cin >> input;
	float z = (float)atof(input.c_str());

	Vec3 translation = Vec3(x, y, z);

	if(model != NULL)
	{
		for(unsigned int i = 0; i < model->lods.size(); i++)
		{
			UberModel::LOD* lod = model->lods[i];

			for(vector<Vec3>::iterator iter = lod->vertices.begin(); iter != lod->vertices.end(); iter++)
				*iter += translation;

			lod->InvalidateVBOs();
		}

		changes = true;
	}

	UpdateStatus();
}

void ScaleModel() 
{ 
	string input;

	cout << "Enter scale factors... " << endl;

	cout << "x = ";
	cin >> input;
	float x = (float)atof(input.c_str());

	cout << "y = ";
	cin >> input;
	float y = (float)atof(input.c_str());

	cout << "z = ";
	cin >> input;
	float z = (float)atof(input.c_str());

	if(model != NULL)
	{
		for(unsigned int i = 0; i < model->lods.size(); i++)
		{
			UberModel::LOD* lod = model->lods[i];

			for(vector<Vec3>::iterator iter = lod->vertices.begin(); iter != lod->vertices.end(); iter++)
			{
				iter->x *= x;
				iter->y *= y;
				iter->z *= z;
			}

			lod->InvalidateVBOs();
		}

		changes = true;
	}

	UpdateStatus();
}

void RotateModel() 
{
	string input;

	cout << "Enter axis of rotation (magnitude = angle)... " << endl;

	cout << "x = ";
	cin >> input;
	float x = (float)atof(input.c_str());

	cout << "y = ";
	cin >> input;
	float y = (float)atof(input.c_str());

	cout << "z = ";
	cin >> input;
	float z = (float)atof(input.c_str());

	Mat3 rm = Mat3::FromScaledAxis(x, y, z);

	if(model != NULL)
	{
		for(unsigned int i = 0; i < model->lods.size(); i++)
		{
			UberModel::LOD* lod = model->lods[i];

			for(vector<Vec3>::iterator iter = lod->vertices.begin(); iter != lod->vertices.end(); iter++)
				*iter = rm * *iter;

			lod->InvalidateVBOs();
		}

		changes = true;
	}

	UpdateStatus();
}

void Exit() { running = false; }

int main(int argc, char** argv)
{
	InitEndianness();

	while(running)
	{
		DisplayStatus();
		ShowMainMenu();

		string input;
		cin >> input;

		int i = atoi(input.c_str());
		switch(i)
		{
		case 1:
			DiscardChanges();
			break;

		case 2:
			if(changes)
				SaveModel();
			break;

		case 3:
			ImportModel();
			break;

		case 4:
			TranslateModel();
			break;

		case 5:
			if(num_verts > 0)
				ScaleModel();
			break;

		case 6:
			if(num_verts > 0)
				RotateModel();
			break;

		case 7:
			Exit();
			break;

		default:
			cout << "That's not a valid action" << endl;
			break;
		}
	}
	return 0;
}

#ifdef WIN32
int WINAPI WinMain(	HINSTANCE	hInstance,		// Instance
					HINSTANCE	hPrevInstance,	// Previous Instance
					LPSTR		lpCmdLine,		// Command Line Parameters
					int			nCmdShow)		// Window Show State
{
	return main(nCmdShow, &lpCmdLine);
}
#endif