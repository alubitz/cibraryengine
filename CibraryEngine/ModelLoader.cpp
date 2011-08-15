#include "StdAfx.h"
#include "ModelLoader.h"

#include "SkeletalAnimation.h"
#include "KeyframeAnimation.h"

#include "Serialize.h"
#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * ModelLoader methods
	 */
	ModelLoader::ModelLoader(ContentMan* man) : ContentTypeHandler<VertexBuffer>(man) { }

	VertexBuffer* ModelLoader::Load(ContentMetadata& what)
	{
		string model_name = what.name;

		VertexBuffer* vbo = new VertexBuffer(Triangles);
		vbo->AddAttribute("gl_Vertex", Float, 3);
		vbo->AddAttribute("gl_Normal", Float, 3);
		vbo->AddAttribute("gl_MultiTexCoord0", Float, 3);
		vbo->AddAttribute("gl_MultiTexCoord1", Float, 3);
		vbo->AddAttribute("gl_MultiTexCoord2", Float, 3);

		int aam_result = LoadAAM("Files/Models/" + model_name + ".aam", vbo);
		if(aam_result != 0)
		{
			stringstream aam_msg;
			aam_msg << "LoadAAM (" << model_name << ") returned with status " << aam_result << "; falling back to OBJ" << endl;
			Debug(aam_msg.str());

			int obj_result = LoadOBJ("Files/Models/" + model_name + ".obj", vbo);
			if(obj_result != 0)
			{
				stringstream obj_msg;
				obj_msg << "...And then LoadOBJ returned with status " << obj_result << "!" << endl;
				Debug(obj_msg.str());

				vbo->Dispose();
				delete vbo;

				return NULL;
			}
			else
			{
				SaveAAM("Files/Models/" + model_name + ".aam", vbo, true);

				Debug("Successfully created AAM from OBJ\n");
			}
		}

		// if we got here it means we didn't fail too many times...
		return vbo;
	}

	void ModelLoader::Unload(VertexBuffer* content, ContentMetadata& meta)
	{
		content->Dispose();
		delete content;
	}




	/*
	 * SkinnedModelLoader methods
	 */
	SkinnedModelLoader::SkinnedModelLoader(ContentMan* man) : ContentTypeHandler<SkinnedModel>(man) { }

	SkinnedModel* SkinnedModelLoader::Load(ContentMetadata& what)
	{
		string model_name = what.name;

		vector<MaterialModelPair> material_model_pairs;
		vector<string> material_names;
		Skeleton* skeleton = new Skeleton();

		int aak_result = LoadAAK("Files/Models/" + model_name + ".aak", material_model_pairs, material_names, skeleton);
		if(aak_result != 0)
		{
			stringstream aak_msg;
			aak_msg << "LoadAAK returned with status " << aak_result << endl;
			Debug(aak_msg.str());

			skeleton->Dispose();
			delete skeleton;

			return NULL;
		}

		// if we got here it means we didn't fail
		return new SkinnedModel(material_model_pairs, material_names, skeleton);
	}

	void SkinnedModelLoader::Unload(SkinnedModel* content, ContentMetadata& meta)
	{
		content->Dispose();
		delete content;
	}




	/*
	 * Loader for OBJ models
	 */
	int LoadOBJ(string filename, VertexBuffer* vbo)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);			// not binary --> size gets reported incorrectly
		if(!file)
			return 1;

		//get filesize
		streamsize size = 0;
		if(file.seekg(0, ios::end).good()) size = file.tellg();
		if(file.seekg(0, ios::beg).good()) size -= file.tellg();

		vector<char> buffer = vector<char>();
		//read contents of the file into the vector
		buffer.resize(size_t(size));
		if(size > 0) file.read((char*)(&buffer[0]), size);

		file.close();

		string str = "";
		for(unsigned int i = 0; i < buffer.size(); i++)
			str += buffer[i];

		vector<string> lines = vector<string>();
		unsigned int line_finder_pos = 0;

		vector<string> vertices = vector<string>();
		vector<string> normals = vector<string>();
		vector<string> texcoords = vector<string>();
		vector<string> triangles = vector<string>();

		while (line_finder_pos < str.length())
		{
			char c = str[line_finder_pos];
			if (c == '\n' || c == '\r')
				line_finder_pos++;
			else
			{
				int next_a = str.find('\n', line_finder_pos);
				int next_b = str.find('\r', line_finder_pos);
				if (next_a != -1 || next_b != -1)
				{
					int next = next_a == -1 ? next_b : next_b == -1 ? next_a : min(next_a, next_b);
					lines.push_back(str.substr(line_finder_pos, next - line_finder_pos));
					line_finder_pos = next + 1;
				}
				else
				{
					lines.push_back(str.substr(line_finder_pos));
					line_finder_pos = str.length();
				}
			}
		}
		for(unsigned int line_num = 0; line_num < lines.size(); line_num++)
		{
			string line = lines[line_num];
			switch ((char)line[0])
			{
				case 'v':
					switch ((char)line[1])
					{
						case 't':
							texcoords.push_back(line.substr(line.find(' ', 0) + 1));
							break;
						case 'n':
							normals.push_back(line.substr(line.find(' ', 0) + 1));
							break;
						default:
							vertices.push_back(line.substr(line.find(' ', 0) + 1));
							break;
					}
					break;
				case 'f':
					triangles.push_back(line.substr(line.find(' ', 0) + 1));
					break;
				default:
					break;		// entry doesn't matter, so don't try to handle it
			}
		}
		vector<Vec3> xyz = vector<Vec3>(), uv = vector<Vec3>(), nxyz = vector<Vec3>();

		float x, y, z, u, v, nx, ny, nz;
		unsigned int i;
		for (i = 0; i < vertices.size(); i++)
		{
			string line = vertices[i];
			int start = 0, end = line.find(' ', start);
			x = (float)atof(line.substr(start, end - start).c_str());
			start = end + 1;
			end = line.find(' ', start);
			end = end != -1 ? end : line.length();
			y = (float)atof(line.substr(start, end - start).c_str());
			start = end + 1;
			end = line.find(' ', start);
			end = end != -1 ? end : line.length();
			z = (float)atof(line.substr(start, end - start).c_str());
			xyz.push_back(Vec3(x, y, z));
		}
		for (i = 0; i < texcoords.size(); i++)
		{
			string line = texcoords[i];
			int start = 0, end = line.find(' ', start);
			u = (float)atof(line.substr(start, end - start).c_str());
			start = end + 1;
			end = line.find(' ', start);
			end = end != -1 ? end : line.length();
			v = (float)atof(line.substr(start, end - start).c_str());
			uv.push_back(Vec3(u, v, 0.0f));
		}
		for (i = 0; i < normals.size(); i++)
		{
			string line = normals[i];
			int start = 0, end = line.find(' ', start);
			nx = (float)atof(line.substr(start, end - start).c_str());
			start = end + 1;
			end = line.find(' ', start);
			end = end != -1 ? end : line.length();
			ny = (float)atof(line.substr(start, end - start).c_str());
			start = end + 1;
			end = line.find(' ', start);
			end = end != -1 ? end : line.length();
			nz = (float)atof(line.substr(start, end - start).c_str());
			nxyz.push_back(Vec3(nx, ny, nz));
		}
		int xyz_index, uv_index, n_index;
		for (i = 0; i < triangles.size(); i++)
		{
			string line = triangles[i];

			int start = 0, end = line.find('/', start);
			xyz_index = atoi(line.substr(start, end - start).c_str()) - 1;
			start = end + 1;
			end = line.find('/', start);
			end = end != -1 ? end : line.length();
			uv_index = atoi(line.substr(start, end - start).c_str()) - 1;
			start = end + 1;
			end = line.find(' ', start);
			end = end != -1 ? end : line.length();
			n_index = atoi(line.substr(start, end - start).c_str()) - 1;
			VTNTT v_a = VTNTT(xyz[xyz_index], uv[uv_index], nxyz[n_index]); 

			start = end + 1;
			end = line.find('/', start);
			end = end != -1 ? end : line.length();
			xyz_index = atoi(line.substr(start, end - start).c_str()) - 1;
			start = end + 1;
			end = line.find('/', start);
			end = end != -1 ? end : line.length();
			uv_index = atoi(line.substr(start, end - start).c_str()) - 1;
			start = end + 1;
			end = line.find(' ', start);
			end = end != -1 ? end : line.length();
			n_index = atoi(line.substr(start, end - start).c_str()) - 1;
			VTNTT v_b = VTNTT(xyz[xyz_index], uv[uv_index], nxyz[n_index]);

			start = end + 1;
			end = line.find('/', start);
			end = end != -1 ? end : line.length();
			xyz_index = atoi(line.substr(start, end - start).c_str()) - 1;
			start = end + 1;
			end = line.find('/', start);
			end = end != -1 ? end : line.length();
			uv_index = atoi(line.substr(start, end - start).c_str()) - 1;
			start = end + 1;
			end = line.find(' ', start);
			end = end != -1 ? end : line.length();
			n_index = atoi(line.substr(start, end - start).c_str()) - 1;
			VTNTT v_c = VTNTT(xyz[xyz_index], uv[uv_index], nxyz[n_index]);

			AddTriangleVertexInfo(vbo, v_a, v_b, v_c);
		}

		return 0;
	}




	/*
	 * Saver for OBJ models
	 */
	int SaveOBJ(string filename, VertexBuffer* vbo)
	{
		ofstream file(filename.c_str(), ios::out | ios::binary);
		if(!file)
			return 1;

		// figuring out how to compress stuff
		vector<Vec3> vertices = vector<Vec3>();
		vector<Vec2> texcoords = vector<Vec2>();
		vector<Vec3> normals = vector<Vec3>();
		vector<unsigned int> vertex_indices = vector<unsigned int>();
		vector<unsigned int> texcoord_indices = vector<unsigned int>();
		vector<unsigned int> normal_indices = vector<unsigned int>();

		for(unsigned int iter = 0; iter < vbo->GetNumVerts(); iter++)
		{
			VTNTT v = GetVTNTT(vbo, iter);
			unsigned int i;

			for(i = 0; i < vertices.size(); i++)
				if(vertices[i] == v.x)
					break;
			if(i == vertices.size())
				vertices.push_back(v.x);
			vertex_indices.push_back(i);

			Vec2 uv = Vec2(v.uvw.x, v.uvw.y);
			for(i = 0; i < texcoords.size(); i++)
				if(texcoords[i] == uv)
					break;
			if(i == texcoords.size())
				texcoords.push_back(uv);
			texcoord_indices.push_back(i);

			for(i = 0; i < normals.size(); i++)
				if(normals[i] == v.n)
					break;
			if(i == normals.size())
				normals.push_back(v.n);
			normal_indices.push_back(i);
		}

		file << "o Main" << endl;

		for(vector<Vec3>::iterator v = vertices.begin(); v != vertices.end(); v++)
			file << "v " << v->x << " " << v->y << " " << v->z << endl;
		for(vector<Vec2>::iterator v = texcoords.begin(); v != texcoords.end(); v++)
			file << "vt " << v->x << " " << v->y << endl;
		for(vector<Vec3>::iterator v = normals.begin(); v != normals.end(); v++)
			file << "n " << v->x << " " << v->y << " " << v->z << endl;

		file << "g Main_main" << endl;
		file << "s 1" << endl;
		for(unsigned int i = 0; i < vertex_indices.size();)
		{
			file << "f";
			for(int j = 0; j < 3; i++, j++)
				file << " " << vertex_indices[i] + 1 << "/" << texcoord_indices[i] + 1 << "/" << normal_indices[i] + 1;
			file << endl;
		}

		return 0;
	}



	/*
	 * Loader for AAM models
	 */
	int LoadAAM(string filename, VertexBuffer* vbo)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		// loading the compressed form of the data
		vector<Vec3> vertices = vector<Vec3>();
		vector<Vec3> texcoords = vector<Vec3>();
		vector<Vec3> normals = vector<Vec3>();

		unsigned int vertex_count = ReadUInt32(file);
		for(unsigned int i = 0; i < vertex_count; i++)
		{
			float x = ReadSingle(file);
			float y = ReadSingle(file);
			float z = ReadSingle(file);
			vertices.push_back(Vec3(x, y, z));
		}

		unsigned int texcoord_count = ReadUInt32(file);
		for(unsigned int i = 0; i < texcoord_count; i++)
		{
			float u = ReadSingle(file);
			float v = ReadSingle(file);
			texcoords.push_back(Vec3(u, v, 0));						// third texcoord always 0
		}

		unsigned int normal_count = ReadUInt32(file);
		for(unsigned int i = 0; i < normal_count; i++)
		{
			float x = ReadSingle(file);
			float y = ReadSingle(file);
			float z = ReadSingle(file);
			normals.push_back(Vec3(x, y, z));
		}

		// load and apply the decompression indices
		unsigned int vinfo_count = ReadUInt32(file);
		VTNTT verts[3];
		int target_vert = 0;
		for(unsigned int i = 0; i < vinfo_count; i++)
		{
			unsigned int x_index = ReadUInt32(file);
			unsigned int uv_index = ReadUInt32(file);
			unsigned int n_index = ReadUInt32(file);

			verts[target_vert++] = VTNTT(vertices[x_index], texcoords[uv_index], normals[n_index]);

			if(target_vert == 3)
			{
				AddTriangleVertexInfo(vbo, verts[0], verts[1], verts[2]);
				target_vert = 0;
			}
		}

		file.close();

		// success!
		return 0;
	}




	/*
	 * Saver for AAM models
	 */
	int SaveAAM(string filename, VertexBuffer* vbo, bool overwrite)
	{
		ofstream file(filename.c_str(), ios::out | ios::binary);
		if(!file)
			return 1;

		// figuring out how to compress stuff
		vector<Vec3> vertices = vector<Vec3>();
		vector<Vec2> texcoords = vector<Vec2>();
		vector<Vec3> normals = vector<Vec3>();
		vector<unsigned int> vertex_indices = vector<unsigned int>();
		vector<unsigned int> texcoord_indices = vector<unsigned int>();
		vector<unsigned int> normal_indices = vector<unsigned int>();

		for(unsigned int iter = 0; iter < vbo->GetNumVerts(); iter++)
		{
			VTNTT v = GetVTNTT(vbo, iter);
			unsigned int i;

			for(i = 0; i < vertices.size(); i++)
				if(vertices[i] == v.x)
					break;
			if(i == vertices.size())
				vertices.push_back(v.x);
			vertex_indices.push_back(i);

			Vec2 uv = Vec2(v.uvw.x, v.uvw.y);
			for(i = 0; i < texcoords.size(); i++)
				if(texcoords[i] == uv)
					break;
			if(i == texcoords.size())
				texcoords.push_back(uv);
			texcoord_indices.push_back(i);

			for(i = 0; i < normals.size(); i++)
				if(normals[i] == v.n)
					break;
			if(i == normals.size())
				normals.push_back(v.n);
			normal_indices.push_back(i);
		}

		// the actual save operations
		unsigned int vertex_count = vertices.size();
		WriteUInt32(vertex_count, file);
		for(unsigned int i = 0; i < vertex_count; i++)
		{
			Vec3 vert = vertices[i];
			WriteSingle(vert.x, file);
			WriteSingle(vert.y, file);
			WriteSingle(vert.z, file);
		}

		unsigned int texcoord_count = texcoords.size();
		WriteUInt32(texcoord_count, file);
		for(unsigned int i = 0; i < texcoord_count; i++)
		{
			Vec2 uv = texcoords[i];
			WriteSingle(uv.x, file);
			WriteSingle(uv.y, file);
		}

		unsigned int normal_count = normals.size();
		WriteUInt32(normal_count, file);
		for(unsigned int i = 0; i < normal_count; i++)
		{
			Vec3 norm = normals[i];
			WriteSingle(norm.x, file);
			WriteSingle(norm.y, file);
			WriteSingle(norm.z, file);
		}

		unsigned int vinfo_count = vbo->GetNumVerts();
		WriteUInt32(vinfo_count, file);
		for(unsigned int i = 0; i < vinfo_count; i++)
		{
			WriteUInt32(vertex_indices[i], file);
			WriteUInt32(texcoord_indices[i], file);
			WriteUInt32(normal_indices[i], file);
		}

		file.close();

		return 0;
	}




	/*
	 * Loader for AAK files
	 */
	int LoadAAK(string filename, vector<MaterialModelPair>& material_model_pairs, vector<string>& material_names, Skeleton*& skeleton)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		unsigned int materials_count = ReadUInt32(file);
		for(unsigned int i = 0; i < materials_count; i++)
		{
			unsigned int mat_name_len = ReadByte(file);
			string mat_name = "";
			for(unsigned int j = 0; j < mat_name_len; j++)
				mat_name += ReadByte(file);

			material_names.push_back(mat_name);

			VertexBuffer* vbo = new VertexBuffer(Triangles);

			MaterialModelPair mmp = MaterialModelPair();
			mmp.material_index = i;
			mmp.vbo = vbo;

			unsigned int vinfo_count = ReadUInt32(file);
			int target = 0;
			SkinVInfo tri[3];
			for(unsigned int j = 0; j < vinfo_count; j++)
			{
				float x = ReadSingle(file);
				float y = ReadSingle(file);
				float z = ReadSingle(file);
				float u = ReadSingle(file);
				float v = ReadSingle(file);
				float nx = ReadSingle(file);
				float ny = ReadSingle(file);
				float nz = ReadSingle(file);
				unsigned char indices[4];
				unsigned char weights[4];
				for(int k = 0; k < 4; k++)
				{
					indices[k] = ReadByte(file);
					weights[k] = ReadByte(file);
				}
				tri[target++] = SkinVInfo(Vec3(x, y, z), Vec3(u, v, 0.0f), Vec3(nx, ny, nz), indices, weights);
				if(target == 3)
				{
					AddTriangleVertexInfo(vbo, tri[0], tri[1], tri[2]);
					target = 0;
				}
			}

			material_model_pairs.push_back(mmp);
		}

		Skeleton* temp_skel;
		Skeleton::ReadSkeleton(file, &temp_skel);

		skeleton = temp_skel;

		return 0;
	}




	/*
	 * Saver for AAK files
	 */
	int SaveAAK(string filename, SkinnedModel* model, bool overwrite)
	{
		ofstream file(filename.c_str(), ios::out | ios::binary);
		if(!file)
			return 1;

		// write number of materials
		unsigned int materials_count = model->material_model_pairs.size();
		WriteUInt32(materials_count, file);

		// write each material
		for(unsigned int i = 0; i < materials_count; i++)
		{
			MaterialModelPair& mmp = model->material_model_pairs[i];
			VertexBuffer* vbo = mmp.vbo;

			// name of material
			unsigned int mat_name_len = model->material_names[i].length();
			WriteByte((unsigned char)mat_name_len, file);
			for(unsigned int j = 0; j < mat_name_len; j++)
				WriteByte((unsigned char)model->material_names[i][j], file);

			// vbo data
			unsigned int vinfo_count = vbo->GetNumVerts();
			WriteUInt32(vinfo_count, file);
			for(unsigned int j = 0; j < vinfo_count; j++)
			{
				SkinVInfo& vinfo = GetSkinVInfo(vbo, j);

				WriteSingle(vinfo.x.x, file);
				WriteSingle(vinfo.x.y, file);
				WriteSingle(vinfo.x.z, file);
				WriteSingle(vinfo.uvw.x, file);
				WriteSingle(vinfo.uvw.y, file);
				WriteSingle(vinfo.n.x, file);
				WriteSingle(vinfo.n.y, file);
				WriteSingle(vinfo.n.z, file);
				for(int k = 0; k < 4; k++)
				{
					WriteByte(vinfo.indices[k], file);
					WriteByte(vinfo.weights[k], file);
				}
			}
		}

		Skeleton::WriteSkeleton(file, model->skeleton);

		file.close();

		return 0;
	}




	/*
	 * Loader for AAA files
	 */
	int LoadAAA(string filename, KeyframeAnimation& anim)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		unsigned char anim_name_len = ReadByte(file);
		string anim_name = "";
		for(unsigned int i = 0; i < anim_name_len; i++)
			anim_name += ReadByte(file);

		unsigned int frame_count = ReadUInt32(file);
		vector<Keyframe> frames = vector<Keyframe>();
		for(unsigned int i = 0; i < frame_count; i++)
		{
			Keyframe frame = Keyframe();

			frame.next = ReadInt32(file);
			frame.duration = ReadSingle(file);

			unsigned int bone_count = ReadUInt32(file);
			for(unsigned int j = 0; j < bone_count; j++)
			{
				string bone_name = "";
				unsigned char bone_name_len = ReadByte(file);
				for(unsigned int k = 0; k < bone_name_len; k++)
					bone_name += ReadByte(file);

				Vec3 ori;
				ori.x = ReadSingle(file);
				ori.y = ReadSingle(file);
				ori.z = ReadSingle(file);
				Vec3 pos;
				pos.x = ReadSingle(file);
				pos.y = ReadSingle(file);
				pos.z = ReadSingle(file);

				frame.values[bone_name] = BoneInfluence(ori, pos, 1.0f);
			}

			frames.push_back(frame);
		}

		anim.name = anim_name;
		anim.frames = frames;

		return 0;
	}




	/*
	 * Saver for AAA files
	 */
	int SaveAAA(string filename, KeyframeAnimation& anim)
	{
		ofstream file(filename.c_str(), ios::out | ios::binary);
		if(!file)
			return 1;

		// write animation name
		unsigned char anim_name_len = anim.name.length();
		WriteByte(anim_name_len, file);
		for(unsigned int i = 0; i < anim_name_len; i++)
			WriteByte(anim.name[i], file);

		// write frames
		unsigned int frame_count = anim.frames.size();
		WriteUInt32(frame_count, file);
		for(vector<Keyframe>::iterator iter =anim.frames.begin(); iter != anim.frames.end(); iter++)
		{
			Keyframe& frame = *iter;

			WriteInt32(frame.next, file);
			WriteSingle(frame.duration, file);

			// write each bone of the frame
			unsigned int values_count = frame.values.size();
			WriteUInt32(values_count, file);
			for(map<string, BoneInfluence>::iterator jter = frame.values.begin(); jter != frame.values.end(); jter++)
			{
				// name of the bone
				unsigned char bone_name_len = jter->first.length();
				WriteByte(bone_name_len, file);
				for(unsigned int i = 0; i < bone_name_len; i++)
					WriteByte(jter->first[i], file);

				// bone influence
				BoneInfluence& inf = jter->second;
				WriteSingle(inf.ori.x, file);			// orientation...
				WriteSingle(inf.ori.y, file);
				WriteSingle(inf.ori.z, file);
				WriteSingle(inf.pos.x, file);			// position...
				WriteSingle(inf.pos.y, file);
				WriteSingle(inf.pos.z, file);
			}
		}

		return 0;
	}
}
