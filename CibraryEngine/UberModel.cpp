#include "StdAfx.h"
#include "UberModel.h"

#include "ModelLoader.h"
#include "BinaryChunk.h"
#include "Serialize.h"
#include "Sphere.h"
#include "Model.h"
#include "VertexBuffer.h"
#include "SkeletalAnimation.h"
#include "Material.h"

#include "DebugLog.h"

#include "Physics.h"

namespace CibraryEngine
{
	/*
	 * UberModel::LOD methods
	 */
	UberModel::LOD::LOD() :
		lod_name(),
		vertices(),
		texcoords(),
		normals(),
		bone_influences(),
		points(),
		edges(),
		triangles(),
		vbos(NULL)
	{
	}

	void UberModel::LOD::InnerDispose()
	{
		if(vbos != NULL)
		{
			for(vector<MaterialModelPair>::iterator iter = vbos->begin(); iter != vbos->end(); iter++)
			{
				VertexBuffer* vbo = iter->vbo;

				vbo->Dispose();
				delete vbo;
			}

			vbos->clear();

			delete vbos;
			vbos = NULL;
		}
	}

	vector<MaterialModelPair>* UberModel::LOD::GetVBOs()
	{
		if(vbos == NULL)
		{
			vector<MaterialModelPair> temp_vbos;

			// TODO: Put point and edge VBOs into the list too, once support for those is added

			unsigned int num_triangles = triangles.size();
			for(unsigned int i = 0; i < num_triangles; i++)
			{
				Triangle& tri = triangles[i];
				unsigned int mat = tri.material;

				VertexBuffer* vbo = NULL;

				// see if there is a VBO with this material already...
				for(vector<MaterialModelPair>::iterator iter = temp_vbos.begin(); iter != temp_vbos.end(); iter++)
				{
					if(iter->material_index == mat)
					{
						vbo = iter->vbo;
						break;
					}
				}

				// didn't find an existing VBO with this material?
				if(vbo == NULL)
				{
					vbo = new VertexBuffer(Triangles);
					vbo->AddAttribute("gl_Vertex", Float, 3);
					vbo->AddAttribute("gl_Normal", Float, 3);
					vbo->AddAttribute("gl_MultiTexCoord0", Float, 3);
					vbo->AddAttribute("gl_MultiTexCoord1", Float, 3);
					vbo->AddAttribute("gl_MultiTexCoord2", Float, 3);
					vbo->AddAttribute("gl_MultiTexCoord3", Float, 4);
					vbo->AddAttribute("gl_MultiTexCoord4", Float, 4);

					MaterialModelPair mmp;
					mmp.vbo = vbo;
					mmp.material_index = mat;

					temp_vbos.push_back(mmp);
				}

				// found or created a VBO; now lets add our triangle to it!
				if(bone_influences.size() > 0)
				{
					SkinVInfo a = SkinVInfo(vertices[tri.a.v], texcoords[tri.a.t], normals[tri.a.n], bone_influences[tri.a.v].indices, bone_influences[tri.a.v].weights);
					SkinVInfo b = SkinVInfo(vertices[tri.b.v], texcoords[tri.b.t], normals[tri.b.n], bone_influences[tri.b.v].indices, bone_influences[tri.b.v].weights);
					SkinVInfo c = SkinVInfo(vertices[tri.c.v], texcoords[tri.c.t], normals[tri.c.n], bone_influences[tri.c.v].indices, bone_influences[tri.c.v].weights);
					AddTriangleVertexInfo(vbo, a, b, c);
				}
				else
				{
					unsigned char indices[4];
					unsigned char weights[4];
					indices[0] = indices[1] = indices[2] = indices[3] = weights[1] = weights[2] = weights[3] = 0;
					weights[0] = 255;

					SkinVInfo a = SkinVInfo(vertices[tri.a.v], texcoords[tri.a.t], normals[tri.a.n], indices, weights);
					SkinVInfo b = SkinVInfo(vertices[tri.b.v], texcoords[tri.b.t], normals[tri.b.n], indices, weights);
					SkinVInfo c = SkinVInfo(vertices[tri.c.v], texcoords[tri.c.t], normals[tri.c.n], indices, weights);
					AddTriangleVertexInfo(vbo, a, b, c);
				}
			}

			vbos = new vector<MaterialModelPair>(temp_vbos);
		}

		return vbos;
	}




	/*
	 * UberModel::BonePhysics methods
	 */
	UberModel::BonePhysics::BonePhysics() :
		bone_name(),
		shape(NULL),
		pos(),
		ori(Quaternion::Identity()),
		span(1.0, 1.0, 0.75)
	{
	}




	/*
	 * UberModel methods
	 */
	UberModel::UberModel() :
		Disposable(),
		lods(),
		materials(),
		bones(),
		specials(),
		bounding_sphere(Vec3(), -1.0)
	{
	}

	void UberModel::InnerDispose()
	{
		for(unsigned int i = 0; i < lods.size(); i++)
		{
			LOD* lod = lods[i];

			lod->Dispose();
			delete lod;
		}
		lods.clear();
	}

	Skeleton* UberModel::CreateSkeleton()
	{
		Skeleton* skel = new Skeleton();

		for(unsigned int i = 0; i < bones.size(); i++)
			skel->AddBone(bones[i].name, bones[i].ori, bones[i].pos);
		for(unsigned int i = 0; i < bones.size(); i++)
		{
			unsigned int parent = bones[i].parent;
			skel->bones[i]->parent = parent == 0 ? NULL : skel->bones[parent - 1];
		}

		/*
		for(unsigned int i = 0; i < bones.size(); i++)
		{
			CibraryEngine::Bone* bone = skel->bones[i];

			string name = bone->name;
			float x = bone->rest_pos.x;
			float y = bone->rest_pos.y;
			float z = bone->rest_pos.z;
			string parent_name = bone->parent == NULL ? "root" : "parent is \"" + bone->parent->name + "\"";

			char str[150];
			sprintf(str, "Bone \"%s\" at (%f, %f, %f), %s\n", name.c_str(), x, y, z, parent_name.c_str());
			Debug(str);
		}
		*/

		return skel;
	}




	/*
	 * Helpers; scroll down to the bottom for UberModelLoader methods
	 */
	struct UnrecognizedChunkHandler : public ChunkTypeFunction { void HandleChunk(BinaryChunk& chunk) { Debug("Unrecognized chunk: " + chunk.GetName() + "\n"); } };

	/*
	 * LOD chunk component callback structs
	 */
	struct LODNameHandler : public ChunkTypeFunction
	{
		UberModel::LOD* lod;
		LODNameHandler(UberModel::LOD* lod) : lod(lod) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			lod->lod_name = ReadString4(ss);
		}
	};

	struct Vert3Handler : public ChunkTypeFunction
	{
		UberModel::LOD* lod;
		Vert3Handler(UberModel::LOD* lod) : lod(lod) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_vertices = ReadUInt32(ss);
			lod->vertices.resize(num_vertices);

			for(unsigned int i = 0; i < num_vertices; i++)
				lod->vertices[i] = ReadVec3(ss);
		}
	};

	struct TexC3Handler : public ChunkTypeFunction
	{
		UberModel::LOD* lod;
		TexC3Handler(UberModel::LOD* lod) : lod(lod) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_coords = ReadUInt32(ss);
			lod->texcoords.resize(num_coords);

			for(unsigned int i = 0; i < num_coords; i++)
				lod->texcoords[i] = ReadVec3(ss);
		}
	};

	struct Norm3Handler : public ChunkTypeFunction
	{
		UberModel::LOD* lod;
		Norm3Handler (UberModel::LOD* lod) : lod(lod) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_norms = ReadUInt32(ss);
			lod->normals.resize(num_norms);

			for(unsigned int i = 0; i < num_norms; i++)
				lod->normals[i] = ReadVec3(ss);
		}
	};

	struct BInf4Handler : public ChunkTypeFunction
	{
		UberModel::LOD* lod;
		BInf4Handler (UberModel::LOD* lod) : lod(lod) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_infs = ReadUInt32(ss);
			lod->bone_influences.resize(num_infs);

			for(unsigned int i = 0; i < num_infs; i++)
			{
				UberModel::CompactBoneInfluence inf;
				for(unsigned int k = 0; k < 4; k++)
				{
					inf.indices[k] = ReadByte(ss);
					inf.weights[k] = ReadByte(ss);
				}

				lod->bone_influences[i] = inf;
			}
		}
	};

	struct VTN1Handler : public ChunkTypeFunction
	{
		UberModel::LOD* lod;
		VTN1Handler (UberModel::LOD* lod) : lod(lod) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_points = ReadUInt32(ss);
			lod->points.resize(num_points);

			for(unsigned int i = 0; i < num_points; i++)
			{
				UberModel::Point point;
				point.material = ReadUInt32(ss);

				UberModel::VTN vtn;
				vtn.v = ReadUInt32(ss);
				vtn.t = ReadUInt32(ss);
				vtn.n = ReadUInt32(ss);

				point.vtn = vtn;

				lod->points[i] = point;
			}
		}
	};

	struct VTN2Handler : public ChunkTypeFunction
	{
		UberModel::LOD* lod;
		VTN2Handler (UberModel::LOD* lod) : lod(lod) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_edges = ReadUInt32(ss);
			lod->edges.resize(num_edges);

			for(unsigned int i = 0; i < num_edges; i++)
			{
				UberModel::Edge edge;
				edge.material = ReadUInt32(ss);

				UberModel::VTN a;
				a.v = ReadUInt32(ss);
				a.t = ReadUInt32(ss);
				a.n = ReadUInt32(ss);

				UberModel::VTN b;
				b.v = ReadUInt32(ss);
				b.t = ReadUInt32(ss);
				b.n = ReadUInt32(ss);

				edge.a = a;
				edge.b = b;

				lod->edges[i] = edge;
			}
		}
	};

	struct VTN3Handler : public ChunkTypeFunction
	{
		UberModel::LOD* lod;
		VTN3Handler (UberModel::LOD* lod) : lod(lod) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_tris = ReadUInt32(ss);
			lod->triangles.resize(num_tris);

			for(unsigned int i = 0; i < num_tris; i++)
			{
				UberModel::Triangle tri;
				tri.material = ReadUInt32(ss);

				UberModel::VTN a;
				a.v = ReadUInt32(ss);
				a.t = ReadUInt32(ss);
				a.n = ReadUInt32(ss);

				UberModel::VTN b;
				b.v = ReadUInt32(ss);
				b.t = ReadUInt32(ss);
				b.n = ReadUInt32(ss);

				UberModel::VTN c;
				c.v = ReadUInt32(ss);
				c.t = ReadUInt32(ss);
				c.n = ReadUInt32(ss);

				tri.a = a;
				tri.b = b;
				tri.c = c;

				lod->triangles[i] = tri;
			}
		}
	};

	void WriteLOD(UberModel::LOD* lod, ostream& whole_ss)
	{
		stringstream ss;

		unsigned int lod_name_len = lod->lod_name.length();
		if(lod_name_len > 0)
		{
			BinaryChunk name_chunk("NAME____");

			stringstream name_ss;
			WriteString4(lod->lod_name, name_ss);

			name_chunk.data = name_ss.str();
			name_chunk.Write(ss);
		}

		if(lod->vertices.size() > 0)
		{
			BinaryChunk vert_chunk("VERT3___");
			stringstream vert_ss;

			unsigned int num_vertices = lod->vertices.size();
			WriteUInt32(num_vertices, vert_ss);
			for(unsigned int j = 0; j < num_vertices; j++)
				WriteVec3(lod->vertices[j], vert_ss);
			vert_chunk.data = vert_ss.str();
			vert_chunk.Write(ss);
		}

		if(lod->texcoords.size() > 0)
		{
			BinaryChunk texc_chunk("TEXC3___");
			stringstream tex_ss;

			unsigned int num_texcoords = lod->texcoords.size();
			WriteUInt32(num_texcoords, tex_ss);
			for(unsigned int j = 0; j < num_texcoords; j++)
				WriteVec3(lod->texcoords[j], tex_ss);
			texc_chunk.data = tex_ss.str();
			texc_chunk.Write(ss);
		}

		if(lod->normals.size() > 0)
		{
			BinaryChunk norm_chunk("NORM3___");
			stringstream norm_ss;

			unsigned int num_normals = lod->normals.size();
			WriteUInt32(num_normals, norm_ss);
			for(unsigned int j = 0; j < num_normals; j++)
				WriteVec3(lod->normals[j], norm_ss);
			norm_chunk.data = norm_ss.str();
			norm_chunk.Write(ss);
		}

		if(lod->bone_influences.size() > 0)
		{
			BinaryChunk bone_chunk("BINF4___");
			stringstream bone_ss;

			unsigned int num_bone_infs = lod->bone_influences.size();
			WriteUInt32(num_bone_infs, bone_ss);
			for(unsigned int j = 0; j < num_bone_infs; j++)
			{
				UberModel::CompactBoneInfluence& inf = lod->bone_influences[j];
				for(unsigned int k = 0; k < 4; k++)
				{
					WriteByte(inf.indices[k], bone_ss);
					WriteByte(inf.weights[k], bone_ss);
				}
			}
			bone_chunk.data = bone_ss.str();
			bone_chunk.Write(ss);
		}

		if(lod->points.size() > 0)
		{
			BinaryChunk point_chunk("VTN1____");
			stringstream point_ss;

			unsigned int num_points = lod->points.size();
			WriteUInt32(num_points, point_ss);
			for(unsigned int j = 0; j < num_points; j++)
			{
				UberModel::Point& point = lod->points[j];
				WriteUInt32(point.material, point_ss);
				WriteUInt32(point.vtn.v, point_ss);
				WriteUInt32(point.vtn.t, point_ss);
				WriteUInt32(point.vtn.n, point_ss);
			}
			point_chunk.data = point_ss.str();
			point_chunk.Write(ss);
		}

		if(lod->edges.size() > 0)
		{
			BinaryChunk edge_chunk("VTN2____");
			stringstream edge_ss;

			unsigned int num_edges = lod->edges.size();
			WriteUInt32(num_edges, edge_ss);
			for(unsigned int j = 0; j < num_edges; j++)
			{
				UberModel::Edge& edge = lod->edges[j];
				WriteUInt32(edge.material, edge_ss);
				WriteUInt32(edge.a.v, edge_ss);
				WriteUInt32(edge.a.t, edge_ss);
				WriteUInt32(edge.a.n, edge_ss);
				WriteUInt32(edge.b.v, edge_ss);
				WriteUInt32(edge.b.t, edge_ss);
				WriteUInt32(edge.b.n, edge_ss);
			}
			edge_chunk.data = edge_ss.str();
			edge_chunk.Write(ss);
		}

		if(lod->triangles.size() > 0)
		{
			BinaryChunk tri_chunk("VTN3____");
			stringstream tri_ss;

			unsigned int num_triangles = lod->triangles.size();
			WriteUInt32(num_triangles, tri_ss);
			for(unsigned int j = 0; j < num_triangles; j++)
			{
				UberModel::Triangle& tri = lod->triangles[j];
				WriteUInt32(tri.material, tri_ss);
				WriteUInt32(tri.a.v, tri_ss);
				WriteUInt32(tri.a.t, tri_ss);
				WriteUInt32(tri.a.n, tri_ss);
				WriteUInt32(tri.b.v, tri_ss);
				WriteUInt32(tri.b.t, tri_ss);
				WriteUInt32(tri.b.n, tri_ss);
				WriteUInt32(tri.c.v, tri_ss);
				WriteUInt32(tri.c.t, tri_ss);
				WriteUInt32(tri.c.n, tri_ss);
			}
			tri_chunk.data = tri_ss.str();
			tri_chunk.Write(ss);
		}

		BinaryChunk lod_chunk("LOD_____");
		lod_chunk.data = ss.str();
		lod_chunk.Write(whole_ss);
	}

	Sphere UberModel::GetBoundingSphere()
	{
		if(bounding_sphere.radius < 0)
		{
			bool first = true;
			for(unsigned int i = 0; i < lods.size(); i++)
			{
				LOD* lod = lods[i];
				unsigned int num_verts = lod->vertices.size();
				for(unsigned int j = 0; j < num_verts; j++)
				{
					if(first)
					{
						bounding_sphere = Sphere(lod->vertices[j], 0);
						first = false;
					}
					else
						bounding_sphere = Sphere::Expand(bounding_sphere, lod->vertices[j]);
				}
			}
		}
		return bounding_sphere;
	}

	/*
	 * UberModel::LoadZZZ callback structs
	 */

	struct LODSChunkHandler : public ChunkTypeFunction
	{
		UberModel* model;
		LODSChunkHandler(UberModel* model) : model(model) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_lods = ReadUInt32(ss);
			for(unsigned int i = 0; i < num_lods; i++)
			{
				BinaryChunk lod_chunk;
				lod_chunk.Read(ss);

				UberModel::LOD* lod = new UberModel::LOD();

				ChunkTypeIndexer indexer;

				UnrecognizedChunkHandler badchunk;
				LODNameHandler namechunk(lod);
				Vert3Handler vertchunk(lod);
				TexC3Handler texcchunk(lod);
				Norm3Handler normchunk(lod);
				BInf4Handler bonechunk(lod);
				VTN1Handler pointchunk(lod);
				VTN2Handler edgechunk(lod);
				VTN3Handler trichunk(lod);

				indexer.SetDefaultHandler(&badchunk);
				indexer.SetHandler("NAME____", &namechunk);
				indexer.SetHandler("VERT3___", &vertchunk);
				indexer.SetHandler("TEXC3___", &texcchunk);
				indexer.SetHandler("NORM3___", &normchunk);
				indexer.SetHandler("BINF4___", &bonechunk);
				indexer.SetHandler("VTN1____", &pointchunk);
				indexer.SetHandler("VTN2____", &edgechunk);
				indexer.SetHandler("VTN3____", &trichunk);

				indexer.HandleChunk(lod_chunk);

				// add our newly processed LOD to the model's collection...
				model->lods.push_back(lod);
			}
		}
	};

	struct MatChunkHandler : public ChunkTypeFunction
	{
		UberModel* model;
		MatChunkHandler(UberModel* model) : model(model) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_mats = ReadUInt32(ss);
			model->materials.resize(num_mats);

			for(unsigned int i = 0; i < num_mats; i++)
			{
				unsigned int mat_name_len = ReadUInt32(ss);

				string mat_name;
				for(unsigned int j = 0; j < mat_name_len; j++)
					mat_name += ReadByte(ss);

				model->materials[i] = mat_name;
			}
		}
	};

	struct BoneChunkHandler : public ChunkTypeFunction
	{
		UberModel* model;
		BoneChunkHandler(UberModel* model) : model(model) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_bones = ReadUInt32(ss);
			model->bones.resize(num_bones);

			for(unsigned int i = 0; i < num_bones; i++)
			{
				UberModel::Bone bone;

				bone.name = ReadString4(ss);
				bone.parent = ReadUInt32(ss);
				bone.pos = ReadVec3(ss);
				bone.ori = ReadQuaternion(ss);

				model->bones[i] = bone;
			}
		}
	};

	struct BonePhysicsHandler : public ChunkTypeFunction
	{
		UberModel* model;
		BonePhysicsHandler(UberModel* model) : model(model) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_phys = ReadUInt32(ss);
			model->bone_physics.resize(num_phys);

			for(unsigned int i = 0; i < num_phys; i++)
			{
				UberModel::BonePhysics phys;

				unsigned int bone_name_len = ReadUInt32(ss);
				phys.bone_name = "";
				for(unsigned int j = 0; j < bone_name_len; j++)
					phys.bone_name += ReadByte(ss);

				phys.shape = ReadCollisionShape(ss);

				// deserializing everything that's left
				phys.mass = ReadSingle(ss);

				phys.pos.x = ReadSingle(ss);
				phys.pos.y = ReadSingle(ss);
				phys.pos.z = ReadSingle(ss);

				phys.ori.w = ReadSingle(ss);
				phys.ori.x = ReadSingle(ss);
				phys.ori.y = ReadSingle(ss);
				phys.ori.z = ReadSingle(ss);

				phys.span.x = ReadSingle(ss);
				phys.span.y = ReadSingle(ss);
				phys.span.z = ReadSingle(ss);

				model->bone_physics[i] = phys;
			}
		}
	};

	struct SpcChunkHandler : public ChunkTypeFunction
	{
		UberModel* model;
		SpcChunkHandler(UberModel* model) : model(model) { }

		void HandleChunk(BinaryChunk& chunk)
		{
			istringstream ss(chunk.data);

			unsigned int num_specials = ReadUInt32(ss);
			model->specials.resize(num_specials);

			for(unsigned int i = 0; i < num_specials; i++)
			{
				UberModel::Special special;

				Vec3 pos;
				pos.x = ReadSingle(ss);
				pos.y = ReadSingle(ss);
				pos.z = ReadSingle(ss);

				Vec3 norm;
				norm.x = ReadSingle(ss);
				norm.y = ReadSingle(ss);
				norm.z = ReadSingle(ss);

				special.pos = pos;
				special.normal = norm;
				special.radius = ReadSingle(ss);
				special.bone = ReadUInt32(ss);

				unsigned int info_len = ReadUInt32(ss);
				string info;
				for(unsigned int j = 0; j < info_len; j++)
					info += ReadByte(ss);

				special.info = info;

				model->specials[i] = special;
			}
		}
	};




	/*
	 * UberModelLoader methods
	 */
	UberModelLoader::UberModelLoader(ContentMan* man) : ContentTypeHandler<UberModel>(man) { }

	UberModel* UberModelLoader::Load(ContentMetadata& what)
	{
		string filename = "Files/Models/" + what.name + ".zzz";

		UberModel* model = NULL;
		unsigned int zzz_result = UberModelLoader::LoadZZZ(model, filename);

		if(zzz_result != 0)
		{
			stringstream zzz_msg;
			zzz_msg << "LoadZZZ (" << what.name << ") returned with status " << zzz_result << "; looking for suitable AAK" << endl;
			Debug(zzz_msg.str());

			// see if there is a skinned model we could convert?
			SkinnedModel* skinny = man->GetCache<SkinnedModel>()->Load(what.name);
			if(skinny == NULL)
				Debug("Unable to find a suitable AAK either!\n");
			else
			{
				model = UberModelLoader::CopySkinnedModel(skinny);
				if(SaveZZZ(model, filename) == 0)
					Debug("Successfully converted AAK --> ZZZ file\n");
				else
					Debug("Successfully loaded AAK file, but could not convert it to ZZZ\n");
			}
		}

		if(model != NULL)
			for(unsigned int i = 0; i < model->materials.size(); i++)
				man->GetCache<Material>()->Load(model->materials[i]);

		return model;
	}

	void UberModelLoader::Unload(UberModel* content, ContentMetadata& meta)
	{
		content->Dispose();
		delete content;
	}

	unsigned int UberModelLoader::LoadZZZ(UberModel*& model, string filename)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		BinaryChunk whole;
		whole.Read(file);

		if(whole.GetName() != "UMODEL__")
			return 2;

		istringstream ss(whole.data);

		ChunkTypeIndexer indexer;

		model = new UberModel();

		// TODO: process additional chunk types here

		UnrecognizedChunkHandler badchunk;
		LODSChunkHandler lodchunk(model);
		MatChunkHandler matchunk(model);
		BoneChunkHandler bonechunk(model);
		BonePhysicsHandler bphyschunk(model);
		SpcChunkHandler spcchunk(model);

		indexer.SetDefaultHandler(&badchunk);
		indexer.SetHandler("LODS____", &lodchunk);
		indexer.SetHandler("MAT_____", &matchunk);
		indexer.SetHandler("BONE____", &bonechunk);
		indexer.SetHandler("SPC_____", &spcchunk);
		indexer.SetHandler("BPHYS___", &bphyschunk);

		indexer.HandleChunk(whole);

		return 0;
	}

	unsigned int UberModelLoader::SaveZZZ(UberModel* model, string filename)
	{
		if(model == NULL)
			return 1;

		BinaryChunk whole;
		whole.SetName("UMODEL__");

		stringstream ss;

		unsigned int lods_count = model->lods.size();
		if(lods_count > 0)
		{
			BinaryChunk lods_chunk("LODS____");
			stringstream lods_ss;
			WriteUInt32(lods_count, lods_ss);
			for(unsigned int i = 0; i < lods_count; i++)
				WriteLOD(model->lods[i], lods_ss);
			lods_chunk.data = lods_ss.str();
			lods_chunk.Write(ss);
		}

		unsigned int mats_count = model->materials.size();
		if(mats_count > 0)
		{
			BinaryChunk mats_chunk("MAT_____");
			stringstream mat_ss;
			WriteUInt32(mats_count, mat_ss);
			for(unsigned int i = 0; i < mats_count; i++)
				WriteString4(model->materials[i], mat_ss);
			mats_chunk.data = mat_ss.str();
			mats_chunk.Write(ss);
		}

		unsigned int bone_count = model->bones.size();
		if(bone_count > 0)
		{
			BinaryChunk bones_chunk("BONE____");
			stringstream bone_ss;
			WriteUInt32(bone_count, bone_ss);
			for(unsigned int i = 0; i < bone_count; i++)
			{
				UberModel::Bone& bone = model->bones[i];
				WriteString4(bone.name, bone_ss);
				WriteUInt32(bone.parent, bone_ss);
				WriteVec3(bone.pos, bone_ss);
				WriteQuaternion(bone.ori, bone_ss);
			}
			bones_chunk.data = bone_ss.str();
			bones_chunk.Write(ss);
		}

		unsigned int phys_count = model->bone_physics.size();
		if(phys_count > 0)
		{
			BinaryChunk phys_chunk("BPHYS___");
			stringstream phys_ss;
			WriteUInt32(phys_count, phys_ss);
			for(unsigned int i = 0; i < phys_count; i++)
			{
				UberModel::BonePhysics& phys = model->bone_physics[i];
				WriteString4(phys.bone_name, phys_ss);
				WriteCollisionShape(phys.shape, phys_ss);
				WriteSingle(phys.mass, phys_ss);
				WriteVec3(phys.pos, phys_ss);
				WriteQuaternion(phys.ori, phys_ss);
				WriteVec3(phys.span, phys_ss);
			}
			phys_chunk.data = phys_ss.str();
			phys_chunk.Write(ss);
		}

		unsigned int special_count = model->specials.size();
		if(special_count > 0)
		{
			BinaryChunk special_chunk("SPC_____");
			stringstream spc_ss;
			WriteUInt32(special_count, spc_ss);
			for(unsigned int i = 0; i < special_count; i++)
			{
				UberModel::Special& spc = model->specials[i];
				WriteVec3(spc.pos, spc_ss);
				WriteVec3(spc.normal, spc_ss);
				WriteSingle(spc.radius, spc_ss);
				WriteUInt32(spc.bone, spc_ss);
				WriteString4(spc.info, spc_ss);
			}
			special_chunk.data = spc_ss.str();
			special_chunk.Write(ss);
		}

		// TODO: export additional chunk types here

		whole.data = ss.str();

		ofstream file(filename.c_str(), ios::out | ios::binary);
		if(!file)
			return 2;

		whole.Write(file);

		return 0;
	}

	UberModel* UberModelLoader::CopySkinnedModel(SkinnedModel* skinny)
	{
		if(skinny == NULL)
			return NULL;

		UberModel* uber = new UberModel();

		AddSkinnedModel(uber, skinny, "LOD 0");

		vector<Bone*>& bones = skinny->skeleton->bones;
		for(unsigned int i = 0; i < bones.size(); i++)
		{
			UberModel::Bone bone;
			bone.name = bones[i]->name;
			bone.pos = bones[i]->rest_pos;
			bone.ori = bones[i]->rest_ori;
			bone.parent = 0;
			for(unsigned int j = 0; j < bones.size(); j++)
				if(bones[j] == bones[i]->parent)
				{
					bone.parent = j + 1;
					break;
				}

			uber->bones.push_back(bone);
		}

		return uber;
	}

	void UberModelLoader::AddSkinnedModel(UberModel* uber, SkinnedModel* skinny, string lod_name)
	{
		unsigned int initial_material = uber->materials.size();
		UberModel::LOD* lod = new UberModel::LOD();

		for(unsigned int i = 0; i < skinny->material_model_pairs.size(); i++)
		{
			MaterialModelPair& pair = skinny->material_model_pairs[i];
			VertexBuffer* vbo = pair.vbo;

			for(unsigned int j = 0; j < vbo->GetNumVerts();)
			{
				UberModel::Triangle triangle;
				triangle.material = pair.material_index + initial_material;

				for(int corner = 0; corner < 3; corner++, j++)
				{
					UberModel::VTN& corner_vtn = corner == 0 ? triangle.a : corner == 1 ? triangle.b : triangle.c;

					SkinVInfo& vinfo = GetSkinVInfo(vbo, j);

					unsigned int k;

					Vec3 x = vinfo.x;
					for(k = 0; k < lod->vertices.size(); k++)
						if(lod->vertices[k] == x)
							break;
					k = lod->vertices.size();
					if(k == lod->vertices.size())
					{
						lod->vertices.push_back(x);

						// this doesn't exist in the plain VTN version
						UberModel::CompactBoneInfluence inf;
						for(int windex = 0; windex < 4; windex++)
						{
							inf.indices[windex] = vinfo.indices[windex];
							inf.weights[windex] = vinfo.weights[windex];
						}
						lod->bone_influences.push_back(inf);
					}
					corner_vtn.v = k;

					Vec3 uvw = vinfo.uvw;
					for(k = 0; k < lod->texcoords.size(); k++)
						if(lod->texcoords[k] == uvw)
							break;
					k = lod->texcoords.size();
					if(k == lod->texcoords.size())
						lod->texcoords.push_back(uvw);
					corner_vtn.t = k;

					Vec3 n = vinfo.n;
					for(k = 0; k < lod->normals.size(); k++)
						if(lod->normals[k] == n)
							break;
					k = lod->normals.size();
					if(k == lod->normals.size())
						lod->normals.push_back(n);
					corner_vtn.n = k;
				}

				lod->triangles.push_back(triangle);
			}
		}
		uber->lods.push_back(lod);

		for(unsigned int i = 0; i < skinny->material_names.size(); i++)
			uber->materials.push_back(skinny->material_names[i]);
	}
}
