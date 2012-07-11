#include "StdAfx.h"
#include "MaterialLoader.h"

#include "DSNLoader.h"

#include "DSNMaterial.h"
#include "GlowyModelMaterial.h"
#include "ParticleMaterial.h"
#include "BillboardMaterial.h"

#include "Shader.h"
#include "UniformVariables.h"

#include "TableParse.h"

namespace CibraryEngine
{
	/*
	 * MaterialLoader methods
	 */
	MaterialLoader::MaterialLoader(ContentMan* man) :
		ContentTypeHandler<Material>(man),
		shader_cache(man->GetCache<Shader>()),
		tex_cache(man->GetCache<Texture2D>())
	{
		// dsn shader stuff...
		Shader* vertex_shader = shader_cache->Load("skel-v");
		Shader* fragment_shader = shader_cache->Load("normal-f");
		Shader* shadow_fragment_shader = shader_cache->Load("shadow-f");

		ShaderProgram* shader = new ShaderProgram(vertex_shader, fragment_shader);

		shader->AddUniform<Texture2D>(new UniformTexture2D("diffuse", 0));
		shader->AddUniform<Texture2D>(new UniformTexture2D("specular", 1));
		shader->AddUniform<Texture2D>(new UniformTexture2D("normal_map", 2));
		shader->AddUniform<Texture1D>(new UniformTexture1D("bone_matrices", 4));
		shader->AddUniform<int>(new UniformInt("bone_count"));
		shader->AddUniform<float>(new UniformFloat("precision_scaler"));

		ShaderProgram* shadow_shader = new ShaderProgram(vertex_shader, shadow_fragment_shader);
		shadow_shader->AddUniform<Texture1D>(new UniformTexture1D("bone_matrices", 0));
		shadow_shader->AddUniform<int>(new UniformInt("bone_count"));
		shadow_shader->AddUniform<float>(new UniformFloat("precision_scaler"));

		dsn_opaque_loader = new DSNLoader(man, shader, shadow_shader, tex_cache->Load("default-n"), tex_cache->Load("default-s"), Opaque);
		dsn_additive_loader = new DSNLoader(man, shader, shadow_shader, tex_cache->Load("default-n"), tex_cache->Load("default-s"), Additive);
		dsn_alpha_loader = new DSNLoader(man, shader, shadow_shader, tex_cache->Load("default-n"), tex_cache->Load("default-s"), Alpha);


		// glowy shader stuff...
		Shader* glowy_v = shader_cache->Load("pass-v");
		Shader* glowy2d_f = shader_cache->Load("glowy2d-f");
		Shader* glowy3d_f = shader_cache->Load("glowy3d-f");

		glowy2d_shader = new ShaderProgram(glowy_v, glowy2d_f);
		glowy2d_shader->AddUniform<Texture2D>(new UniformTexture2D("texture", 0));

		glowy3d_shader = new ShaderProgram(glowy_v, glowy3d_f);
		glowy3d_shader->AddUniform<Texture3D>(new UniformTexture3D("texture", 0));
	}

	Material* MaterialLoader::Load(ContentMetadata& what)
	{
		Material* material = NULL;

		string short_filename = what.name + ".txt";
		int load_result = LoadMaterial("Files/Materials/" + short_filename, &material);
		if(load_result != 0)
		{
			Debug(((stringstream&)(stringstream() << "LoadMaterial (" << short_filename << ") failed; error code is " << load_result << endl)).str());
			return NULL;
		}

		return material;
	}

	void MaterialLoader::Unload(Material* content, ContentMetadata& meta)
	{
		content->Dispose();
		delete content;
	}




	struct DSNMaterialSetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		istream* stream;
		Material** result_out;
		DSNLoader* dsn_loader;

		DSNMaterialSetter(istream* stream, DSNLoader* dsn_loader, Material** result_out) : 
			stream(stream),
			result_out(result_out),
			dsn_loader(dsn_loader)
		{
		}

		TableParseable* Set(string val) { *result_out = dsn_loader->Load(val.substr(1, val.length() - 2)); return NULL; }
	};

	struct GlowyMaterialField : public NamedItemDictionaryTableParser
	{
		ContentMan* content;
		Material** result_out;
		string name;

		ShaderProgram* shader_2d;
		ShaderProgram* shader_3d;

		struct SpriteSheetInfo { int col_size; int row_size; int cols; int rows; int frames; SpriteSheetInfo() : frames(-1) { } } spritesheet;

		IntSetter col_size;
		IntSetter row_size;
		IntSetter cols;
		IntSetter rows;
		IntSetter frames;

		GlowyMaterialField(istream* stream, string name, ContentMan* content, ShaderProgram* shader_2d, ShaderProgram* shader_3d, Material** result_out) :
			NamedItemDictionaryTableParser(stream),
			content(content),
			result_out(result_out),
			name(name),
			shader_2d(shader_2d),
			shader_3d(shader_3d),
			spritesheet(),
			col_size(&spritesheet.col_size),
			row_size(&spritesheet.row_size),
			cols(&spritesheet.cols),
			rows(&spritesheet.rows),
			frames(&spritesheet.frames)
		{
			field_setters["col_size"] = &col_size;
			field_setters["row_size"] = &row_size;
			field_setters["cols"] = &cols;
			field_setters["rows"] = &rows;
			field_setters["frames"] = &frames;
		}

		void End()
		{
			if(spritesheet.frames > 0)
				*result_out = new GlowyModelMaterial(Texture3D::FromSpriteSheetAnimation(content->GetCache<Texture2D>()->Load(name), spritesheet.col_size, spritesheet.row_size, spritesheet.cols, spritesheet.rows, spritesheet.frames), shader_2d);
			else
				*result_out = new GlowyModelMaterial(content->GetCache<Texture2D>()->Load(name), shader_2d);
		}
	};

	struct GlowyMaterialSetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		istream* stream;
		ContentMan* content;

		ShaderProgram* shader_2d;
		ShaderProgram* shader_3d;

		Material** result_out;

		GlowyMaterialSetter(istream* stream, ContentMan* content, ShaderProgram* shader_2d, ShaderProgram* shader_3d, Material** result_out) : stream(stream), content(content), shader_2d(shader_2d), shader_3d(shader_3d), result_out(result_out) { }

		TableParseable* Set(string val) { return new GlowyMaterialField(stream, val.substr(1, val.length() - 2), content, shader_2d, shader_3d, result_out); }
	};




	struct ParticleMaterialField : public NamedItemDictionaryTableParser
	{
		ContentMan* content;
		Material** result_out;
		string name;

		BlendStyle blend;

		struct SpriteSheetInfo { int col_size; int row_size; int cols; int rows; int frames; SpriteSheetInfo() : frames(-1) { } } spritesheet;

		IntSetter col_size;
		IntSetter row_size;
		IntSetter cols;
		IntSetter rows;
		IntSetter frames;

		ParticleMaterialField(istream* stream, string name, ContentMan* content, BlendStyle blend, Material** result_out) :
			NamedItemDictionaryTableParser(stream),
			content(content),
			result_out(result_out),
			name(name),
			blend(blend),
			spritesheet(),
			col_size(&spritesheet.col_size),
			row_size(&spritesheet.row_size),
			cols(&spritesheet.cols),
			rows(&spritesheet.rows),
			frames(&spritesheet.frames)
		{
			field_setters["col_size"] = &col_size;
			field_setters["row_size"] = &row_size;
			field_setters["cols"] = &cols;
			field_setters["rows"] = &rows;
			field_setters["frames"] = &frames;
		}

		void End()
		{
			if(spritesheet.frames > 0)
				*result_out = new ParticleMaterial(Texture3D::FromSpriteSheetAnimation(content->GetCache<Texture2D>()->Load(name), spritesheet.col_size, spritesheet.row_size, spritesheet.cols, spritesheet.rows, spritesheet.frames), blend);
			else
				*result_out = new ParticleMaterial(content->GetCache<Texture2D>()->Load(name), blend);
		}
	};

	struct ParticleMaterialSetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		istream* stream;
		ContentMan* content;

		Material** result_out;

		BlendStyle blend;

		ParticleMaterialSetter(istream* stream, ContentMan* content, BlendStyle blend, Material** result_out) : stream(stream), content(content), result_out(result_out), blend(blend) { }

		TableParseable* Set(string val) { return new ParticleMaterialField(stream, val.substr(1, val.length() - 2), content, blend, result_out); }
	};




	struct BillboardMaterialSetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		istream* stream;
		ContentMan* content;
		
		BlendStyle blend;
		ShaderProgram* shader;
		Material** result_out;

		BillboardMaterialSetter(istream* stream, ContentMan* content, BlendStyle blend, ShaderProgram* shader, Material** result_out) : stream(stream), content(content), blend(blend), shader(shader), result_out(result_out) { }

		TableParseable* Set(string val) { *result_out = new BillboardMaterial(content->GetCache<Texture2D>()->Load(val.substr(1, val.length() - 2)), blend); return NULL; }
	};




	unsigned int MaterialLoader::LoadMaterial(string filename, Material** result_out)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		NamedItemDictionaryTableParser parser(&file);

		DSNMaterialSetter dsn_opaque_setter(&file, dsn_opaque_loader, result_out);
		DSNMaterialSetter dsn_additive_setter(&file, dsn_additive_loader, result_out);
		DSNMaterialSetter dsn_alpha_setter(&file, dsn_alpha_loader, result_out);
		GlowyMaterialSetter glowy_setter(&file, man, glowy2d_shader, glowy3d_shader, result_out);
		ParticleMaterialSetter particle_alpha_setter(&file, man, Alpha, result_out);
		ParticleMaterialSetter particle_glowy_setter(&file, man, Additive, result_out);
		BillboardMaterialSetter billboard_setter(&file, man, Additive, glowy2d_shader, result_out);
		BillboardMaterialSetter billboard_alpha_setter(&file, man, Alpha, glowy2d_shader, result_out);

		parser.field_setters["dsn"] = &dsn_opaque_setter;				// duplicates, lol
		parser.field_setters["dsn_opaque"] = &dsn_opaque_setter;

		parser.field_setters["dsn_additive"] = &dsn_additive_setter;
		parser.field_setters["dsn_alpha"] = &dsn_alpha_setter;

		parser.field_setters["glowy"] = &glowy_setter;

		parser.field_setters["particle"] = &particle_glowy_setter;
		parser.field_setters["particle_alpha"] = &particle_alpha_setter;

		parser.field_setters["billboard"] = &billboard_setter;
		parser.field_setters["billboard_alpha"] = &billboard_alpha_setter;

		parser.ParseTable();

		file.close();

		return 0;
	}
}
