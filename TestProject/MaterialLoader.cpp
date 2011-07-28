#include "MaterialLoader.h"

#include "DSNLoader.h"
#include "DSNMaterial.h"
#include "GlowyModelMaterial.h"

namespace Test
{
	/*
	 * MaterialLoader methods
	 */
	MaterialLoader::MaterialLoader(ContentMan* man) :
		ContentTypeHandler<Material>(man)
	{
		// creating shader
		Shader* vertex_shader = man->Load<Shader>("vert-v");
		Shader* fragment_shader = man->Load<Shader>("normal-f");

		ShaderProgram* shader = new ShaderProgram(vertex_shader, fragment_shader);

		shader->AddUniform<Texture2D>(new UniformTexture2D("diffuse", 0));
		shader->AddUniform<Texture2D>(new UniformTexture2D("specular", 1));
		shader->AddUniform<Texture2D>(new UniformTexture2D("normal_map", 2));
		shader->AddUniform<Texture1D>(new UniformTexture1D("bone_matrices", 4));
		shader->AddUniform<int>(new UniformInt("bone_count"));

		dsn_loader = new DSNLoader(man, shader, man->Load<Texture2D>("default-n"), man->Load<Texture2D>("default-s"));



		Shader* glowy_v = man->Load<Shader>("pass-v");
		Shader* glowy2d_f = man->Load<Shader>("glowy2d-f");
		Shader* glowy3d_f = man->Load<Shader>("glowy3d-f");

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
			stringstream ss;
			ss << "LoadMaterial (" << short_filename << ") failed; error code is " << load_result << endl;
			Debug(ss.str());

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
				*result_out = new GlowyModelMaterial(Texture3D::FromSpriteSheetAnimation(content->Load<Texture2D>(name), spritesheet.col_size, spritesheet.row_size, spritesheet.cols, spritesheet.rows, spritesheet.frames), shader_2d);
			else
				*result_out = new GlowyModelMaterial(content->Load<Texture2D>(name), shader_2d);
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




	int MaterialLoader::LoadMaterial(string filename, Material** result_out)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 1;

		NamedItemDictionaryTableParser parser(&file);

		DSNMaterialSetter dsn_setter(&file, dsn_loader, result_out);
		GlowyMaterialSetter glowy_setter(&file, man, glowy2d_shader, glowy3d_shader, result_out);

		parser.field_setters["dsn"] = &dsn_setter;
		parser.field_setters["glowy"] = &glowy_setter;

		parser.ParseTable();

		return 0;
	}
}
