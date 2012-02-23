#include "StdAfx.h"

#include "ContentMan.h"
#include "ContentHandle.h"
#include "ContentMetadata.h"
#include "ContentTypeHandler.h"

// all of the default loadable types...
#include "Texture2D.h"
#include "BitmapFont.h"
#include "Cursor.h"
#include "Model.h"
#include "ModelLoader.h"
#include "Shader.h"
#include "TextureCube.h"
#include "SoundBuffer.h"
#include "UberModel.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	/*
	 * ContentMan methods
	 */
	ContentMan::ContentMan()
	{
		CreateCache<Texture2D>(new Texture2DLoader(this));
		CreateCache<BitmapFont>(new BitmapFontLoader(this));
		CreateCache<Cursor>(new CursorLoader(this));
		CreateCache<VertexBuffer>(new ModelLoader(this));
		CreateCache<SkinnedModel>(new SkinnedModelLoader(this));
		CreateCache<Shader>(new ShaderLoader(this));
		CreateCache<TextureCube>(new TextureCubeLoader(this));
		CreateCache<SoundBuffer>(new SoundBufferLoader(this));
		CreateCache<UberModel>(new UberModelLoader(this));
		CreateCache<CollisionShape>(new CollisionShapeLoader(this));
	}
}
