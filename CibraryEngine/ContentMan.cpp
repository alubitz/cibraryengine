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

namespace CibraryEngine
{
	/*
	 * ContentMan methods
	 */
	ContentMan::ContentMan()
	{
		SetHandler<Texture2D>(new Texture2DLoader(this));
		SetHandler<BitmapFont>(new BitmapFontLoader(this));
		SetHandler<Cursor>(new CursorLoader(this));
		SetHandler<VTNModel>(new ModelLoader(this));
		SetHandler<SkinnedModel>(new SkinnedModelLoader(this));
		SetHandler<Shader>(new ShaderLoader(this));
		SetHandler<TextureCube>(new TextureCubeLoader(this));
		SetHandler<SoundBuffer>(new SoundBufferLoader(this));
		SetHandler<UberModel>(new UberModelLoader(this));
	}
}
