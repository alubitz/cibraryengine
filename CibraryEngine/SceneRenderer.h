#pragma once

#include "StdAfx.h"

#include "MathTypes.h"

namespace CibraryEngine
{
	using namespace std;

	class CameraView;

	class Material;
	class SceneRenderer;
	class RenderNode;

	// override these
	/** Class representing a light source; override these functions */
	class LightSource
	{
		public:

			virtual ~LightSource() { }

			/** Determines whether a LightSource is close enough to an object that its light should affect that object; default implementation returns true */
			virtual bool IsWithinLightingRange(Sphere bs) { return true; }
			/** Set this LightSource as the specified OpenGL LIGHT */
			virtual void SetLight(int which) { }
			/** Unset this LightSource as the specified OpenGL LIGHT */
			virtual void UnsetLight(int which) { }
	};

	// the thing that draws stuff
	/** Class which handles drawing all your render nodes */
	class SceneRenderer
	{
		public:

			/** The camera from which this scene is being viewed */
			CameraView* camera;

			/** Collection of all the objects in the scene, to be populated by the Vis function of entities */
			vector<RenderNode> objects;
			/** Collection of all the lights in the scene, to be populated by the Vis function of entities */
			vector<LightSource*> lights;

			/** Initializes a SceneRenderer */
			SceneRenderer(CameraView* camera) : camera(camera), objects(), lights() { }
			virtual ~SceneRenderer() { }

			/** Renders the scene */
			virtual void Render();
			/** Calls the cleanup functions of the various materials */
			void Cleanup();
	};
}
