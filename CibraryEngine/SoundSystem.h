#pragma once

#include "StdAfx.h"
#include "Disposable.h"
#include "TimingInfo.h"

#include "Vector.h"

namespace CibraryEngine
{
	class SoundSource;
	class SoundBuffer;

	using namespace std;

	/** Class for playing sound */
	class SoundSystem : public Disposable
	{
		private:

			float speed_of_sound;
			float doppler_factor;

			Vec3 listener_pos;
			Vec3 listener_vel;
			Vec3 listener_forward;
			Vec3 listener_up;

			void PlaceListener();

		protected:

			void InnerDispose();

		public:

			/** OpenAL device */
			ALCdevice* al_device;
			/** OpenAL context */
			ALCcontext* al_context;

			/** Whether the system is enabled */
			bool master_enable;
			/** List of active sound sources */
			list<SoundSource*> sources;

			/** Initializes a SoundSystem */
			SoundSystem();

			/** Tries to enable the SoundSystem; returns true if successful */
			bool TryToEnable();
			/** Updates the SoundSystem */
			void Update(const TimingInfo& time);
			/** Stops all sounds */
			void StopAll();

			/** Plays the specified effect at the listener's position, and returns a SoundSource that can be used to control the playing sound */
			SoundSource* PlayEffect(SoundBuffer* buffer, float loudness, bool looping);
			/** Plays the specified effect with the specified position and velocity, and returns a SoundSource that can be used to control the playing sound */
			SoundSource* PlayEffect(SoundBuffer* buffer, const Vec3& pos, const Vec3& vel, float loudness, bool looping);

			/** Gets the speed of sound */
			float GetSpeedOfSound();
			/** Gets the Doppler factor */
			float GetDopplerFactor();
			/** Gets the position of the listener */
			Vec3 GetListenerPos();
			/** Gets the velocity of the listener */
			Vec3 GetListenerVel();
			/** Gets the forward vector of the listener */
			Vec3 GetListenerForward();
			/** Gets the up vector of the listener */
			Vec3 GetListenerUp();

			/** Sets the speed of sound */
			void SetSpeedOfSound(float s);
			/** Sets the Doppler factor */
			void SetDopplerFactor(float f);
			/** Sets the position of the listener */
			void SetListenerPos(const Vec3& vec);
			/** Sets tehe velocity of the listener */
			void SetListenerVel(const Vec3& vec);
			/** Sets the forward vector of the listener */
			void SetListenerForward(const Vec3& vec);
			/** Sets the up vector of the listener */
			void SetListenerUp(const Vec3& vec);

			/** If an OpenAL error has occurred, outputs to the debug log */
			static bool CheckForALErrors();
			/** If an OpenAL error has occurred, outputs the specified line number and filename to the debug log */
			static bool CheckForALErrors(int line, const string& file);
	};
}
