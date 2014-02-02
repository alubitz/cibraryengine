#pragma once

#include "StdAfx.h"

#include "TimingInfo.h"
#include "Disposable.h"

#include "Vector.h"

namespace CibraryEngine
{
	enum SoundType
	{
		Music,
		Effects,
		Voice
	};

	class SoundSystem;
	class SoundBuffer;

	class SoundSource : public Disposable
	{
		private:

			SoundType sound_type;
			float loudness;
			bool valid;

			void SetPosVel();

		protected:

			void InnerDispose();

			void InvalidateSound();

		public:

			SoundSystem* sys;

			SoundBuffer* sound;
			Vec3 pos, vel;

			unsigned int id;

			SoundSource(SoundType sound_type, const Vec3& pos, const Vec3& vel, SoundBuffer* sound, float vol, bool looping, SoundSystem* sys);

			void Update(const TimingInfo& time);
			void StopLooping();

			float GetLoudness();
			void SetLoudness(float f);

			bool IsValid();
	};
}
