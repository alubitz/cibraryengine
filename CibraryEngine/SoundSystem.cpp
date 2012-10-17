#include "StdAfx.h"
#include "SoundSystem.h"
#include "SoundSource.h"
#include "SoundBuffer.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * SoundSystem methods
	 */
	SoundSystem::SoundSystem() :
		speed_of_sound(1000),
		doppler_factor(0),
		listener_pos(),
		listener_vel(),
		listener_forward(0, 0, -1),
		listener_up(0, 1, 0),
		al_device(NULL),
		al_context(NULL),
		master_enable(false),
		sources()
	{
	}

	void SoundSystem::InnerDispose()
	{
		for(list<SoundSource*>::iterator iter = sources.begin(); iter != sources.end(); ++iter)
		{
			SoundSource* source = *iter;
			source->Dispose();
			delete source;
		}
		sources.clear();
	}

	void SoundSystem::PlaceListener()
	{
		if(!master_enable)
			return;

		alListener3f(AL_POSITION, listener_pos.x, listener_pos.y, listener_pos.z);
		alListener3f(AL_VELOCITY, listener_vel.x, listener_vel.y, listener_vel.z);

		float ori[] = { listener_forward.x, listener_forward.y, listener_forward.z, listener_up.x, listener_up.y, listener_up.z };

		alListenerfv(AL_ORIENTATION, ori);
	}

	bool SoundSystem::TryToEnable()
	{
		if(master_enable)
			return true;
		else
		{
			if(alcIsExtensionPresent(NULL, "ALC_ENUMERATION_EXT"))
				if(const ALchar* device_list = alcGetString(NULL, ALC_DEVICE_SPECIFIER))
				{
					vector<const ALchar*> device_names;

					size_t pos = 0;
					while(size_t size = strlen(device_list + pos))
					{
						device_names.push_back(device_list + pos);
						pos += size + 1;
					}

					Debug(((stringstream&)(stringstream() << "Found " << device_names.size() << " audio device(s)\n")).str());
					for(vector<const ALchar*>::iterator iter = device_names.begin(); iter != device_names.end(); ++iter)
						Debug(((stringstream&)(stringstream() << '\t' << *iter << endl)).str());
				}
				else
					Debug("Something funky going on with your OpenAL; Unable to enumerate OpenAL devices\n");
			else
				Debug("Unable to enumerate OpenAL devices\n");

			al_device = alcOpenDevice("Generic Software");
			al_context = alcCreateContext(al_device, 0);

			ALCenum alc_error = alcGetError(al_device);
			if(alc_error != ALC_NO_ERROR)
			{
				Debug(((stringstream&)(stringstream() << "ALC error: " << alc_error << "; unable to initialize audio\n" << endl)).str());
				return false;
			}

			alcMakeContextCurrent(al_context);

			master_enable = true;
			SetSpeedOfSound(speed_of_sound);
			SetDopplerFactor(doppler_factor);

			if(CheckForALErrors() || al_device == NULL || al_context == NULL)
			{
				master_enable = false;
				Debug("Unable to initialize audio\n");
				return false;
			}

			Debug("Audio system initialized successfully\n");
			return true;
		}
	}

	void SoundSystem::Update(TimingInfo time)
	{
		if(!master_enable)
			return;

		PlaceListener();

		for(list<SoundSource*>::iterator iter = sources.begin(); iter != sources.end();)
		{
			SoundSource* source = *iter;
			source->Update(time);
			if(!source->IsValid())
			{
				source->Dispose();
				delete source;
				iter = sources.erase(iter);
			}
			else
				++iter;
		}

		CheckForALErrors();
	}

	void SoundSystem::StopAll()
	{
		for(list<SoundSource*>::iterator iter = sources.begin(); iter != sources.end(); ++iter)
		{
			SoundSource* source = *iter;
			source->Dispose();
			delete source;
		}
		sources.clear();
	}

	SoundSource* SoundSystem::PlayEffect(SoundBuffer* buffer, float loudness, bool looping)
	{
		return PlayEffect(buffer, listener_pos, listener_vel, loudness, looping);
	}

	SoundSource* SoundSystem::PlayEffect(SoundBuffer* buffer, Vec3 pos, Vec3 vel, float loudness, bool looping)
	{
		if(!master_enable)
			return NULL;
		if(buffer == NULL)
			return NULL;

		SoundSource* source = new SoundSource(Effects, pos, vel, buffer, loudness, looping, this);
		sources.push_back(source);

		return source;
	}

	float SoundSystem::GetSpeedOfSound() { return speed_of_sound; }
	float SoundSystem::GetDopplerFactor() { return doppler_factor; }
	Vec3 SoundSystem::GetListenerPos() { return listener_pos; }
	Vec3 SoundSystem::GetListenerVel() { return listener_vel; }
	Vec3 SoundSystem::GetListenerForward() { return listener_forward; }
	Vec3 SoundSystem::GetListenerUp() { return listener_up; }

	void SoundSystem::SetSpeedOfSound(float s)
	{
		speed_of_sound = s;
		if(master_enable)
			alSpeedOfSound(s);
	}
	void SoundSystem::SetDopplerFactor(float f)
	{
		doppler_factor = f;
		if(master_enable)
			alDopplerFactor(f);
	}
	void SoundSystem::SetListenerPos(Vec3 vec) { listener_pos = vec; }
	void SoundSystem::SetListenerVel(Vec3 vec) { listener_vel = vec; }
	void SoundSystem::SetListenerForward(Vec3 vec) { listener_forward = vec; }
	void SoundSystem::SetListenerUp(Vec3 vec) { listener_up = vec; }

	bool SoundSystem::CheckForALErrors()
	{
		ALenum err = alGetError();
		if(err != AL_NO_ERROR)
		{
			Debug("AL error!\n");
			return true;
		}
		return false;
	}

	bool SoundSystem::CheckForALErrors(int line, string file)
	{
		ALenum err = alGetError();
		if(err != AL_NO_ERROR)
		{
			stringstream message;
			message << "AL Error #" << err << " at line " << line << " of " << file << endl;
			Debug(message.str());
			return true;
		}
		return false;
	}
}
