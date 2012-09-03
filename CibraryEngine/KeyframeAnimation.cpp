#include "StdAfx.h"
#include "KeyframeAnimation.h"

#include "Serialize.h"

namespace CibraryEngine
{
	using boost::unordered_map;
	using boost::unordered_set;

	/*
	 * Keyframe methods
	 */
	Keyframe::Keyframe() : values(), next(-1), duration(1.0f) { }
	Keyframe::Keyframe(float duration) : values(), next(-1), duration(duration) { }




	/*
	 * KeyframeAnimation methods
	 */
	KeyframeAnimation::KeyframeAnimation() : name(""), frames(), current_index(0), current_time(0.0f) { }
	KeyframeAnimation::KeyframeAnimation(string name) : name(name), frames(), current_index(0), current_time(0.0f) { }

	bool KeyframeAnimation::Advance(TimingInfo time)
	{
		if(current_index == -1)
			return false;
		else
		{
			current_time += time.elapsed;

			Keyframe& current = frames[current_index];

			// controls how much lag the animation system is willing to put up with, measured in seconds
			const float really_far_behind = 1.0f;
			// if the animation is more than this far behind (in seconds), don't keep advancing through all the frames in between
			if(current_time < really_far_behind + current.duration)
				while(current_time > current.duration)
				{
					current_time -= current.duration;
					current_index = frames[current_index].next;

					// if the animation finishes, quit early
					if(current_index == -1)
						return false;
				}
			else
			{
				// if it IS really far behind, advance exactly 1 frame
				current_time = 0;
				current_index = frames[current_index].next;

				if(current_index == -1)
					return false;
			}

			return true;
		}
	}

	void KeyframeAnimation::UpdatePose(TimingInfo time)
	{
		if(Advance(time))
		{
			Keyframe cur = frames[current_index];

			// linearly interpolate (lerp) only if this is not the final frame of the animation
			bool interpolate = cur.next != -1;

			if(interpolate)
			{
				Keyframe nxt = frames[cur.next];										// the next frame in the animation

				float b_frac = current_time / cur.duration, a_frac = 1 - b_frac;					// lerp coefficients

				unordered_set<unsigned int> cur_names;
				unordered_set<unsigned int> next_names;

				unordered_map<unsigned int, BoneInfluence>::iterator map_iter;
				unordered_set<unsigned int>::iterator set_iter;

				for(map_iter = cur.values.begin(); map_iter != cur.values.end(); ++map_iter)
					cur_names.insert(map_iter->first);
				for(map_iter = nxt.values.begin(); map_iter != nxt.values.end(); ++map_iter)
					next_names.insert(map_iter->first);

				// find the list of bone names used in both bones...
				unordered_set<unsigned int> shared_names(cur_names);
				for(set_iter = shared_names.begin(); set_iter != shared_names.end();)
				{
					if(next_names.find(*set_iter) == next_names.end())
						set_iter = shared_names.erase(set_iter);
					else
						++set_iter;
				}
				// ...and remove those names from the other two lists
				for(set_iter = cur_names.begin(); set_iter != cur_names.end();)
				{
					if(shared_names.find(*set_iter) != shared_names.end())
						set_iter = cur_names.erase(set_iter);
					else
						++set_iter;
				}
				for(set_iter = next_names.begin(); set_iter != next_names.end();)
				{
					if(shared_names.find(*set_iter) != shared_names.end())
						set_iter = next_names.erase(set_iter);
					else
						++set_iter;
				}

				for(set_iter = cur_names.begin(); set_iter != cur_names.end(); ++set_iter)
				{
					BoneInfluence vec = cur.values[*set_iter];
					SetBonePose(*set_iter, vec.ori * a_frac, vec.pos * a_frac);
				}
				for(set_iter = next_names.begin(); set_iter != next_names.end(); ++set_iter)
				{
					BoneInfluence vec = nxt.values[*set_iter];
					SetBonePose(*set_iter, vec.ori * b_frac, vec.pos * b_frac);
				}
				for(set_iter = shared_names.begin(); set_iter != shared_names.end(); ++set_iter)
				{
					BoneInfluence cur_vec = cur.values[*set_iter];
					BoneInfluence next_vec = nxt.values[*set_iter];
					SetBonePose(*set_iter, cur_vec.ori * a_frac + next_vec.ori * b_frac, cur_vec.pos * a_frac + next_vec.pos * b_frac);
				}
			}
			else
			{
				// here we can forego the extra vector and scalar multiplications that are necessary for lerp
				for(unordered_map<unsigned int, BoneInfluence>::iterator iter = cur.values.begin(); iter != cur.values.end(); ++iter)
				{
					BoneInfluence vec = cur.values[iter->first];
					SetBonePose(iter->first, vec.ori, vec.pos);
				}
			}
		}
		else
			SetActive(false);
	}

	void KeyframeAnimation::JumpToTime(float time)
	{
		if(time >= 0)
		{
			current_index = 0;
			while(true)
			{
				if(frames[current_index].duration > time)
				{
					current_time = time;
					break;
				}
				else
				{
					time -= frames[current_index].duration;
					++current_index;
					if(current_index == frames.size())
						current_index = 0;
				}
			};
		}
		else if(time < 0)
		{
			current_index = frames.size() - 1;
			while(true)
			{
				if(frames[current_index].duration > -time)
				{
					current_time = frames[current_index].duration + time;
					break;
				}
				else
				{
					time += frames[current_index].duration;
					--current_index;
					if(current_index < 0)
						current_index = frames.size() - 1;
				}
			}
		}
	}
}
