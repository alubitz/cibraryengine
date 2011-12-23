#include "StdAfx.h"
#include "KeyframeAnimation.h"

#include "Serialize.h"

namespace CibraryEngine
{
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
		if (current_index == -1)
			return false;
		else
		{
			current_time += time.elapsed;

			Keyframe& current = frames[current_index];

			// controls how much lag the animation system is willing to put up with, measured in seconds
			const float really_far_behind = 1.0f;
			// if the animation is more than this far behind (in seconds), don't keep advancing through all the frames in between
			if (current_time < really_far_behind + current.duration)
				while (current_time > current.duration)
				{
					current_time -= current.duration;
					current_index = frames[current_index].next;
					
					// if the animation finishes, quit early
					if (current_index == -1)
						return false;
				}
			else
			{
				// if it IS really far behind, advance exactly 1 frame
				current_time = 0;
				current_index = frames[current_index].next;

				if (current_index == -1)
					return false;
			}

			return true;
		}
	}

	void KeyframeAnimation::UpdatePose(TimingInfo time)
	{
		if (Advance(time))
		{
			Keyframe cur = frames[current_index];

			// linearly interpolate (lerp) only if this is not the final frame of the animation
			// TODO: maybe make it possible to force interpolation off? for certain keyframes only?
			bool interpolate = cur.next != -1;

			if (interpolate)
			{
				Keyframe nxt = frames[cur.next];                                      // the next frame in the animation

				float b_frac = current_time / cur.duration, a_frac = 1 - b_frac;                   // lerp coefficients

				list<unsigned int> cur_names = list<unsigned int>();
				list<unsigned int> next_names = list<unsigned int>();
				for(map<unsigned int, BoneInfluence>::iterator iter = cur.values.begin(); iter != cur.values.end(); iter++)
					cur_names.push_back(iter->first);
				for(map<unsigned int, BoneInfluence>::iterator iter = nxt.values.begin(); iter != nxt.values.end(); iter++)
					next_names.push_back(iter->first);

				// find the list of bone names used in both bones...
				list<unsigned int> shared_names = list<unsigned int>(cur_names);
				for(list<unsigned int>::iterator iter = shared_names.begin(); iter != shared_names.end();)
				{
					bool found = false;
					for(list<unsigned int>::iterator jter = next_names.begin(); jter != next_names.end(); jter++)
						if(*jter == *iter)
						{
							found = true;
							break;
						}
					if(!found)
						iter = shared_names.erase(iter);
					else
						iter++;
				}
				// ...and remove those names from the other two lists
				for(list<unsigned int>::iterator iter = cur_names.begin(); iter != cur_names.end();)
				{
					bool found = false;
					for(list<unsigned int>::iterator jter = shared_names.begin(); jter != shared_names.end(); jter++)
						if(*jter == *iter)
						{
							found = true;
							break;
						}
					if(found)
						iter = cur_names.erase(iter);
					else
						iter++;
				}
				for(list<unsigned int>::iterator iter = next_names.begin(); iter != next_names.end();)
				{
					bool found = false;
					for(list<unsigned int>::iterator jter = shared_names.begin(); jter != shared_names.end(); jter++)
						if(*jter == *iter)
						{
							found = true;
							break;
						}
					if(found)
						iter = next_names.erase(iter);
					else
						iter++;
				}

				for (list<unsigned int>::iterator iter = cur_names.begin(); iter != cur_names.end(); iter++)
				{
					BoneInfluence vec = cur.values[*iter];
					SetBonePose(*iter, vec.ori * a_frac, vec.pos * a_frac, vec.div * a_frac);
				}
				for (list<unsigned int>::iterator iter = next_names.begin(); iter != next_names.end(); iter++)
				{
					BoneInfluence vec = nxt.values[*iter];
					SetBonePose(*iter, vec.ori * b_frac, vec.pos * b_frac, vec.div * b_frac);
				}
				for (list<unsigned int>::iterator iter = shared_names.begin(); iter != shared_names.end(); iter++)
				{
					BoneInfluence cur_vec = cur.values[*iter];
					BoneInfluence next_vec = nxt.values[*iter];
					SetBonePose(*iter, cur_vec.ori * a_frac + next_vec.ori * b_frac, cur_vec.pos * a_frac + next_vec.pos * b_frac, cur_vec.div * a_frac + next_vec.div * b_frac);
				}
			}
			else
			{
				// here we can forego the extra vector and scalar multiplications that are necessary for lerp
				for(map<unsigned int, BoneInfluence>::iterator iter = cur.values.begin(); iter != cur.values.end(); iter++)
				{
					BoneInfluence vec = cur.values[iter->first];
					SetBonePose(iter->first, vec.ori, vec.pos, vec.div);
				}
			}
		}
		else
			SetActive(false);
	}
}
