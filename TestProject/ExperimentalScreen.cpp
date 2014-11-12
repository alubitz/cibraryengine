#include "StdAfx.h"
#include "ExperimentalScreen.h"

namespace Test
{
	/*
	 * ExperimentalScreen private implementation struct
	 */
	struct ExperimentalScreen::Imp
	{
		unsigned int num_inputs, num_outputs, num_middles;

		vector<float> lots_of_floats;

		float* inputs;
		float* top_matrix;
		float* middle_sums;
		float* middles;
		float* phiprime_mids;
		float* bottom_matrix;
		float* output_sums;
		float* outputs;
		float* phiprime_outs;
		float* correct_outputs;
		float* errors;
		float* temp_tops;
		float* temp_bottoms;

		ExperimentalScreen* scr;
		vector<AutoMenuItem*> auto_menu_items;

		AutoMenuItem *output1, *output2;

		struct BackButton : public AutoMenuItem
		{
			BackButton (ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		Imp(ExperimentalScreen* scr) : scr(scr), auto_menu_items(), output1(NULL), output2(NULL)
		{
			ContentMan* content = scr->content;

			unsigned int row = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Experimental Screen", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", row++, false));

			auto_menu_items.push_back(output1 = new AutoMenuItem(content, "", row++, false));
			auto_menu_items.push_back(output2 = new AutoMenuItem(content, "", row++, false));

			auto_menu_items.push_back(new BackButton(content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				scr->AddItem(auto_menu_items[i]);

			InitBrainStuff();
		}

		~Imp()
		{
			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
			{
				AutoMenuItem* item = auto_menu_items[i];

				item->Dispose();
				delete item;
			}
			auto_menu_items.clear();
		}

		struct TransIndices
		{
			unsigned int state1, state2, state3;
			unsigned int trans1, trans2;
		};

		unsigned int step;
		unsigned int state_floats, trans_floats;
		vector<vector<float>> states;
		vector<vector<float>> trans;
		vector<unsigned int> froms;
		vector<unsigned int> tos;
		vector<TransIndices> indices;
		vector<float> state_scales;
		vector<float> trans_scales;
		vector<float> scores_before;
		vector<float> scores_after;

		vector<unsigned int> shuffle;

		static void SetTableScale(vector<vector<float>>& data, vector<float>& scales, unsigned int entry_floats)
		{
			scales.resize(entry_floats);
			for(unsigned int i = 0; i < entry_floats; ++i)
			{
				float& scale = scales[i];
				
				scale = 1.0f;
				for(vector<vector<float>>::iterator iter = data.begin(); iter != data.end(); ++iter)
					scale = max(scale, fabs((*iter)[i]));

				scale = scale > 0 ? 1.0f / scale : 1.0f;			// catch NAN, etc.
			}
			for(vector<vector<float>>::iterator iter = data.begin(); iter != data.end(); ++iter)
				for(unsigned int i = 0; i < entry_floats; ++i)
				{
					float& datum = (*iter)[i];

					datum *= scales[i];
					if(!(datum >= -1.0f && datum <= 1.0f))
					{
						Debug(((stringstream&)(stringstream() << "bad datum = " << datum << endl)).str());
						datum = 0.0f;
					}
				}
		}

		void LoadLogs()
		{
			ifstream file("Files/Logs/learnme.statelog", ios::in | ios::binary);
			if(!file)
				Debug("Unable to load state transitions logfile!\n");
			else
			{
				unsigned int bone_floats = ReadUInt32(file);
				unsigned int num_bones   = ReadUInt32(file);

				trans_floats         = ReadUInt32(file);
				state_floats         = bone_floats * num_bones;

				unsigned int num_state_entries = ReadUInt32(file);
				for(unsigned int i = 0; i < num_state_entries; ++i)
				{
					vector<float> data(state_floats);
					for(unsigned int j = 0; j < state_floats; ++j)
						data[j] = ReadSingle(file);
					states.push_back(data);
				}

				unsigned int num_trans_entries = ReadUInt32(file);
				for(unsigned int i = 0; i < num_trans_entries; ++i)
				{
					vector<float> data(trans_floats);
					froms.push_back(ReadUInt32(file));
					tos  .push_back(ReadUInt32(file));
					for(unsigned int j = 0; j < trans_floats; ++j)
						data[j] = ReadSingle(file);
					trans.push_back(data);
				}

				file.close();

				// scale everything to an appropriate range
				SetTableScale(states, state_scales, state_floats);
				SetTableScale(trans,  trans_scales, trans_floats);

				// set up indices
				for(unsigned int i = 0; i < num_trans_entries; ++i)
				{
					TransIndices index;
					index.state1 = froms[i];
					index.trans1 = i;
					index.state2 = tos[i];

					for(unsigned int j = 0; j < num_trans_entries; ++j)
						if(froms[j] == tos[i])
						{
							index.trans2 = j;
							index.state3 = tos[j];

							indices.push_back(index);
						}
				}
			}
			
			string str = ((stringstream&)(stringstream() << "dataset contains " << trans.size() << " state transition records, within which there are " << indices.size() << " unique 2-transition samples" << endl)).str();
			output1->SetText(str);
			Debug(str);
		}

		void InitBrainStuff()
		{
			LoadLogs();

			step = 0;

			num_inputs  = (state_floats + trans_floats) * 2;
			num_outputs = state_floats;
			num_middles = 25;

			unsigned int total_size = 0;
			total_size += num_inputs;						// inputs
			total_size += num_middles * 3;					// middle sums, middles, phiprime mids
			total_size += num_outputs * 5;					// output sums, outputs, phiprime outs, correct outputs, and errors
			total_size += num_inputs * num_middles * 2;		// top matrix, temp tops
			total_size += num_middles * num_outputs * 2;	// bottom matrix, temp bottoms

			lots_of_floats.clear();
			lots_of_floats.resize(total_size);
			float* fptr = lots_of_floats.data();

			inputs          = fptr;    fptr += num_inputs;
			middle_sums     = fptr;    fptr += num_middles;
			middles         = fptr;    fptr += num_middles;
			phiprime_mids   = fptr;    fptr += num_middles;
			output_sums     = fptr;    fptr += num_outputs;
			outputs         = fptr;    fptr += num_outputs;
			phiprime_outs   = fptr;    fptr += num_outputs;
			correct_outputs = fptr;    fptr += num_outputs;
			errors          = fptr;    fptr += num_outputs;

			top_matrix      = fptr;    fptr += num_inputs * num_middles;
			temp_tops       = fptr;    fptr += num_inputs * num_middles;
			bottom_matrix   = fptr;    fptr += num_middles * num_outputs;
			temp_bottoms    = fptr;    fptr += num_middles * num_outputs;

			float random_range = 0.5f;
			for(unsigned int i = 0; i < num_inputs * num_middles; ++i)
				top_matrix[i] = Random3D::Rand(-random_range, random_range);
			for(unsigned int i = 0; i < num_middles * num_outputs; ++i)
				bottom_matrix[i] = Random3D::Rand(-random_range, random_range);
		}

		void Multiply(const float* matrix, const float* in_begin, float* out_begin, unsigned int num_in, unsigned int num_out)
		{
			const float* in_end  = in_begin  + num_in;
			const float* out_end = out_begin + num_out;
			const float* mat_ptr = matrix;

			float value;

			for(float* out_ptr = out_begin; out_ptr != out_end; ++out_ptr)
			{
				value = 0.0f;
				for(const float* in_ptr = in_begin; in_ptr != in_end; ++in_ptr, ++mat_ptr)
					value += *mat_ptr * *in_ptr;
				*out_ptr = value;
			}
		}

		void Sigmoid(const float* inputs, float* outputs, unsigned int count)
		{
			for(const float* in_end = inputs + count; inputs != in_end; ++inputs, ++outputs)
				*outputs = tanh(*inputs);
		}

		float CheckOutput(const float* outputs, const float* correct, unsigned int count)
		{
			float tot = 0.0f;
			for(const float *out_ptr = outputs, *correct_ptr = correct, *out_end = out_ptr + count; out_ptr != out_end; ++out_ptr, ++correct_ptr)
			{
				float dif = *out_ptr - *correct_ptr;
				tot += dif * dif;
			}
			return tot;
		}

		inline void Evaluate(const float* top_matrix, const float* bottom_matrix, const float* inputs, float* middle_sums, float* middles, float* output_sums, float* outputs)
		{
			Multiply(top_matrix,    inputs,  middle_sums, num_inputs,  num_middles);
			Sigmoid(middle_sums, middles, num_middles);
			Multiply(bottom_matrix, middles, output_sums, num_middles, num_outputs);
			Sigmoid(output_sums, outputs, num_outputs);
		}

		inline float EvaluateAndScore(const float* top_matrix, const float* bottom_matrix, const float* inputs, float* middle_sums, float* middles, float* output_sums, float* outputs, const float* correct, float* errors)
		{
			Evaluate(top_matrix, bottom_matrix, inputs, middle_sums, middles, output_sums, outputs);
			
			float tot = 0.0f;
			const float* correct_ptr = correct;
			for(float *err_ptr = errors, *out_ptr = outputs, *out_end = out_ptr + num_outputs; out_ptr != out_end; ++err_ptr, ++out_ptr, ++correct_ptr)
			{
				*err_ptr = *out_ptr - *correct_ptr;
				tot += *err_ptr * *err_ptr;
			}
			return tot;
		}

		// outputs will not contain [particularly] useful results!
		float Train(float learning_rate, const float* inputs, float* middle_sums, float* middles, float* output_sums, float* outputs, const float* correct_outputs, float* errors)
		{
			float initial_error = EvaluateAndScore(top_matrix, bottom_matrix, inputs, middle_sums, middles, output_sums, outputs, correct_outputs, errors);

			float* ppo_end = phiprime_outs + num_outputs;
			float* ppm_end = phiprime_mids + num_middles;
			for(float *ppo_ptr = phiprime_outs, *outputs_ptr = outputs; ppo_ptr != ppo_end; ++ppo_ptr, ++outputs_ptr)
				*ppo_ptr = 1.0f - *outputs_ptr * *outputs_ptr;
			for(float *ppm_ptr = phiprime_mids, *middles_ptr = middles; ppm_ptr != ppm_end; ++ppm_ptr, ++middles_ptr)
				*ppm_ptr = 1.0f - *middles_ptr * *middles_ptr;

			float* temp_bottom_ptr = temp_bottoms;
			float* bottom_ptr      = bottom_matrix;
			float* middles_end     = middles + num_middles;
			for(float *ppo_ptr = phiprime_outs, *ppo_end = ppo_ptr + num_outputs, *err_ptr = errors; ppo_ptr != ppo_end; ++ppo_ptr, ++err_ptr)
				for(float* mid_ptr = middles; mid_ptr != middles_end; ++mid_ptr, ++temp_bottom_ptr, ++bottom_ptr)
					*temp_bottom_ptr = *bottom_ptr - learning_rate * 2.0f * *err_ptr * *ppo_ptr * *mid_ptr;

			float* temp_top_ptr = temp_tops;
			float* top_ptr      = top_matrix;
			const float* inputs_end   = inputs + num_inputs;
			for(const float* input_ptr = inputs; input_ptr != inputs_end; ++input_ptr)
				for(float* top_plus_middles = top_ptr + num_middles; top_ptr != top_plus_middles; ++top_ptr, ++temp_top_ptr)
				{
					bottom_ptr = bottom_matrix;

					float derrordtopcoeff = 0.0f;
					for(float *ppo_ptr = phiprime_outs, *err_ptr = errors; ppo_ptr != ppo_end; ++ppo_ptr, ++err_ptr)
					{
						float doutputdtopcoeff = 0.0f;
						for(float* ppm_ptr = phiprime_mids; ppm_ptr != ppm_end; ++ppm_ptr, ++bottom_ptr)
							doutputdtopcoeff += *bottom_ptr * *ppm_ptr * *input_ptr;
						derrordtopcoeff += *err_ptr * doutputdtopcoeff * *ppo_ptr;
					}
					derrordtopcoeff *= 2.0f;
					*temp_top_ptr = *top_ptr - learning_rate * derrordtopcoeff;
				}

			memcpy(bottom_matrix, temp_bottoms, num_middles * num_outputs * sizeof(float));
			memcpy(top_matrix,    temp_tops,    num_inputs  * num_middles * sizeof(float));

			return initial_error;
		}

		void MaybeRandomizeCoefficient(float& coeff)
		{
			static const unsigned int odds_against = 200;
			static const float        amount       = 0.0001f;

			if(Random3D::RandInt() % odds_against == 0)
				coeff += Random3D::Rand(-amount, amount);
		}

		

		void Update(const TimingInfo& time)
		{
			static const float learning_rate   = 0.035f;

			unsigned int records_per_pass = indices.size();
			
			unsigned int pass   = step / records_per_pass;
			unsigned int record = step % records_per_pass;

			if(record == 0)
			{
				shuffle.clear();
				shuffle.resize(records_per_pass);

				unsigned int* shuffle_begin = shuffle.data();
				unsigned int* shuffle_end = shuffle_begin + records_per_pass;
				for(unsigned int i = 1; i < records_per_pass; ++i)
				{
					unsigned int* shuffle_ptr = shuffle_begin + Random3D::RandInt() % records_per_pass;
					while(*shuffle_ptr)
					{
						++shuffle_ptr;
						if(shuffle_ptr == shuffle_end)
							shuffle_ptr = shuffle_begin;
					}
					*shuffle_ptr = i;
				}
			}

			for(float *tm_ptr = top_matrix, *tm_end = tm_ptr + num_inputs * num_middles; tm_ptr != tm_end; ++tm_ptr)
				MaybeRandomizeCoefficient(*tm_ptr);
			for(float *bm_ptr = top_matrix, *bm_end = bm_ptr + num_middles * num_outputs; bm_ptr != bm_end; ++bm_ptr)
				MaybeRandomizeCoefficient(*bm_ptr);

			// deliberately empty for loop
			unsigned int use_record = shuffle[record];
			const TransIndices& index = indices[use_record];

			memcpy(inputs,                                   states[index.state1].data(), state_floats * sizeof(float));
			memcpy(inputs + state_floats,                    trans [index.trans1].data(), trans_floats * sizeof(float));
			memcpy(inputs + state_floats     + trans_floats, states[index.state2].data(), state_floats * sizeof(float));
			memcpy(inputs + state_floats * 2 + trans_floats, trans [index.trans2].data(), trans_floats * sizeof(float));
			memcpy(correct_outputs,                          states[index.state3].data(), state_floats * sizeof(float));

			float score_before = Train           ( learning_rate,             inputs, middle_sums, middles, output_sums, outputs, correct_outputs, errors );
			float score_after  = EvaluateAndScore( top_matrix, bottom_matrix, inputs, middle_sums, middles, output_sums, outputs, correct_outputs, errors );

			if(scores_before.empty())
				scores_before.resize(records_per_pass);
			if(scores_after.empty())
				scores_after.resize(records_per_pass);

			float last = scores_after[use_record];
			scores_before[use_record] = score_before;
			scores_after [use_record] = score_after;

			float mavg_tot = 0.0f;
			if(pass == 0)
				for(unsigned int i = 0; i <= record; ++i)
					mavg_tot += scores_before[shuffle[i]];
			else
				for(unsigned int i = 0; i < records_per_pass; ++i)
					mavg_tot += scores_before[i];
			float mavg = pass == 0 ? mavg_tot / (record + 1) : mavg_tot / records_per_pass;

			string str = ((stringstream&)(stringstream() << "pass = " << pass << "; record[" << record << "] = " << use_record << "; last = " << last << "; before = " << score_before << "; after = " << score_after << "; recent \"before\" avg = " << mavg << endl)).str();
			output2->SetText(str);
			Debug(str);

			++step;
		}
	};




	/*
	 * ExperimentalScreen methods
	 */
	ExperimentalScreen::ExperimentalScreen(ProgramWindow* win, ProgramScreen* previous) : MenuScreen(win, previous), imp(NULL) { }

	void ExperimentalScreen::Activate()
	{
		MenuScreen::Activate();

		if(imp == NULL)
			imp = new Imp(this);
	}

	void ExperimentalScreen::Deactivate()
	{
		MenuScreen::Deactivate();

		if(imp) { delete imp; imp = NULL; }
	}

	ProgramScreen* ExperimentalScreen::Update(const TimingInfo& time) { imp->Update(time); return MenuScreen::Update(time); }
}
