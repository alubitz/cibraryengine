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

		vector<float> inputs;
		vector<float> top_matrix;
		vector<float> middle_sums;
		vector<float> middles;
		vector<float> bottom_matrix;
		vector<float> output_sums;
		vector<float> outputs;
		vector<float> correct_outputs;
		vector<float> errors;
		vector<float> temp_tops;
		vector<float> temp_bottoms;

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

			inputs         .resize(num_inputs );
			middle_sums    .resize(num_middles);
			middles        .resize(num_middles);
			output_sums    .resize(num_outputs);
			outputs        .resize(num_outputs);
			correct_outputs.resize(num_outputs);
			errors         .resize(num_outputs);

			top_matrix   .resize(num_inputs  * num_middles);
			bottom_matrix.resize(num_middles * num_outputs);

			temp_tops   .resize(num_inputs  * num_middles);
			temp_bottoms.resize(num_middles * num_outputs);

			float random_range = 0.5f;
			for(vector<float>::iterator iter = top_matrix.begin();    iter != top_matrix.end();    ++iter)
				*iter = Random3D::Rand(-random_range, random_range);
			for(vector<float>::iterator iter = bottom_matrix.begin(); iter != bottom_matrix.end(); ++iter)
				*iter = Random3D::Rand(-random_range, random_range);
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

		void Evaluate(const float* top_matrix, const float* bottom_matrix, const float* inputs, float* middle_sums, float* middles, float* output_sums, float* outputs)
		{
			Multiply(top_matrix,    inputs,  middle_sums, num_inputs,  num_middles);
			Sigmoid(middle_sums, middles, num_middles);
			Multiply(bottom_matrix, middles, output_sums, num_middles, num_outputs);
			Sigmoid(output_sums, outputs, num_outputs);
		}

		float EvaluateAndScore(const float* top_matrix, const float* bottom_matrix, const float* inputs, float* middle_sums, float* middles, float* output_sums, float* outputs, const float* correct, float* errors)
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

		void DoVariations(float* matrix, unsigned int size, const float* inputs, float* middle_sums, float* middles, float* output_sums, float* outputs, const float* correct_outputs, float learning_rate)
		{
			static const float dx = 0.000001f;

			float* top_data    = top_matrix.data();
			float* bottom_data = bottom_matrix.data();

			for(float *mat_ptr = matrix, *mat_end = mat_ptr + size; mat_ptr != mat_end; ++mat_ptr)
			{
				float y1 = EvaluateAndScore(top_data, bottom_data, inputs, middle_sums, middles, output_sums, outputs, correct_outputs, errors.data());

				float x1 = *mat_ptr;
				*mat_ptr += dx;

				float y2 = EvaluateAndScore(top_data, bottom_data, inputs, middle_sums, middles, output_sums, outputs, correct_outputs, errors.data());
				float dy = y2 - y1;

				*mat_ptr = x1 - learning_rate * dy / dx;
			}
		}

		// outputs will not contain [particularly] useful results!
		float Train(float learning_rate, const float* inputs, float* middle_sums, float* middles, float* output_sums, float* outputs, const float* correct_outputs, float* errors)
		{
			float initial_error = EvaluateAndScore(top_matrix.data(), bottom_matrix.data(), inputs, middle_sums, middles, output_sums, outputs, correct_outputs, errors);

			//DoVariations(bottom_matrix.data(), num_middles * num_outputs, inputs, middle_sums, middles, output_sums, outputs, correct_outputs, learning_rate);
			//DoVariations(top_matrix.data(),    num_inputs  * num_middles, inputs, middle_sums, middles, output_sums, outputs, correct_outputs, learning_rate);

			// x = inputs, a = middle sums, y = middles, b = output sums, z = outputs
			for(unsigned int i = 0; i < num_outputs; ++i)
			{
				float phiprime_bi = 1.0f - outputs[i] * outputs[i];
				for(unsigned int j = 0; j < num_middles; ++j)
				{
					float dzidnji = phiprime_bi * middles[j];
					float depsdnji = 2.0f * errors[i] * dzidnji;

					temp_bottoms[i * num_middles + j] = bottom_matrix[i * num_middles + j] - learning_rate * depsdnji;
				}
			}

			for(unsigned int k = 0; k < num_inputs; ++k)
			{
				for(unsigned int j = 0; j < num_middles; ++j)
				{
					float depsdmkj = 0.0f;
					for(unsigned int i = 0; i < num_outputs; ++i)
					{
						float phiprime_bi = 1.0f - outputs[i] * outputs[i];
						float dzidmkj = 0.0f;
						for(unsigned int jj = 0; jj < num_middles; ++jj)
						{
							float phiprime_aj = 1.0f - middles[jj] * middles[jj];
							float dyjdmkj = phiprime_aj * inputs[k];
							dzidmkj += bottom_matrix[i * num_middles + jj] * dyjdmkj;
						}
						dzidmkj *= phiprime_bi;
						depsdmkj += errors[i] * dzidmkj;
					}
					depsdmkj *= 2.0f;
					temp_tops[j * num_inputs + k] = top_matrix[j * num_inputs + k] - learning_rate * depsdmkj;
				}
			}

			memcpy(bottom_matrix.data(), temp_bottoms.data(), num_middles * num_outputs * sizeof(float));
			memcpy(top_matrix.data(),    temp_tops.data(),    num_inputs  * num_middles * sizeof(float));

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

				for(unsigned int i = 1; i < records_per_pass; ++i)
				{
					unsigned int j = Random3D::RandInt() % records_per_pass;
					while(shuffle[j] != 0)
						j = (j + 1) % records_per_pass;
					shuffle[j] = i;
				}
			}

			for(vector<float>::iterator iter = top_matrix.begin();    iter != top_matrix.end();    ++iter)
				MaybeRandomizeCoefficient(*iter);
			for(vector<float>::iterator iter = bottom_matrix.begin(); iter != bottom_matrix.end(); ++iter)
				MaybeRandomizeCoefficient(*iter);

			// deliberately empty for loop
			unsigned int use_record = shuffle[record];
			const TransIndices& index = indices[use_record];

			memcpy(inputs.data(),                                   states[index.state1].data(), state_floats * sizeof(float));
			memcpy(inputs.data() + state_floats,                    trans [index.trans1].data(), trans_floats * sizeof(float));
			memcpy(inputs.data() + state_floats     + trans_floats, states[index.state2].data(), state_floats * sizeof(float));
			memcpy(inputs.data() + state_floats * 2 + trans_floats, trans [index.trans2].data(), trans_floats * sizeof(float));
			memcpy(correct_outputs.data(),                          states[index.state3].data(), state_floats * sizeof(float));

			float score_before = Train           ( learning_rate,                           inputs.data(), middle_sums.data(), middles.data(), output_sums.data(), outputs.data(), correct_outputs.data(), errors.data() );
			float score_after  = EvaluateAndScore( top_matrix.data(), bottom_matrix.data(), inputs.data(), middle_sums.data(), middles.data(), output_sums.data(), outputs.data(), correct_outputs.data(), errors.data() );

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
