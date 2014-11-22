#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "NeuralNet.h"

#include "ScaledIOBrain.h"

#define NUM_MIDDLE_LAYER_NEURONS   50
#define BASE_LEARNING_RATE         0.002f

namespace Test
{
	/*
	 * ExperimentalScreen private implementation struct
	 */
	struct ExperimentalScreen::Imp
	{
		ScaledIOBrain* siob;

		ExperimentalScreen* scr;
		vector<AutoMenuItem*> auto_menu_items;

		AutoMenuItem* output_text[6];

		struct BackButton : public AutoMenuItem
		{
			BackButton(ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		struct ShakeButton : public AutoMenuItem
		{
			ShakeButton(ContentMan* content, int row) : AutoMenuItem(content, "Shake", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { ((ExperimentalScreen*)mse->menu)->imp->Shake(); }
		};

		Imp(ExperimentalScreen* scr) : scr(scr), auto_menu_items()
		{
			// init screen stuff
			ContentMan* content = scr->content;

			unsigned int row = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Experimental Screen", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", row++, false));

			for(unsigned int i = 0; i < 6; ++i)
				auto_menu_items.push_back(output_text[i] = new AutoMenuItem(content, "", row++, false));

			auto_menu_items.push_back(new ShakeButton(content, row++));
			auto_menu_items.push_back(new BackButton(content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				scr->AddItem(auto_menu_items[i]);

			// init brain stuff
			LoadLogs();

			step = 0;

			mavg = -1;

			if(state_floats > 0)
			{
				siob = new ScaledIOBrain(NeuralNet::New(state_floats, 21, NUM_MIDDLE_LAYER_NEURONS));
				siob->nn->Randomize(0.1f);

				siob->input_scales.clear();
				siob->input_scales.insert(siob->input_scales.end(), state_scales.begin(), state_scales.end());

				siob->output_scales.clear();
				siob->output_scales.insert(siob->output_scales.end(), out_scales.begin(), out_scales.end());
			}
			else
				siob = NULL;
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

			if(siob != NULL)
			{
				SaveNeuralNet();

				delete siob;
				siob = NULL;
			}
		}

		struct TransIndices
		{
			unsigned int state1, state2;

			float importance;
		};

		unsigned int step;
		unsigned int state_floats;
		vector<vector<float>> states;
		vector<vector<float>> outputs;
		vector<TransIndices> indices;
		vector<float> state_scales;
		vector<float> out_scales;


		vector<float> scores_before;
		vector<float> scores_after;

		float mavg;

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

				scale = scale > 0 ? 0.95f / scale : 1.0f;			// catch NAN, etc.
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
			{
				Debug("Unable to load state transitions logfile!\n");
				/*trans_floats =*/ state_floats = 0;
			}
			else
			{
				unsigned int bone_floats      = ReadUInt32(file);
				unsigned int num_bones        = ReadUInt32(file);

				unsigned int cp_floats        = ReadUInt32(file);
				unsigned int max_cps_per_foot = ReadUInt32(file);
				unsigned int num_cps          = ReadUInt32(file);

				state_floats = bone_floats * num_bones + cp_floats * num_cps;

				unsigned int num_state_entries = ReadUInt32(file);
				for(unsigned int i = 0; i < num_state_entries; ++i)
				{
					vector<float> data(state_floats);
					for(unsigned int j = 0; j < state_floats; ++j)
						data[j] = ReadSingle(file);
					states.push_back(data);
				}

				// set up indices
				unsigned int num_trans_entries = ReadUInt32(file);
				for(unsigned int i = 0; i < num_trans_entries; ++i)
				{
					TransIndices index;
					index.state1 = ReadUInt32(file);
					index.state2 = ReadUInt32(file);
					indices.push_back(index);
				}

				file.close();
				

				// find output vector to match state2 vectors
				outputs.clear();
				for(unsigned int i = 0; i < num_state_entries; ++i)
				{
					vector<float> data;
					for(unsigned int j = 0; j < num_trans_entries; ++j)
						if(indices[j].state2 == i)
						{
							for(unsigned int k = 0; k < 7; ++k)
								data.push_back(states[i][k]);
							for(unsigned int k = 0; k < 7; ++k)
								data.push_back(states[i][k + 13]);
							for(unsigned int k = 0; k < 7; ++k)
								data.push_back(states[i][k + 13 * 6]);

							break;
						}
					if(data.empty())
						data.resize(21);
					outputs.push_back(data);
				}

				// scale everything to an appropriate range
				SetTableScale(states,  state_scales,  state_floats);
				SetTableScale(outputs, out_scales,    21);

				float importance_tot = 0.0f;
				for(unsigned int i = 0; i < indices.size(); ++i)
				{
					indices[i].importance = GetIndexTTL(i) < 2.0f ? 20.0f : 1.0f;
					importance_tot += indices[i].importance;
				}
				importance_tot = indices.size() / importance_tot;
				for(unsigned int i = 0; i < indices.size(); ++i)
					indices[i].importance *= importance_tot;
			}
			
			string str = ((stringstream&)(stringstream() << "dataset contains " << states.size() << " state records, between which there are " << indices.size() << " transitions" << endl)).str();
			output_text[0]->SetText(str);
			Debug(str);
		}

		float GetIndexTTL(unsigned int index_index) const
		{
			float ttl = 0.0f;
			float children = 0.0f;
			TransIndices t = indices[index_index];
			for(unsigned int i = 0; i < indices.size(); ++i)
				if(indices[i].state1 == t.state2)
				{
					ttl += GetIndexTTL(i);
					++children;
				}
			return children == 0 ? 0.0f : ttl / children;
		}

		void Shake()
		{
			if(siob == NULL)
				return;

			static const float shake_amount = 0.025f;

			for(float *fptr = siob->nn->top_matrix, *fend = fptr + siob->nn->top_matrix_size; fptr != fend; ++fptr)
				*fptr += Random3D::Rand(-shake_amount, shake_amount);
			for(float *fptr = siob->nn->bottom_matrix, *fend = fptr + siob->nn->bottom_matrix_size; fptr != fend; ++fptr)
				*fptr += Random3D::Rand(-shake_amount, shake_amount);
		}


		void PushInputFloats(unsigned int size, float*& input_ptr, const vector<float>& source)
		{
			memcpy(input_ptr, source.data(), size * sizeof(float));
			input_ptr += size;
		}

		void Update(const TimingInfo& time)
		{
			if(siob == NULL)
				return;

			unsigned int records_per_pass = indices.size();
			
			unsigned int pass   = step / records_per_pass;
			unsigned int record = step % records_per_pass;

			if(record == 0)
			{
				shuffle.clear();
				shuffle.resize(records_per_pass);

				for(unsigned int i = 1; i < records_per_pass; ++i)
					shuffle[i] = i;
				for(unsigned int i = 0; i < records_per_pass; ++i)
					swap(shuffle[i], shuffle[Random3D::RandInt(records_per_pass)]);
			}

			// deliberately empty for loop
			unsigned int use_record = shuffle[record];
			const TransIndices& index = indices[use_record];

			float* in_ptr = siob->nn->inputs;
			PushInputFloats(state_floats, in_ptr, states[index.state1]);
			
			memcpy(siob->nn->correct_outputs, outputs[index.state2].data(), 21 * sizeof(float));

			float use_learning_rate = index.importance * BASE_LEARNING_RATE * (mavg > 1.0f ? 1.0f + 2.0f * (mavg - 1.0f) : 1.0f);
			float score_before = siob->nn->Train(use_learning_rate);
			float score_after  = siob->nn->EvaluateAndScore();

			if(scores_before.empty())
				scores_before.resize(records_per_pass);
			if(scores_after.empty())
				scores_after.resize(records_per_pass);

			float last = scores_after[use_record];
			scores_before[use_record] = score_before;
			scores_after [use_record] = score_after;

			float mavg_tot  = 0.0f;
			float mavg_wtot = 0.0f;
			if(pass == 0)
				for(unsigned int i = 0; i <= record; ++i)
				{
					float weight = indices[shuffle[i]].importance;
					mavg_tot  += weight * scores_before[shuffle[i]];
					mavg_wtot += weight;
				}
			else
				for(unsigned int i = 0; i < records_per_pass; ++i)
				{
					float weight = indices[i].importance;
					mavg_tot  += weight * scores_before[i];
					mavg_wtot += weight;
				}
			mavg = mavg_tot / mavg_wtot;

			output_text[1]->SetText(((stringstream&)(stringstream() << "pass = " << pass << "; record[" << record << "] = " << use_record)).str());
			output_text[2]->SetText(((stringstream&)(stringstream() << "last   = " << last)).str());
			output_text[3]->SetText(((stringstream&)(stringstream() << "before = " << score_before)).str());
			output_text[4]->SetText(((stringstream&)(stringstream() << "after  = " << score_after)).str());
			output_text[5]->SetText(((stringstream&)(stringstream() << "recent \"before\" avg = " << mavg)).str());
			Debug(((stringstream&)(stringstream() << "pass = " << pass << "; record[" << record << "] = " << use_record << "; last = " << last << "; before = " << score_before << "; after = " << score_after << "; recent \"before\" avg = " << mavg << endl)).str());

			++step;
		}



		void SaveNeuralNet() const
		{
			time_t raw_time;
			time(&raw_time);
			tm now = *localtime(&raw_time);

			string filename = ((stringstream&)(stringstream() << "Files/Brains/neuralnet-" << now.tm_year + 1900 << "-" << now.tm_mon + 1 << "-" << now.tm_mday << "-" << now.tm_hour << "-" << now.tm_min << "-" << now.tm_sec << ".brain")).str();

			ofstream file(filename, ios::out | ios::binary);
			if(!file)
				Debug("Failed to save brain!\n");
			else
			{
				siob->Write(file);
				file.close();
			}
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
