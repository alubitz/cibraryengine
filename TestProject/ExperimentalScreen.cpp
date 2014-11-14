#include "StdAfx.h"
#include "ExperimentalScreen.h"

#include "NeuralNet.h"

#include "ScaledIOBrain.h"

#define NUM_MIDDLE_LAYER_NEURONS   15
#define LEARNING_RATE              0.005f

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

		AutoMenuItem* outputs[6];

		struct BackButton : public AutoMenuItem
		{
			BackButton (ContentMan* content, int row) : AutoMenuItem(content, "Back", row, true) { }
			void DoAction(MenuSelectionEvent* mse) { mse->menu->SetNextScreen(mse->menu->GetPreviousScreen()); }
		};

		Imp(ExperimentalScreen* scr) : scr(scr), auto_menu_items()
		{
			// init screen stuff
			ContentMan* content = scr->content;

			unsigned int row = 0;
			auto_menu_items.push_back(new AutoMenuItem(content, "Experimental Screen", row++, false));
			auto_menu_items.push_back(new AutoMenuItem(content, "-------------------------------------------------------", row++, false));

			for(unsigned int i = 0; i < 6; ++i)
				auto_menu_items.push_back(outputs[i] = new AutoMenuItem(content, "", row++, false));

			auto_menu_items.push_back(new BackButton(content, row++));

			for(unsigned int i = 0; i < auto_menu_items.size(); ++i)
				scr->AddItem(auto_menu_items[i]);

			// init brain stuff
			LoadLogs();

			step = 0;

			siob = new ScaledIOBrain(NeuralNet::New((state_floats + trans_floats) * 2, state_floats, NUM_MIDDLE_LAYER_NEURONS));
			siob->nn->Randomize(0.1f);

			siob->input_scales.clear();
			siob->input_scales.insert(siob->input_scales.end(), state_scales.begin(), state_scales.end());
			siob->input_scales.insert(siob->input_scales.end(), trans_scales.begin(), trans_scales.end());
			siob->input_scales.insert(siob->input_scales.end(), state_scales.begin(), state_scales.end());
			siob->input_scales.insert(siob->input_scales.end(), trans_scales.begin(), trans_scales.end());

			siob->output_scales.clear();
			siob->output_scales.insert(siob->output_scales.end(), state_scales.begin(), state_scales.end());
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

			SaveNeuralNet();
			if(siob) { delete siob; siob = NULL; }
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
			outputs[0]->SetText(str);
			Debug(str);
		}

		

		void PushInputFloats(unsigned int size, float*& input_ptr, const vector<float>& source)
		{
			memcpy(input_ptr, source.data(), size * sizeof(float));
			input_ptr += size;
		}

		void Update(const TimingInfo& time)
		{
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
			PushInputFloats(trans_floats, in_ptr, trans [index.trans1]);
			PushInputFloats(state_floats, in_ptr, states[index.state2]);
			PushInputFloats(trans_floats, in_ptr, trans [index.trans2]);

			memcpy(siob->nn->correct_outputs, states[index.state3].data(), state_floats * sizeof(float));

			float score_before = siob->nn->Train(LEARNING_RATE);
			float score_after  = siob->nn->EvaluateAndScore();

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

			outputs[1]->SetText(((stringstream&)(stringstream() << "pass = " << pass << "; record[" << record << "] = " << use_record)).str());
			outputs[2]->SetText(((stringstream&)(stringstream() << "last   = " << last)).str());
			outputs[3]->SetText(((stringstream&)(stringstream() << "before = " << score_before)).str());
			outputs[4]->SetText(((stringstream&)(stringstream() << "after  = " << score_after)).str());
			outputs[5]->SetText(((stringstream&)(stringstream() << "recent \"before\" avg = " << mavg)).str());
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
