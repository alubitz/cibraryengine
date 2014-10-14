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

		float moving_average;
		list<float> mavg_list;

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

			num_inputs  = 10;
			num_outputs = 4;
			num_middles = 50;

			inputs .resize(num_inputs);
			middles.resize(num_middles);
			outputs.resize(num_outputs);

			middle_sums.resize(num_middles);
			output_sums.resize(num_outputs);

			correct_outputs.resize(num_outputs);

			float random_range = 0.5f;

			top_matrix.resize(num_inputs * num_middles);
			for(vector<float>::iterator iter = top_matrix.begin(); iter != top_matrix.end(); ++iter)
				*iter = Random3D::Rand(-random_range, random_range);

			bottom_matrix.resize(num_middles * num_outputs);
			for(vector<float>::iterator iter = bottom_matrix.begin(); iter != bottom_matrix.end(); ++iter)
				*iter = Random3D::Rand(-random_range, random_range);

			moving_average = 0;
			mavg_list.clear();
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

		void DoVariations(float* matrix, unsigned int size, const float* inputs, float* middle_sums, float* middles, float* output_sums, float* outputs, const float* correct_outputs, float learning_rate)
		{
			static const float dx = 0.000001f;

			for(float *mat_ptr = matrix, *mat_end = mat_ptr + size; mat_ptr != mat_end; ++mat_ptr)
			{
				Evaluate(top_matrix.data(), bottom_matrix.data(), inputs, middle_sums, middles, output_sums, outputs);
				float y1 = CheckOutput(outputs, correct_outputs, num_outputs);

				float x1 = *mat_ptr;
				*mat_ptr += dx;

				Evaluate(top_matrix.data(), bottom_matrix.data(), inputs, middle_sums, middles, output_sums, outputs);
				float y2 = CheckOutput(outputs, correct_outputs, num_outputs);
				float dy = y2 - y1;

				*mat_ptr = x1 - learning_rate * dy / dx;
			}
		}

		// outputs will not contain [particularly] useful results!
		float Train(const float* inputs, float* middle_sums, float* middles, float* output_sums, float* outputs, const float* correct_outputs, float learning_rate)
		{
			Evaluate(top_matrix.data(), bottom_matrix.data(), inputs, middle_sums, middles, output_sums, outputs);
			float initial_error = CheckOutput(outputs, correct_outputs, num_outputs);

			DoVariations(bottom_matrix.data(), num_middles * num_outputs, inputs, middle_sums, middles, output_sums, outputs, correct_outputs, learning_rate);
			DoVariations(top_matrix.data(),    num_inputs  * num_middles, inputs, middle_sums, middles, output_sums, outputs, correct_outputs, learning_rate);

			//Evaluate(top_matrix.data(), bottom_matrix.data(), inputs, middle_sums, middles, output_sums, outputs);
			//return CheckOutput(outputs, correct_outputs, num_outputs);

			return initial_error;
		}

		void MaybeRandomizeCoefficient(float& coeff)
		{
			if(Random3D::RandInt() % 100 == 0)
				coeff += Random3D::Rand(-0.001f, 0.001f);
		}

		void Update(const TimingInfo& time)
		{
			static const unsigned int count    = 1;
			static const unsigned int mavg_max = 200;

			static const float domain          = 0.7f;
			static const float learning_rate   = 0.005f;

			for(vector<float>::iterator iter = top_matrix.begin(); iter != top_matrix.end(); ++iter)
				MaybeRandomizeCoefficient(*iter);
			for(vector<float>::iterator iter = bottom_matrix.begin(); iter != bottom_matrix.end(); ++iter)
				MaybeRandomizeCoefficient(*iter);

			float tot = 0.0f;
			for(unsigned int i = 0; i < count; ++i)
			{
				Vec3 a = Random3D::RandomNormalizedVector(Random3D::Rand(domain));
				Vec3 b = Random3D::RandomNormalizedVector(Random3D::Rand(domain));
				Vec3 c = Random3D::RandomNormalizedVector(Random3D::Rand(domain));

				Vec3 cross = Vec3::Cross(b - a, c - a);
				float xmag = cross.ComputeMagnitude();
				Vec3 d = xmag > 0.05f ? Vec3() : cross * (0.7f / xmag);

				inputs[0] = 1.0f;
				inputs[1] = a.x;
				inputs[2] = a.y;
				inputs[3] = a.z;
				inputs[4] = b.x;
				inputs[5] = b.y;
				inputs[6] = b.z;
				inputs[7] = c.x;
				inputs[8] = c.y;
				inputs[9] = c.z;

				correct_outputs[0] = d.x;
				correct_outputs[1] = d.y;
				correct_outputs[2] = d.z;
				correct_outputs[3] = Vec3::Dot(d, a);

				tot += Train(inputs.data(), middle_sums.data(), middles.data(), output_sums.data(), outputs.data(), correct_outputs.data(), learning_rate);
			}

			float avg = tot / count;

			moving_average *= mavg_list.size();
			moving_average += avg;

			mavg_list.push_back(avg);
			if(mavg_list.size() > mavg_max)
			{
				moving_average -= *mavg_list.begin();
				mavg_list.pop_front();
			}
			moving_average /= mavg_list.size();

			output1->SetText(((stringstream&)(stringstream() << "avg = " << moving_average)).str());
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
