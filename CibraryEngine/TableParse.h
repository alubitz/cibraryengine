#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "Quaternion.h"
#include "Vector.h"

namespace CibraryEngine
{
	using namespace std;

	class TableParseable : public Disposable
	{
		protected:
			virtual void InnerDispose();
		public:
			TableParseable();
			virtual TableParseable* DoOperation(string a, string b = "") = 0;
			virtual void End() = 0;
	};

	/** Class for parsing tables */
	class TableParserInstance : public TableParseable
	{
		protected:

			virtual void InnerDispose();

			enum ParserState
			{
				StartOfLine,
				Comment,
				StackDepth,
				VariableNameStart,
				VariableName,
				PostVariable,
				ArgumentValue,
				PostArgument,
				Error
			} mode;
			istream* stream;

			vector<TableParseable*> stack;

			unsigned int stack_depth;
			string variable_name;
			string argument_value;
			bool in_string_literal;

			void DoOperationUsingOperands(string a, string b);

		public:

			// constructor
			TableParserInstance(istream* stream);

			/** The almighty "go" button for the table parser **/
			void ParseTable();

			bool ParseLine();
			void ParseChar(char c);

			virtual bool CanContinue();

			virtual TableParseable* DoOperation(string a, string b);
			virtual void End();

			void ParserException(string message);
	};

	class NamedItemDictionaryTableParser : public TableParserInstance
	{
		protected:

			virtual void InnerDispose();

		public:

			class FieldSetter { public: virtual TableParseable* Set(string val) = 0; };

			map<string, FieldSetter*> field_setters;

			NamedItemDictionaryTableParser(istream* stream);

			virtual TableParseable* DoOperation(string a, string b);
			virtual TableParseable* OnFieldNotFound(string a, string b);
	};


	/*
	 * Math setters
	 */
	struct IntSetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		int* i;
		IntSetter(int* i);

		TableParseable* Set(string val);
	};

	struct FloatSetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		float* f;
		FloatSetter(float* f);

		TableParseable* Set(string val);
	};

	struct QuaternionSetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		Quaternion* quat;
		istream* stream;

		QuaternionSetter(Quaternion* quat, istream* stream);
		TableParseable* Set(string val);
	};

	struct Vec3Setter : public NamedItemDictionaryTableParser::FieldSetter
	{
		Vec3* vec;
		istream* stream;

		Vec3Setter(Vec3* vec, istream* stream);

		TableParseable* Set(string val);
	};
}
