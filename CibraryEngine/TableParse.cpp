#include "StdAfx.h"
#include "TableParse.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * TableParseable methods
	 */
	TableParseable::TableParseable() : Disposable() { }
	void TableParseable::InnerDispose() { Disposable::InnerDispose(); }




	/*
	 * TableParserInstance methods
	 */
	TableParserInstance::TableParserInstance(istream* stream) :
		TableParseable(),
		mode(StartOfLine),
		stream(stream),
		stack(),
		stack_depth(0),
		variable_name(),
		argument_value(),
		in_string_literal(false)
	{
	}

	void TableParserInstance::InnerDispose() { TableParseable::InnerDispose(); }

	void TableParserInstance::ParseTable()
	{
		while(ParseLine())
			continue;
		if(mode != Error)
		{
			while (stack.size() > 0)
			{
				TableParseable* popped = stack[stack.size() - 1];
				popped->End();

				stack.pop_back();
			}
		}
	}

	bool TableParserInstance::ParseLine()
	{
		char line_chars[256];
		if(stream->eof())
			return false;
		stream->getline(line_chars, 256);

		string line = string(line_chars);

		in_string_literal = false;

		for (unsigned int i = 0; i < line.length() && mode != Error; i++)
			ParseChar(line[i]);

		ParseChar('\n');

		bool no_error = mode != Error;
		mode = StartOfLine;
		return no_error || CanContinue();
	}

	bool TableParserInstance::CanContinue() { return false; }				// ported this from the C# version... no idea what it's for

	TableParseable* TableParserInstance::DoOperation(string a, string b) { return NULL; }				// default operation does nothing and does not push the stack

	void TableParserInstance::DoOperationUsingOperands(string a, string b = "")
	{
		while (stack_depth <= stack.size())
		{
			TableParseable* popped = stack[stack.size() - 1];
			popped->End();

			stack.pop_back();
		}

		TableParseable* op = stack.size() == 0 ? this : stack[stack.size() - 1];

		TableParseable* result = op->DoOperation(a, b);

		if (result != NULL)
			stack.push_back(result);
	}

	void TableParserInstance::ParseChar(char c)
	{
		switch (mode)
		{
			case Error:
				return;
			case StartOfLine:
				if (c == '\n' || c == '~')      // end of line
				{
					mode = Comment;
					return;
				}
				else if (isspace(c))
					return;
				else if (c == ':')              // stack depth indicated by a string of colons
				{
					mode = StackDepth;
					stack_depth = 1;
					return;
				}
				else
				{
					ParserException("the first non-whitespace character of a line must either be : or ~");
					return;
				}

			case Comment:
				// ignore the text of comments
				return;

			case StackDepth:
				if (c == '\n' || c == '~')
				{
					ParserException("stack depth may not be the last thing in a line (excluding comments)");
					return;
				}
				else if (isspace(c))                  // found the end of the string of colons
				{
					mode = VariableNameStart;
					variable_name = "";
					return;
				}
				else if (isalpha(c) || c == '_')
				{
					mode = VariableName;
					variable_name = "" + c;
					return;
				}
				else if (c == ':')                              // increase stack depth once more
				{
					stack_depth++;
					return;
				}
				else
				{
					ParserException("invalid character after stack depth; expected whitespace or a variable name (starting with a letter or an underscore)");
					return;
				}

			case VariableNameStart:
				if (c == '\n' || c == '~')
					if (variable_name.length() == 0)
					{
						ParserException("stack depth may not be the last thing in a line (excluding comments)");
						return;
					}
					else
					{
						DoOperationUsingOperands(variable_name, NULL);
						mode = Comment;
						return;
					}
				else if (isalnum(c) || (variable_name.length() > 0 && c == '_'))
				{
					variable_name += c;
					mode = VariableName;
					return;
				}
				else if (isspace(c))
					return;
				else
				{
					ParserException("invalid character in variable name");
					return;
				}

			case VariableName:
				if (c == '\n' || c == '~')
				{
					DoOperationUsingOperands(variable_name, NULL);
					mode = Comment;
					return;
				}
				else if (isalnum(c) || (variable_name.length() > 0 && c == '_'))
				{
					variable_name += c;
					mode = VariableName;
					return;
				}
				else if (isspace(c))
				{
					mode = PostVariable;
					return;
				}
				else
				{
					ParserException("invalid character in variable name");
					return;
				}

			case PostVariable:
				if (c == '\n' || c == '~')
				{
					DoOperationUsingOperands(variable_name);
					mode = Comment;
					return;
				}
				else if (isspace(c))                  // more whitespace following the variable
					return;
				else if (isalnum(c) || c == '_' || c == '\"' || c == '-')
				{
					in_string_literal = (c == '\"');
					mode = ArgumentValue;
					argument_value = string() + c;
					return;
				}
				else
				{
					ParserException("invalid character following variable name");
					return;
				}

			case ArgumentValue:
				if (in_string_literal)
				{
					if (c == '\n')
					{
						ParserException("unclosed string literal");
						return;
					}
					else if (c == '\"')                                             // closing quotes are okay
					{
						in_string_literal = false;
						mode = PostArgument;
					}
					argument_value += c;
					return;
				}
				else if (c == '\n' || c == '~')
				{
					DoOperationUsingOperands(variable_name, argument_value);
					mode = Comment;
					return;
				}
				else if (isspace(c))
				{
					mode = PostArgument;
					return;
				}
				else if (isalnum(c) || c == '_' || c == '.' || c == '-')         // letters, numbers, - signs (for negative numbers), or dots (decimal point) are all valid here
				{
					argument_value += c;
					return;
				}
				else
				{
					ParserException("invalid/unexpected character in argument value");
					return;
				}

			case PostArgument:
				if (c == '\n' || c == '~')
				{
					DoOperationUsingOperands(variable_name, argument_value);
					mode = Comment;
					return;
				}
				else if (isspace(c))
					return;

				ParserException("invalid character following argument value");
				return;

		}
	}

	void TableParserInstance::ParserException(string message)
	{
		// could throw an exception, but i don't feel like it...
		Debug(message + "\n");

		mode = Error;
	}

	void TableParserInstance::End() { }




	/*
	 * NamedItemDictionaryTableParser methods
	 */
	NamedItemDictionaryTableParser::NamedItemDictionaryTableParser(istream* stream) : TableParserInstance(stream) { }

	void NamedItemDictionaryTableParser::InnerDispose() { TableParserInstance::InnerDispose(); }

	TableParseable* NamedItemDictionaryTableParser::DoOperation(string a, string b)
	{
		string lowercase_a = "";
		for(unsigned int i = 0; i < a.length(); i++)
			lowercase_a += tolower(a[i]);
		map<string, FieldSetter*>::iterator found = field_setters.find(lowercase_a);
		if(found != field_setters.end())
		{
			FieldSetter& setter = *found->second;
			return setter.Set(b);
		}
		else
			return OnFieldNotFound(a, b);
	}

	TableParseable* NamedItemDictionaryTableParser::OnFieldNotFound(string a, string b)
	{
		ParserException("field name not recognized");

		return this;
	}




	/*
	 * IntSetter methods
	 */
	IntSetter::IntSetter(int* i) : i(i) { }
	TableParseable* IntSetter::Set(string val) { *i = atoi(val.c_str()); return NULL; }




	/*
	 * FloatSetter methods
	 */
	FloatSetter::FloatSetter(float* f) : f(f) { }
	TableParseable* FloatSetter::Set(string val) { *f = (float)atof(val.c_str()); return NULL; }




	/*
	 * QuaternionSetter stuff
	 */
	struct QuaternionField : public NamedItemDictionaryTableParser
	{
		Quaternion* quat;
		FloatSetter w;
		FloatSetter x;
		FloatSetter y;
		FloatSetter z;

		QuaternionField(Quaternion* quat, istream* stream) :
			NamedItemDictionaryTableParser(stream),
			quat(quat),
			w(&quat->w),
			x(&quat->x),
			y(&quat->y),
			z(&quat->z)
		{
			field_setters["w"] = &w;
			field_setters["x"] = &x;
			field_setters["y"] = &y;
			field_setters["z"] = &z;
		}
	};

	QuaternionSetter::QuaternionSetter(Quaternion* quat, istream* stream) : quat(quat), stream(stream) { }
	TableParseable* QuaternionSetter::Set(string val) { return new QuaternionField(quat, stream); }




	/*
	 *  Vec3Setter stuff
	 */
	struct Vec3Field : public NamedItemDictionaryTableParser
	{
		Vec3* vec;
		FloatSetter x;
		FloatSetter y;
		FloatSetter z;

		Vec3Field(Vec3* vec, istream* stream) :
			NamedItemDictionaryTableParser(stream),
			vec(vec),
			x(&vec->x),
			y(&vec->y),
			z(&vec->z)
		{
			field_setters["x"] = &x;
			field_setters["y"] = &y;
			field_setters["z"] = &z;
		}
	};

	Vec3Setter::Vec3Setter(Vec3* vec, istream* stream) : vec(vec), stream(stream) { }
	TableParseable* Vec3Setter::Set(string val) { return new Vec3Field(vec, stream); }
}
