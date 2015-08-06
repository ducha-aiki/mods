// Read an INI file into easy-to-access name/value pairs.

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include "../ini.h"
#include "INIReader.h"

#include <sstream>

#include <iostream>
using std::string;

std::string IntToStr( int n )
{
  std::ostringstream result;
  result << n;
  return result.str();
}


INIReader::INIReader(string filename)
{
  _error = ini_parse(filename.c_str(), ValueHandler, this);
}

int INIReader::ParseError()
{
  return _error;
}

string INIReader::Get(string section, string name, string default_value)
{
  string key = MakeKey(section, name);
  return _values.count(key) ? _values[key] : default_value;
}
string INIReader::GetString(string section, string name, string default_value)
{
  string valstr = Get(section, name, "");
  unsigned prev_pos = 0;

  int semicolon = valstr.find(';');
  if (semicolon != (int)string::npos)
    valstr = valstr.substr(0,semicolon);
  return valstr;
}
long INIReader::GetInteger(string section, string name, long default_value)
{
  string valstr = Get(section, name, "");
  int semicolon = valstr.find(';');
  if (semicolon != (int)string::npos)
    valstr = valstr.substr(0,semicolon);
  const char* value = valstr.c_str();
  char* end;
  // This parses "1234" (decimal) and also "0x4D2" (hex)
  long n = strtol(value, &end, 0);
  return end > value ? n : default_value;
}
double INIReader::GetDouble(string section, string name, double default_value)
{
  string valstr = Get(section, name, "");
  int semicolon = valstr.find(';');
  if (semicolon != (int)string::npos)
    valstr = valstr.substr(0,semicolon);
  const char* value = valstr.c_str();
  char* end;

  double n = strtod(value, &end);
  return end > value ? n : default_value;
}
bool INIReader::GetBoolean(string section, string name, bool default_value)
{
  string valstr = Get(section, name, "");
  // Convert to lower case to make string comparisons case-insensitive
  std::transform(valstr.begin(), valstr.end(), valstr.begin(), ::tolower);
  int semicolon = valstr.find(';');
  if (semicolon != (int)string::npos)
    valstr = valstr.substr(0,semicolon);
  if (valstr == "true" || valstr == "yes" || valstr == "on" || valstr == "1")
    return true;
  else if (valstr == "false" || valstr == "no" || valstr == "off" || valstr == "0")
    return false;
  else
    return default_value;
}
void INIReader::GetDoubleVector(string section, string name, std::vector<double> &vect)
{
  std::vector<double> DoubleVector;
  string valstr = Get(section, name, "");
  unsigned prev_pos = 0;

  int semicolon = valstr.find(';');
  if (semicolon != (int)string::npos)
    valstr = valstr.substr(0,semicolon);

  int found = valstr.find(',');
  if (found == (int)string::npos) //just one member in set
    {
      const char* value = valstr.c_str();
      char* end;
      double n = strtod(value, &end);
      DoubleVector.push_back(n);
    }
  else
    {
      while (found!=(int)string::npos)
        {
          string curr_str = valstr.substr(prev_pos,(found-prev_pos));
          const char* value = curr_str.c_str();
          char* end;
          double n = strtod(value, &end);
          DoubleVector.push_back(n);
          prev_pos = found+1;
          found = valstr.find(',',prev_pos);
        }
      string curr_str = valstr.substr(prev_pos,(found-prev_pos));
      const char* value = curr_str.c_str();
      char* end;
      double n = strtod(value, &end);
      DoubleVector.push_back(n);
    }
  vect = DoubleVector;
}
void INIReader::GetStringVector(std::string section, std::string name, std::vector<std::string> &vect)
{
  std::vector<std::string> StrVector;
  string valstr = Get(section, name, "");
  unsigned prev_pos = 0;

  int semicolon = valstr.find(';');
  if (semicolon != (int)string::npos)
    valstr = valstr.substr(0,semicolon);

  int found = valstr.find(',');
  if (found == (int)string::npos) //just one member in set
    StrVector.push_back(valstr);
  else
    {
      while (found!=(int)string::npos)
        {
          string curr_str = valstr.substr(prev_pos,(found-prev_pos));
          StrVector.push_back(curr_str);
          prev_pos = found+1;
          found = valstr.find(',',prev_pos);
        }
      string curr_str = valstr.substr(prev_pos,(found-prev_pos));
      StrVector.push_back(curr_str);
    }
  vect = StrVector;
}
string INIReader::MakeKey(string section, string name)
{
  string key = section + "." + name;
  // Convert to lower case to make section/name lookups case-insensitive
  std::transform(key.begin(), key.end(), key.begin(), ::tolower);
  return key;
}

int INIReader::ValueHandler(void* user, const char* section, const char* name,
                            const char* value)
{
  INIReader* reader = (INIReader*)user;
  string key = MakeKey(section, name);
  if (reader->_values[key].size() > 0)
    reader->_values[key] += "\n";
  reader->_values[key] += value;
  return 1;
}
