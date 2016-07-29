#include "custom_command.h"

string formation_config_tuning(string file_name)
{
    string parameters, line;  
    ifstream file(file_name.c_str(), ios::in);

    if(file)
	{		
		while(getline(file, line))
		    parameters+=line+" ";
		file.close();
		return parameters;
	}
	else
	{
		cout << "Error : cannot open file for formation_config" << endl;
		return "";
	}
}
