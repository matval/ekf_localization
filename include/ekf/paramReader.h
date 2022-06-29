/*
Created by Karan on 6/6/18.
Modified by Mateus
*/

#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <map>
#include <vector>
#include <string>

using namespace std;

class ParameterReader
{
    public:
        ParameterReader(string filename = "parameters.txt")
        {
            bool verbose = false;

            ifstream fin( filename.c_str() );
            if (!fin)
            {
                cerr << "parameter file does not exist." << endl;
                return;
            }

            if(verbose)
                cout << "\n------ Reading in Parameter File: " << filename << endl;;

            while(!fin.eof())
            {                
                string str;
                getline( fin, str );
                
                if(verbose)
                    cout << "Line Read: " << str << endl;
                
                if (str[0] == '#')
                    continue;

                int pos = str.find("=");
                if (pos == -1)
                {
                    if(verbose)
                        cout << "pos found = -1 ---- Continuing loop...\r\n";
                    continue;
                }
                string key = str.substr( 0, pos );
                string value = str.substr( pos+1, str.length() );

                this->data[key] = value;

                if(verbose)
                    cout << "Key Found with Value: " << key << " -> " << value << endl;

                if ( !fin.good() )
                {
                    cout << "\r\n";
                    break;
                }
            }
        }

        string getData(string key, string default_value)
        {
            map<string, string>::iterator iter;
            iter = this->data.find(key.c_str());
            std::cout << "Searching for key (" << key.c_str() << ") => " << this->data[key] << '\n';
            if (iter == this->data.end())
            {
                cerr << "Parameter name " << key << " not found! Using: " << default_value << endl;
                return default_value; //string("NOT_FOUND");
            }
            return iter->second;
        }

        vector<double> getVector(string key, vector<double> default_value)
        {
            map<string, string>::iterator iter;
            iter = this->data.find(key.c_str());
            std::cout << "Searching for key (" << key.c_str() << ") => " << this->data[key] << '\n';
            if (iter == this->data.end())
            {
                cerr << "Parameter name " << key << " not found!" << endl;
                return default_value; //string("NOT_FOUND");
            }

            vector<double> value_vec;
            stringstream ss(iter->second);
            while(ss.good())
            {
                string substr;
                getline(ss, substr, ',');
                double value = atof(substr.c_str());
                value_vec.push_back(value);
            }
            return value_vec;
        }

    public:
        map<string, string> data;
};
