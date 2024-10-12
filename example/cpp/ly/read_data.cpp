#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;

int main() 
{
    string str;
    //解析txt文本
    auto split_func = [](const string &s, char delimiter)
        -> vector<string>
    {
        vector<string> tokens;
        string token;
        istringstream tokenStream(s);
        while (getline(tokenStream, token, delimiter))
        {
            tokens.push_back(token);
        }
        return tokens;
    };
    //
    std::ifstream file_po("/home/robot/robot/roake_param_identify/build/drag/drag_data/position.txt");
    vector<vector<double>> position(12973, vector<double>(7));
    int rows = 0; // mat行号
    int cols = 0; // mat列号
    string line;
    while (getline(file_po, line))
    {
        vector<string> tokens = split_func(line, ' ');
        cols = 0;
        for (const auto &t : tokens)
        {
            position[rows][cols] = stod(t);
            cols++;
        }
        rows++;
    }
    std::ifstream file_ve("/home/robot/robot/roake_param_identify/build/drag/drag_data/velocity.txt");
    vector<vector<double>> velocity(12973, vector<double>(7));
    rows = 0;
    cols = 0;
    while (getline(file_ve, line))
    {
        vector<string> tokens = split_func(line, ' ');
        cols = 0;
        for (const auto &t : tokens)
        {
            velocity[rows][cols] = stod(t);
            cols++;
        }
        rows++;
    }
    return 0;
}