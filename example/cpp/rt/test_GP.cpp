#include <iostream>
#include <cmath>

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

vector<vector<double>> tau_collect(10001, vector<double>(7));
int rows = 0; // mat行号
int cols = 0; // mat列号
string line;
while (getline(file, line))
{
    vector<string> tokens = split_func(line, ',');
    cols = 0;
    for (const auto &t : tokens)
    {
        tau_collect[rows][cols] = stod(t);
        cols++;
    }
    rows++;
}
