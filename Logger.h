#pragma once

#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <string>

using namespace std;

class Logger
{
	ofstream output;
	

public:
	
	void printString(string str);
	template <typename T> void print(T t);
	Logger(string str);

	~Logger(void);
};

