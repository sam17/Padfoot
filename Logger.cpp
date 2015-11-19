#include "stdafx.h"
#include "Logger.h"


Logger::Logger(string str)
{
	output.open(str);
}

template <typename T> void Logger::print(T t)
{
	output<<t<<" ";
}
void Logger::printString(string text)
{
	output<<text<<" ";
}

Logger::~Logger(void)
{
	output.close();
}
