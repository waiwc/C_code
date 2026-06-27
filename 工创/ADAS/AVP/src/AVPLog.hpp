/*!
* Outputs AVP-concerned data to a file for further analysis
*
* @file AVPLog.hpp
* @author YejingWang
* @version 2.0 12/24/2020
*/

#pragma once

#include <fstream>
#include <iostream>

// change this to 1 to enable logging
#define AVP_LOG 0

class AVPLog
{
public:
	static AVPLog& getInstance()
	{
		static AVPLog obj;
		return obj;
	}

	~AVPLog()
	{
		mFile.close();
	}

	void addData(const double data, const bool bNewLine = false)
	{
		mFile << data << ",";

		if (bNewLine)
		{
			mFile << std::endl;
		}
	}

	void addNewLine()
	{
		mFile << std::endl;
	}

	void coutparkingspace(double id, double x, double y)
	{
		mFile << "id: " << id << " ,x : " << x << " ,y :" << y;
		addNewLine();
	}

private:
	AVPLog()
	{
		mFile.open("C:/Sim-One/SimOneAPI/ADAS/AVP/AVP.txt");
	}

	std::ofstream mFile;
};
