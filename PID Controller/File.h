#pragma once
#include <fstream>
#include <string>
#include <vector>

class CSV_File {

public:
	CSV_File ();
	~CSV_File ();

	void CreateFile (std::string FilePath);
	void OpenFile (std::string FilePath);
	void CloseFile ();

	void WriteLine (std::vector<std::string> Cells);

private:
	std::fstream FileStream;
	std::string FilePath;
	bool FileIsOpen;

	std::string * Columns;
	std::string * Rows;
};