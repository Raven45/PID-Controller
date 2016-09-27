	#include "File.h"

	CSV_File::CSV_File () {
		this->FilePath = "default.csv";
		this->FileIsOpen = false;
	}

	CSV_File::~CSV_File () {

		if (FileIsOpen) {
			CloseFile ();
		}
	}

	void CSV_File::CreateFile (std::string FilePath) {

		//Set file path.
		this->FilePath = FilePath;

		//Append .csv extension if not present.
		if (FilePath.substr (FilePath.size () - 4) != ".csv") {
			this->FilePath.append (".csv");
		}

		//Clear the file stream buffer.
		FileStream.clear ();

		//Force the OS to create the file in the file system.
		FileStream.open (FilePath.c_str (), std::ios::out);

		//Close the file.
		FileStream.close ();

		//Reopen the file with the needed options.
		FileStream.open (FilePath.c_str (), std::ios::in | std::ios::out);

		//Mark the internal flag that we have an open file.
		this->FileIsOpen = true;
	}

	void CSV_File::OpenFile (std::string FilePath) {

		if (FilePath.substr (FilePath.size () - 4) == ".csv") {
		
			this->FilePath = FilePath;

			//Clear the file stream buffer.
			FileStream.clear ();

			//Attempt to open the file
			FileStream.open (FilePath.c_str (), std::ios::in);

			if (FileStream) {

				//Close the file.
				FileStream.close ();

				//Re-open file with proper flags
				FileStream.open (FilePath.c_str (), std::ios::in | std::ios::out | std::ios::app);
			}

			//File doesn't exist
			else {
				this->FileIsOpen = false;
			}
		}
	}

	void CSV_File::CloseFile () {

		this->FileIsOpen = false;

		FileStream.flush ();
		FileStream.close ();
	}

	void CSV_File::WriteLine (std::vector<std::string> Cells) {

		std::string Line = "";

		Line.append (Cells[0]);
		for (int i = 1; i < Cells.size (); i++) {
			Line.append (",");
			Line.append (Cells[i]);
		}

		//Write our line.
		FileStream.write (Line.c_str (), Line.size ());
		FileStream.write ("\n", 1);
	}






