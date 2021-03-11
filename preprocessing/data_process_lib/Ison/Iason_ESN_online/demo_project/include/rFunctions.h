

#include <sys/stat.h>
//#include <unistd.h>
#include <string>

inline bool fileExists(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}


int count_esn(std::string base){
	int esn_nets = 1;

	//std::ostringstream convert;
	//convert << base << esn_nets + 1 << ".txt";
	//std::string filename = convert.str();

	std::string filename = base + "1.txt";

	while (fileExists(filename)){

		esn_nets++;
		//convert.str("");
		//convert.clear();
		//convert << base << esn_nets + 1 << ".txt";
		//filename = convert.str();
		filename = base + std::to_string(esn_nets) + ".txt";


	}
	return esn_nets-1;
}


