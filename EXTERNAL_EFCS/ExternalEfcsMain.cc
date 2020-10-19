#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <fstream>
#include <stdint.h>
#include <memory>
#include <time.h>

#include <ExternalEfcs.hh>

int main(int argc, char *argv[])
{
	if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " ./ExternalEfcs <ID> " << std::endl;
		return 1;
    }
	ExternalEfcs myExternalEfcs;
	int Id;
	
	Id = atoi(argv[1]);
	myExternalEfcs.setId(Id);
	
	std::cout << "System ExternalEfcs created Id: "  
	          << myExternalEfcs.getId()
	          << std::endl;
	          
    myExternalEfcs.constructSockets();
    myExternalEfcs.compute();

	return 1;
}
