// ----------------------------------------------------------------------------
// OpenSDSE - HLA Compliant Distributed Aircraft Simulation
// Copyright (C) 2017  ISAE
//
// This program is free software ; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation ; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY ; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program ; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//
// ----------------------------------------------------------------------------


// System standard includes 
#include <iostream>
#include <sstream>
#include <cstdlib>

// Federate Specific includes 
#include "CockpitFederateHla1516e.hh"
#include "CockpitFederateThreadHla1516e.hh"
#include "CockpitMainWindow.hh"

//  Link with Qt Window
CockpitMainWindow *CockpitFederateThreadHla1516e::window;
CockpitFederateThreadHla1516e::CockpitFederateThreadHla1516e(CockpitMainWindow *_window)
{    
  CockpitFederateThreadHla1516e::window =_window;
}

void CockpitFederateThreadHla1516e::run()
{
	// Federation, Federate and FOM file name 
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
	string federationName =  getFederationName(params);
	string federateName   =  getFederateName(params, "COCKPIT");
	string fomFile        =  getFedFile(params);
	delete params;
	
	// TO DO: It has to be included in a DEBUG_XXX compilation flag
	std::wcout << L"***************************************************" << std::endl;
    std::wcout << L"" << std::endl;
	std::wcout << L"OpenSDSE Federate " << CockpitFederateHla1516e::getWString(federateName.c_str()) << " is started" << std::endl;
	#ifdef COLMPILATION_INFO_HH_DEF 
	std::wcout << "" << std::endl;
	std::wcout << L"Compilation Date: " << OPEN_SDSE_COMPILATION_DATE << std::endl;
    std::wcout << L"GCC version used: " << OPEN_SDSE_COMPILATION_GCC_VERSION << std::endl;
    std::wcout << L"Compilation OS and Kernel used: " << OPEN_SDSE_COMPILATION_OS_AND_KERNEL << std::endl;
	#endif
	#ifdef GIT_VERSION_INFO_HH_DEF 
	std::wcout << L"" << std::endl;
	std::wcout << L"Git Revision ID: " << OPEN_SDSE_GIT_VERSION_ID << std::endl;
    std::wcout << L"Git Revision Date: " << OPEN_SDSE_GIT_VERSION_DATE << std::endl;
	#endif
	std::wcout << L"" << std::endl;
    std::wcout << L"***************************************************" << std::endl;
	
	
	std::wstring FederationName = CockpitFederateHla1516e::getWString(federationName.c_str());
	std::wstring FederateName = CockpitFederateHla1516e::getWString(federateName.c_str());
	std::wstring FomFile = CockpitFederateHla1516e::getWString(fomFile.c_str());

	// Create a federate object.
	// This object inherit from appropriate FederateAmbassador class
	// and embbed the appropriate RTIambassador object.
	CockpitFederateHla1516e myFederate(FederationName, FederateName, FomFile);

	// Create, Join, Publish, Subscribe and Register
	myFederate.createFederationExecution();
	myFederate.joinFederationExecution();
	myFederate.getAllHandles();
	myFederate.publishAndSubscribe();
	myFederate.registerObjectInstances();

	// HLA tm settings
	myFederate.setHlaTimeManagementSettings( 20.0 // Timestep
                                           , 3.0 // Lookahead
                                           , 0.0 // LocalTime
                                           , 1500000 // SimulationEndTime
                                           );
	myFederate.enableTimeRegulation();
	myFederate.enableTimeConstrained();
	myFederate.enableAsynchronousDelivery();

	// Initial Sync protocol
	myFederate.pauseInitTrim();
	myFederate.initialization();
	myFederate.pauseInitSim();

	// Execution / Simulation Loop
	while (myFederate.getEndOfSimulation())
	{
		Q_EMIT gps_update_signal();
		Q_EMIT pfd_update_signal();
		Q_EMIT ecam_update_signal();
		myFederate.runOneStep();
	}

	// Unpublish, unsubscribe, Resign and Destroy
	myFederate.unpublishAndUnsubscribe();
	myFederate.resignFederationExecution();
	myFederate.destroyFederationExecution();

}
