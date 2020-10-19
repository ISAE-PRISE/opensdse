----------------------------------------------------------------------
OpenSDSE
----------------------------------------------------------------------
SDSE is a french accronym which stands for "Embedded Systems 
Distributed Simulation". Open refers to the open source version.

This project is a distributed simulation for an aircraft compliant 
with the High Level Architecture (HLA). 

The simulation is compliant with the old standard HLA 1.3 and the 
latest one HLA IEEE 1516 Evolved.

It has been mainly tested with CERTI HLA/RTI Middleware:
https://savannah.nongnu.org/projects/certi

current version: 1.0

For any question, please contact:
- jean-baptiste.chaudron@isae.fr
- d.saussie@polymtl.ca

----------------------------------------------------------------------
INSTALLATION
----------------------------------------------------------------------

1) Install CERTI:
prerequise: Check if libxml2-dev, flex, bison package are installed
On Ubuntu: 
>> sudo apt install libxml2-dev flex bison

install process:
>> cd ${WHERE_YOU_WANNA_INSTALL}
>> git clone https://git.savannah.nongnu.org/git/certi.git
>> mkdir build
>> mkdir install
>> cd build
>> cmake -DCMAKE_INSTALL_PREFIX=../install ../certi/
>> make -j4 install

2) Install Open SDSE:
prerequise: Qt5 installed with Widgets WebKitWidgets Svg libraries
On Ubuntu
>> sudo apt install qtmultimedia5-dev libqt5webkit5-dev qtdeclarative5-dev qtbase5-dev libqt5svg5

Then this is a pretty common CMake project as well.

>> source ${WHERE_YOU_HAVE_INSTALLED_CERTI}/install/share/scripts/myCERTI_env.sh
>> git clone 
>> mkdir build
>> mkdir install
>> cd build
>> cmake -DCMAKE_INSTALL_PREFIX=../install ../opensdse
>> make -j4 install


----------------------------------------------------------------------
RUN IT
----------------------------------------------------------------------
Then you need to copy the file ./run/fom/sdse_hla1516.xml into the certi FOM folder.
For me:
>> cp ./install/fom/sdse_hla1516.xml  ${WHERE_YOU_HAVE_INSTALLED_CERTI}/install/share/federations/

Launch the simulator:
>> source ./install/scripts/Local_Run_bash_1516e_External_Efcs.sh

At this stage if all is good than you must see lot of gnome terminal open 
and a cockpit aircraft.

You must locate the terminal "FED 5: FLIGHT_DYNAMICS_FED"

On the "FED 5: FLIGHT_DYNAMICS_FED" it requires you to press a key for 
TRIMMING (i.e. Compute aircraft equilibrum point) then another time to launch the
SIMULATION at this equilibrum point. Then the aircraft is flight and you can see it on the PFD.

----------------------------------------------------------------------
DOCUMENTATION
----------------------------------------------------------------------
We prepared a user guide (which is a little outdated now).
Please refer to it for an overview. 
The document is OpenSDSE_user_guide.pdf (current version is a draft).

----------------------------------------------------------------------
OTHER OPEN-SOURCES PROJECTS USED
----------------------------------------------------------------------
OpenSDSE contains parts of others open source projects integrated in the source
code:
- JsBSim
- Flightgear
- QtFlightInstruments
- TinyXML2
- GnuPlot
We tried our best to properly reference the link to each open source project we
were/are using. However, we may have missed some references thus if you find any
inconsistency in the references, please contact: 
jean-baptiste.chaudron@isae.fr

