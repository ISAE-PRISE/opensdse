# ----------------------------------------------------------------------------
# OpenSDSE - HLA Compliant Distributed Aircraft Simulation
# Copyright (C) 2017  ISAE
#
# This program is free software ; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation ; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY ; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program ; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
#
# ----------------------------------------------------------------------------

###################################################################################
# PROJECT NAME : #
PROJECT(OPEN-SDSE)

###################################################################################
# CERTI and SDSE requires CMake 2.8.11 version #
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)
SET(CMAKE_CXX_FLAGS "-std=c++0x")

###################################################################################
# WARNING POLICY #
CMAKE_POLICY(SET CMP0014 OLD)
cmake_policy(SET CMP0003 NEW)

###################################################################################
####################  Use CERTI specific CMake modules first  #####################
#####           You should set CERTI_HOME env var to appropriate value ############
######### or put here the full path to the directory containing FindRTI.cmake #####
#####################  Use CERTI_HOME env var if it is defined ####################
###################################################################################
IF ("$ENV{CERTI_HOME}" STRGREATER "")
    FILE(TO_CMAKE_PATH "$ENV{CERTI_HOME}" CERTI_HOME_TEMP)
    SET(CERTI_HOME ${CERTI_HOME_TEMP} CACHE PATH "The CERTI install directory")
    LIST(APPEND CMAKE_MODULE_PATH "${CERTI_HOME}/share/scripts")

# Or try to use CERTI possible installed at the same INSTALL_PREFIX
ELSE ("$ENV{CERTI_HOME}" STRGREATER "")
    LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_INSTALL_PREFIX}/share/scripts")    
ENDIF ("$ENV{CERTI_HOME}" STRGREATER "")
#####
#SET(CMAKE_MODULE_PATH $ENV{CERTI_HOME}/share/scripts)

################################################################################
################### For configuration of library for root    ###################
################################################################################
OPTION(USE_FULL_RPATH
             "Use the full RPATH" OFF)
             
IF (USE_FULL_RPATH)       
    # see http://www.cmake.org/Wiki/CMake_RPATH_handling#Always_full_RPATH
    # use, i.e. don't skip the full RPATH for the build tree
    SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
    
    # when building, don't use the install RPATH already
    # (but later on when installing)
    SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
    
    # the RPATH to be used when installing
    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    
    # add the automatically determined parts of the RPATH
    # which point to directories outside the build tree to the install RPATH
    SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
ENDIF(USE_FULL_RPATH)

################################################################################
################# You should uncomment the following line ######################
###################### if you want VERBOSE build ###############################
################################################################################
#SET(CMAKE_VERBOSE_MAKEFILE ON)

################################################################################
################### The preferred (default) behaviour is to ###################
#################### build library as shared on all platform ###################
##################################################################################
OPTION(BUILD_SHARED
  "Build libraries as shared library" ON)
IF (BUILD_SHARED)
  SET(BUILD_SHARED_LIBS ON)
ELSE (BUILD_SHARED)
  SET(BUILD_SHARED_LIBS OFF)
ENDIF (BUILD_SHARED)

##################################################################################
##################### Change the build type to release ###########################
####################### when you are ready to release ############################
##################################################################################
#SET(CMAKE_BUILD_TYPE "Debug")
###
SET(CMAKE_BUILD_TYPE "Release")

#########################################################################
############## Define your project packaging basics #####################
#########################################################################
SET(PACKAGE_NAME      ${CMAKE_PROJECT_NAME})
SET(DESCRIPTION_SUMMARY "SDSE - Distributed Simulation for Embedded Systems (French acronym) - HLA standard compliant Fligh Simulator.")
SET(PACKAGE_VERSION_MAJOR "1")
SET(PACKAGE_VERSION_MINOR "0")
SET(PACKAGE_VERSION_PATCH "0")
SET(PACKAGE_VERSION   "${PACKAGE_VERSION_MAJOR}.${PACKAGE_VERSION_MINOR}.${PACKAGE_VERSION_PATCH}")
###
SET(PACKAGE_BUGREPORT "jean-baptiste.chaudron@isae.fr")
SET(PACKAGE_BUGREPORT "david.saussie@polymtl.ca")

MESSAGE(STATUS "Configuring ${PACKAGE_NAME} version ${PACKAGE_VERSION}")

option(HLA_13
           "SDSE Federation will use HLA 1.3 standard Federation Object Model" ON)
           
option(HLA_1516
           "SDSE Federation will use HLA 1516-2000 standard Federation Object Model" OFF)
           
option(HLA_1516E
           "SDSE Federation will use HLA 1516-2010 Evolved standard Federation Object Model" OFF)
     
# To enable this option you have to set in to ON into the cmake command line
# cmake -DCMAKE_INSTALL_PREFIX=../install ../src/ -DJOYSTICK_FED_ONLY=ON           
option(JOYSTICK_FED_ONLY "Allow to only compile Joystick Federate to test with other federations" OFF)

#########################################################################
###################   Call i.e. FIND RTI   ##############################
#########################################################################
FIND_PACKAGE(RTI REQUIRED)
INCLUDE_DIRECTORIES(${RTI_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${RTI1516e_INCLUDE_DIR})
ADD_DEFINITIONS(${RTI_DEFINITIONS})
MESSAGE(STATUS "* RTI FOUND *")

#########################################################################
###################   Call i.e. FIND Qt5   ##############################
#########################################################################
IF (NOT JOYSTICK_FED_ONLY)
FIND_PACKAGE(Qt5 COMPONENTS Widgets WebKitWidgets Svg QUIET)
IF (Qt5_FOUND)
  MESSAGE(STATUS "* Qt${Qt5_VERSION} Found *" )
ELSE ()
  MESSAGE(STATUS "* Qt5 NOT FOUND: GUI NOT ACTIVATED *")
ENDIF (Qt5_FOUND)
ENDIF (NOT JOYSTICK_FED_ONLY)

#########################################################################
#################### CPack specific Configuration #######################
#########################################################################
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "SDSE HLA FEDERATION using CERTI")
###
#SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/HLA_Tutorial_DESCRIPTION.txt")
###
#SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
SET(CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
SET(CPACK_SYSTEM_NAME ${CMAKE_SYSTEM_NAME})
#########################################################################
# Avoid putting BUILDIR inside source package (simple case)
#########################################################################
IF ("${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
     SET(CPACK_SOURCE_IGNORE_FILES "/CVS/;/\\\\.svn/;\\\\.swp$;\\\\.#;/#;.*~;cscope.*;\\\\.cproject;\\\\.project;autom4te.cache/*")
ELSE ("${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
     SET(CPACK_SOURCE_IGNORE_FILES "/CVS/;/\\\\.svn/;\\\\.swp$;\\\\.#;/#;.*~;cscope.*;\\\\.cproject;\\\\.project;autom4te.cache/*;${CMAKE_BINARY_DIR}/*")
ENDIF ("${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")

SET(CPACK_SOURCE_GENERATOR "ZIP;TGZ")
IF (WIN32)
  SET(CPACK_GENERATOR "ZIP;NSIS")
  ## Add a custom target callable from IDE (Mostly for Visual Studio)
  GET_FILENAME_COMPONENT(CPACK_COMMAND ${CMAKE_COMMAND} PATH)
  SET(CPACK_COMMAND ${CPACK_COMMAND}/cpack)
  ADD_CUSTOM_TARGET(PACKAGE_SOURCE_ZIP
              COMMAND ${CPACK_COMMAND} -G ZIP --config CPackSourceConfig.cmake
              COMMENTS "Build a ZIP file containing the source")           
  # NSIS installer specific part
  INCLUDE(InstallRequiredSystemLibraries)
ELSE (WIN32)
  SET(CPACK_GENERATOR "ZIP;TGZ")
ENDIF(WIN32)
SET(CPACK_PACKAGE_VERSION ${PACKAGE_VERSION})
SET(CPACK_PACKAGE_VERSION_MAJOR ${PACKAGE_VERSION_MAJOR})
SET(CPACK_PACKAGE_VERSION_MINOR ${PACKAGE_VERSION_MINOR})
SET(CPACK_PACKAGE_VERSION_PATCH ${PACKAGE_VERSION_PATCH})

INCLUDE(CPack)

IF (NOT JOYSTICK_FED_ONLY)

# PATHS CONFIGURATION FOR THE WHOLE PROJECT ##################
# FOR 01_JOYSTICK FEDERATE #
SET(1_JOYSTICK_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/01_JOYSTICK)
ADD_SUBDIRECTORY(${1_JOYSTICK_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${1_JOYSTICK_FED_SOURCE_DIRECTORY}
					${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
					)

# FOR 02_EFCS FEDERATE #
SET(2_EFCS_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/02_EFCS)
ADD_SUBDIRECTORY(${2_EFCS_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${2_EFCS_FED_SOURCE_DIRECTORY}
					${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
					)
					
# FOR EXTERNAL EFCS #
SET(EXTERNAL_EFCS_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/EXTERNAL_EFCS)
ADD_SUBDIRECTORY(${EXTERNAL_EFCS_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${EXTERNAL_EFCS_SOURCE_DIRECTORY}
					${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
					)

# FOR 03_CONTROL_SURFACES FEDERATE #
SET(3_CONTROL_SURFACES_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/03_CONTROL_SURFACES)
ADD_SUBDIRECTORY(${3_CONTROL_SURFACES_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${3_CONTROL_SURFACES_FED_SOURCE_DIRECTORY}
					${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
					)

# FOR 04_ENGINES FEDERATE #
SET(4_ENGINES_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/04_ENGINES)
ADD_SUBDIRECTORY(${4_ENGINES_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${4_ENGINES_FED_SOURCE_DIRECTORY}
					${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
					)

# FOR 05_FLIGHT_DYNAMICS FEDERATE #
SET(5_FLIGHT_DYNAMICS_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/05_FLIGHT_DYNAMICS)
ADD_SUBDIRECTORY(${5_FLIGHT_DYNAMICS_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${5_FLIGHT_DYNAMICS_FED_SOURCE_DIRECTORY}
					${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
					)

# FOR 06_SENSORS FEDERATE #
SET(6_SENSORS_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/06_SENSORS)
ADD_SUBDIRECTORY(${6_SENSORS_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${6_SENSORS_FED_SOURCE_DIRECTORY}
					${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
					)

# FOR 07_VISUALIZATION FEDERATE #
SET(7_VISUALIZATION_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/07_VISUALIZATION)
ADD_SUBDIRECTORY(${7_VISUALIZATION_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${7_VISUALIZATION_FED_SOURCE_DIRECTORY}
					${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
					)

# FOR 08_ENVIRONMENT FEDERATE #
SET(8_ENVIRONMENT_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/08_ENVIRONMENT)
ADD_SUBDIRECTORY(${8_ENVIRONMENT_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${8_ENVIRONMENT_FED_SOURCE_DIRECTORY}
                    ${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
                    )

# FOR 09_COCKPIT FEDERATE #
SET(09_COCKPIT_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/09_COCKPIT)
ADD_SUBDIRECTORY(${09_COCKPIT_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${09_COCKPIT_FED_SOURCE_DIRECTORY}
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND
                    ${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
                    )
					
# FOR 10_DATA_LOGGER FEDERATE #
SET(10_DATA_LOGGER_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/10_DATA_LOGGER)
SET(10_DATA_LOGGER_FED_GNUPLOT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/10_DATA_LOGGER/Gnuplot)
ADD_SUBDIRECTORY(${10_DATA_LOGGER_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${10_DATA_LOGGER_FED_SOURCE_DIRECTORY}
                    ${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
                    )
                    
# FOR COMMON SOURCES FILES #
SET(COMMON_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/COMMON)
ADD_SUBDIRECTORY(${COMMON_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${COMMON_SOURCE_DIRECTORY})

# FOR SDSE_CONFIGURATOR TOOL #
SET(SDSE_CONFIGURATOR_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR)
ADD_SUBDIRECTORY(${SDSE_CONFIGURATOR_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${SDSE_CONFIGURATOR_SOURCE_DIRECTORY})

# INIT CONFIGURATION PATH #
SET(INIT_FILES_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/INIT_FILES)

#################################################################################
#  SETUP 1_JOYSTICK FEDERATE (FED 1) #
SET(EXE_NAME JoystickFederateHla13)
SET(SRC_LIST ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/JoystickFederateMainHla13.cc
             ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/JoystickFederateHla13.cc
             ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/Joystick.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

# ONLY COMPILE FULLY ON LINUX (because of linux/joystick API)
# Compilation on Mac gives a federate which publish 0 for all commands
  ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
  TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES} ${LIBRT})
  INSTALL(TARGETS ${EXE_NAME} 
          RUNTIME DESTINATION bin)
          
SET(EXE_NAME JoystickFederateHla1516e)
SET(SRC_LIST ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/JoystickFederateMainHla1516e.cc
             ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/JoystickFederateHla1516e.cc
             ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/Joystick.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

# ONLY COMPILE FULLY ON LINUX (because of linux/joystick API)
# Compilation on Mac gives a federate which publish 0 for all commands
  ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
  TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES} ${LIBRT})
  INSTALL(TARGETS ${EXE_NAME} 
          RUNTIME DESTINATION bin)

#################################################################################
# SETUP 2_EFCS FEDERATE (FED 2) #      
SET(EXE_NAME EfcsFederateHla13)
SET(SRC_LIST ${2_EFCS_FED_SOURCE_DIRECTORY}/EfcsFederateMainHla13.cc
             ${2_EFCS_FED_SOURCE_DIRECTORY}/EfcsFederateHla13.cc
             ${2_EFCS_FED_SOURCE_DIRECTORY}/Efcs.cc
             ${2_EFCS_FED_SOURCE_DIRECTORY}/ControllerPID.cc
             ${COMMON_SOURCE_DIRECTORY}/A320.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)
        
SET(EXE_NAME EfcsFederateHla1516e)
SET(SRC_LIST ${2_EFCS_FED_SOURCE_DIRECTORY}/EfcsFederateMainHla1516e.cc
             ${2_EFCS_FED_SOURCE_DIRECTORY}/EfcsFederateHla1516e.cc
             ${2_EFCS_FED_SOURCE_DIRECTORY}/Efcs.cc
             ${2_EFCS_FED_SOURCE_DIRECTORY}/ControllerPID.cc
             ${COMMON_SOURCE_DIRECTORY}/SendingSocketUDP.cc
             ${COMMON_SOURCE_DIRECTORY}/ReceivingSocketUDP.cc
             ${COMMON_SOURCE_DIRECTORY}/A320.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)
        
SET(EXE_NAME ExternalEfcs)
SET(SRC_LIST ${EXTERNAL_EFCS_SOURCE_DIRECTORY}/ExternalEfcsMain.cc
             ${EXTERNAL_EFCS_SOURCE_DIRECTORY}/ExternalEfcs.cc
             ${2_EFCS_FED_SOURCE_DIRECTORY}/Efcs.cc
             ${2_EFCS_FED_SOURCE_DIRECTORY}/ControllerPID.cc
             ${COMMON_SOURCE_DIRECTORY}/SendingSocketUDP.cc
             ${COMMON_SOURCE_DIRECTORY}/ReceivingSocketUDP.cc
             ${COMMON_SOURCE_DIRECTORY}/A320.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)

#################################################################################       
# SETUP 3_CONTROL_SURFACES FEDERATE (FED 3) #
SET(EXE_NAME ControlSurfacesFederateHla13)
SET(SRC_LIST ${3_CONTROL_SURFACES_FED_SOURCE_DIRECTORY}/ControlSurfacesFederateMainHla13.cc 
             ${3_CONTROL_SURFACES_FED_SOURCE_DIRECTORY}/ControlSurfacesFederateHla13.cc
             ${3_CONTROL_SURFACES_FED_SOURCE_DIRECTORY}/ControlSurface.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${COMMON_SOURCE_DIRECTORY}/A320.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME}
        RUNTIME DESTINATION bin)
        
SET(EXE_NAME ControlSurfacesFederateHla1516e)
SET(SRC_LIST ${3_CONTROL_SURFACES_FED_SOURCE_DIRECTORY}/ControlSurfacesFederateMainHla1516e.cc 
             ${3_CONTROL_SURFACES_FED_SOURCE_DIRECTORY}/ControlSurfacesFederateHla1516e.cc
             ${3_CONTROL_SURFACES_FED_SOURCE_DIRECTORY}/ControlSurface.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${COMMON_SOURCE_DIRECTORY}/A320.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME}
        RUNTIME DESTINATION bin)

#################################################################################
# SETUP 4_ACTUATEUR_MOTEUR FEDERATE  (FED 4) #
SET(EXE_NAME EnginesFederateHla13)
SET(SRC_LIST ${4_ENGINES_FED_SOURCE_DIRECTORY}/EnginesFederateMainHla13.cc 
             ${4_ENGINES_FED_SOURCE_DIRECTORY}/EnginesFederateHla13.cc
             ${4_ENGINES_FED_SOURCE_DIRECTORY}/Engine.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)
        
SET(EXE_NAME EnginesFederateHla1516e)
SET(SRC_LIST ${4_ENGINES_FED_SOURCE_DIRECTORY}/EnginesFederateMainHla1516e.cc 
             ${4_ENGINES_FED_SOURCE_DIRECTORY}/EnginesFederateHla1516e.cc
             ${4_ENGINES_FED_SOURCE_DIRECTORY}/Engine.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)

#################################################################################
# SETUP 5_FLIGHT_DYNAMICS FEDERATE (FED 5) #
SET(EXE_NAME FlightDynamicsFederateHla13)
SET(SRC_LIST ${5_FLIGHT_DYNAMICS_FED_SOURCE_DIRECTORY}/FlightDynamicsFederateMainHla13.cc 
             ${5_FLIGHT_DYNAMICS_FED_SOURCE_DIRECTORY}/FlightDynamicsFederateHla13.cc
             ${5_FLIGHT_DYNAMICS_FED_SOURCE_DIRECTORY}/FlightDynamics.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${COMMON_SOURCE_DIRECTORY}/FGMatrix33.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${COMMON_SOURCE_DIRECTORY}/A320.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc           
             ${COMMON_SOURCE_DIRECTORY}/EarthModel.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)
        
SET(EXE_NAME FlightDynamicsFederateHla1516e)
SET(SRC_LIST ${5_FLIGHT_DYNAMICS_FED_SOURCE_DIRECTORY}/FlightDynamicsFederateMainHla1516e.cc 
             ${5_FLIGHT_DYNAMICS_FED_SOURCE_DIRECTORY}/FlightDynamicsFederateHla1516e.cc
             ${5_FLIGHT_DYNAMICS_FED_SOURCE_DIRECTORY}/FlightDynamics.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${COMMON_SOURCE_DIRECTORY}/FGMatrix33.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${COMMON_SOURCE_DIRECTORY}/A320.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc           
             ${COMMON_SOURCE_DIRECTORY}/EarthModel.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)
#################################################################################
# SETUP 6_SENSORS FEDERATE (FED 6) # 
SET(EXE_NAME SensorsFederateHla13)
SET(SRC_LIST ${6_SENSORS_FED_SOURCE_DIRECTORY}/SensorsFederateMainHla13.cc 
             ${6_SENSORS_FED_SOURCE_DIRECTORY}/SensorsFederateHla13.cc
             ${6_SENSORS_FED_SOURCE_DIRECTORY}/Sensor.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc              
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES} ${LIBRT})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)
        
        
SET(EXE_NAME SensorsFederateHla1516e)
SET(SRC_LIST ${6_SENSORS_FED_SOURCE_DIRECTORY}/SensorsFederateMainHla1516e.cc 
             ${6_SENSORS_FED_SOURCE_DIRECTORY}/SensorsFederateHla1516e.cc
             ${6_SENSORS_FED_SOURCE_DIRECTORY}/Sensor.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc              
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES} ${LIBRT})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)



#################################################################################
# SETUP 7_VISUALIZATION FEDERATE (FED 7) #
SET(EXE_NAME VisualizationFederateHla13)      
SET(SRC_LIST ${7_VISUALIZATION_FED_SOURCE_DIRECTORY}/VisualizationFederateMainHla13.cc 
	 ${7_VISUALIZATION_FED_SOURCE_DIRECTORY}/VisualizationFederateHla13.cc
	 ${7_VISUALIZATION_FED_SOURCE_DIRECTORY}/Visualization.cc
	 ${COMMON_SOURCE_DIRECTORY}/A320.cc
	 ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
	 ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
	 ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
	 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
	 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
	 )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)
        
SET(EXE_NAME VisualizationFederateHla1516e)      
SET(SRC_LIST ${7_VISUALIZATION_FED_SOURCE_DIRECTORY}/VisualizationFederateMainHla1516e.cc 
	 ${7_VISUALIZATION_FED_SOURCE_DIRECTORY}/VisualizationFederateHla1516e.cc
	 ${7_VISUALIZATION_FED_SOURCE_DIRECTORY}/Visualization.cc
	 ${COMMON_SOURCE_DIRECTORY}/A320.cc
	 ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
	 ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
	 ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
	 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
	 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
	 )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)


#################################################################################
# SETUP 08_ENVIRONMENT FEDERATE (FED 8) #
SET(EXE_NAME EnvironmentFederateHla13)
SET(SRC_LIST ${8_ENVIRONMENT_FED_SOURCE_DIRECTORY}/EnvironmentFederateMainHla13.cc 
             ${8_ENVIRONMENT_FED_SOURCE_DIRECTORY}/EnvironmentFederateHla13.cc
             ${8_ENVIRONMENT_FED_SOURCE_DIRECTORY}/Environment.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${COMMON_SOURCE_DIRECTORY}/FGMatrix33.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)
        
SET(EXE_NAME EnvironmentFederateHla1516e)
SET(SRC_LIST ${8_ENVIRONMENT_FED_SOURCE_DIRECTORY}/EnvironmentFederateMainHla1516e.cc 
             ${8_ENVIRONMENT_FED_SOURCE_DIRECTORY}/EnvironmentFederateHla1516e.cc
             ${8_ENVIRONMENT_FED_SOURCE_DIRECTORY}/Environment.cc
             ${COMMON_SOURCE_DIRECTORY}/UnitConversion.cc
             ${COMMON_SOURCE_DIRECTORY}/FGColumnVector3.cc
             ${COMMON_SOURCE_DIRECTORY}/FGMatrix33.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES})
INSTALL(TARGETS ${EXE_NAME} 
        RUNTIME DESTINATION bin)
####################################################################################
# SETUP 09_COCKPIT (FED 9) #
if (Qt5_FOUND)
	SET(EXE_NAME CockpitFederateHla13)
	SET(SRC_LIST ${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitFederateThreadHla13.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitFederateHla13.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/Cockpit.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitMainHla13.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitMainWindow.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/FCU/FCUwindow.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/ECAMwindow.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/N1display.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetPFD.cpp
				 ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetNAV.cpp
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi_PFD.cpp
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi_NAV.cpp
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/LayoutSquare.cpp
				 
	             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
	             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
				 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
				 )

	# Qt5 specific functions #
	Set(HDR_LIST  	${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitMainWindow.hh
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitFederateThreadHla13.hh
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/FCU/FCUwindow.hh
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/ECAMwindow.hh
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetPFD.h
				    ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetNAV.h
	                ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi_PFD.h
	                ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi_NAV.h
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/LayoutSquare.h)
	
	SET(FORMS_LIST 	${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitMainWindow.ui
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/FCU/FCUwindow.ui
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/ECAMwindow.ui
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetNAV.ui
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetPFD.ui)
	
	SET(RESSOURCES_LIST ${09_COCKPIT_FED_SOURCE_DIRECTORY}/Ressources.qrc
						${09_COCKPIT_FED_SOURCE_DIRECTORY}/FCU/fcu_rc.qrc
						${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/ecam_rc.qrc
						${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi.qrc)

	QT5_WRAP_CPP(09_COCKPIT_HEADERS_MOC ${HDR_LIST})
	QT5_WRAP_UI(09_COCKPIT_FORMS_HEADERS ${FORMS_LIST})
	QT5_ADD_RESOURCES(09_COCKPIT_RESSOURCES_RCC ${RESSOURCES_LIST})

	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

  ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST} ${09_COCKPIT_HEADERS_MOC}
                  ${09_COCKPIT_FORMS_HEADERS} ${09_COCKPIT_RESSOURCES_RCC})
  TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES} ${LIBRT} 
                          Qt5::Widgets Qt5::WebKitWidgets Qt5::Svg)
  INSTALL(TARGETS ${EXE_NAME} 
          RUNTIME DESTINATION bin)
          
          
	SET(EXE_NAME CockpitFederateHla1516e)
	SET(SRC_LIST ${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitFederateThreadHla1516e.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitFederateHla1516e.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/Cockpit.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitMainHla1516e.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitMainWindow.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/FCU/FCUwindow.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/ECAMwindow.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/N1display.cc
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetPFD.cpp
				 ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetNAV.cpp
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi_PFD.cpp
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi_NAV.cpp
	             ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/LayoutSquare.cpp
				 
	             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
	             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
				 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
				 )

	# Qt5 specific functions #
	Set(HDR_LIST  	${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitMainWindow.hh
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitFederateThreadHla1516e.hh
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/FCU/FCUwindow.hh
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/ECAMwindow.hh
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetPFD.h
				    ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetNAV.h
	                ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi_PFD.h
	                ${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi_NAV.h
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/LayoutSquare.h)
	
	SET(FORMS_LIST 	${09_COCKPIT_FED_SOURCE_DIRECTORY}/CockpitMainWindow.ui
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/FCU/FCUwindow.ui
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/ECAMwindow.ui
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetNAV.ui
					${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/WidgetPFD.ui)
	
	SET(RESSOURCES_LIST ${09_COCKPIT_FED_SOURCE_DIRECTORY}/Ressources.qrc
						${09_COCKPIT_FED_SOURCE_DIRECTORY}/FCU/fcu_rc.qrc
						${09_COCKPIT_FED_SOURCE_DIRECTORY}/ECAM/ecam_rc.qrc
						${09_COCKPIT_FED_SOURCE_DIRECTORY}/PFD_ND/qfi.qrc)

	QT5_WRAP_CPP(09_COCKPIT_HEADERS_MOC ${HDR_LIST})
	QT5_WRAP_UI(09_COCKPIT_FORMS_HEADERS ${FORMS_LIST})
	QT5_ADD_RESOURCES(09_COCKPIT_RESSOURCES_RCC ${RESSOURCES_LIST})

	INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

  ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST} ${09_COCKPIT_HEADERS_MOC}
                  ${09_COCKPIT_FORMS_HEADERS} ${09_COCKPIT_RESSOURCES_RCC})
  TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES} ${LIBRT} 
                          Qt5::Widgets Qt5::WebKitWidgets Qt5::Svg)
  INSTALL(TARGETS ${EXE_NAME} 
          RUNTIME DESTINATION bin)
endif (Qt5_FOUND)

if (NOT Qt5_FOUND)
	MESSAGE(STATUS "Qt5 not found! Please check your settings - Cockpit federate not configured for compilation")
endif (NOT Qt5_FOUND)

####################################################################################
# SETUP 10_DATA_LOGGER (FED 10) #	
SET(EXE_NAME DataLoggerFederateHla13)
SET(SRC_LIST ${10_DATA_LOGGER_FED_SOURCE_DIRECTORY}/DataLoggerFederateMainHla13.cc
			 ${10_DATA_LOGGER_FED_SOURCE_DIRECTORY}/DataLoggerFederateHla13.cc
			 ${10_DATA_LOGGER_FED_SOURCE_DIRECTORY}/DataLogger.cc
			 ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
			 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
			 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
			 )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES} ${LIBRT} )
INSTALL(TARGETS ${EXE_NAME} 
		RUNTIME DESTINATION bin)
		
SET(EXE_NAME DataLoggerFederateHla1516e)
SET(SRC_LIST ${10_DATA_LOGGER_FED_SOURCE_DIRECTORY}/DataLoggerFederateMainHla1516e.cc
			 ${10_DATA_LOGGER_FED_SOURCE_DIRECTORY}/DataLoggerFederateHla1516e.cc
			 ${10_DATA_LOGGER_FED_SOURCE_DIRECTORY}/DataLogger.cc
			 ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
			 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
			 ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
			 )

ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES} ${LIBRT} )
INSTALL(TARGETS ${EXE_NAME} 
		RUNTIME DESTINATION bin)

#################################################################################
# SETUP SDSE_CONFIGURATOR  #
IF (Qt5_FOUND)
  SET(EXE_NAME OpenSdseConfigurator)
  SET(SRC_LIST ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/main.cc
               ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/XParamMainWindow.cc
               ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/XMLDoc.cc
               ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/XNode.cc
               ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
               ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp)

  # Qt5 specific functions  #
  SET(HDR_LIST   ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/XParamMainWindow.hh)
  SET(FORMS_LIST ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/XParamMainWindow.ui)

  QT5_WRAP_CPP(SDSE_CONFIGURATOR_HEADERS_MOC ${HDR_LIST})
  QT5_WRAP_UI(SDSE_CONFIGURATOR_FORMS_HEADERS ${FORMS_LIST})

  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

  ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST} ${SDSE_CONFIGURATOR_HEADERS_MOC} ${SDSE_CONFIGURATOR_FORMS_HEADERS})
  TARGET_LINK_LIBRARIES(${EXE_NAME} Qt5::Widgets)
  INSTALL(TARGETS ${EXE_NAME} 
          RUNTIME DESTINATION bin)
ENDIF (Qt5_FOUND)

if (NOT Qt5_FOUND)
	MESSAGE(STATUS "Qt5 not found! Please check your settings - SdseConfigurator Tool not configured for compilation")
endif (NOT Qt5_FOUND)

##################################################################################
# Executable & Files Installation #
INSTALL(FILES 
	${10_DATA_LOGGER_FED_GNUPLOT_DIRECTORY}/multiplot_sdse.gnuplot
        DESTINATION logs)
        
INSTALL(FILES 
	${10_DATA_LOGGER_FED_GNUPLOT_DIRECTORY}/gnuplot_launcher.sh
        DESTINATION logs)
        
INSTALL(FILES 
	${10_DATA_LOGGER_FED_GNUPLOT_DIRECTORY}/SimulationVsRealTime.gnuplot
        DESTINATION logs)
        
INSTALL(FILES 
	${10_DATA_LOGGER_FED_GNUPLOT_DIRECTORY}/gnuplot_launcher_simu_vs_real_time.sh
        DESTINATION logs)

INSTALL(FILES 
	${INIT_FILES_DIRECTORY}/sdse_init_parameters.xml
        DESTINATION bin)

INSTALL(FILES 
	${INIT_FILES_DIRECTORY}/sdse_hla13.fed
        DESTINATION fom)
        
INSTALL(FILES 
	${INIT_FILES_DIRECTORY}/sdse_hla1516.xml
        DESTINATION fom)

INSTALL(FILES 
	${INIT_FILES_DIRECTORY}/open_sdse_logo.png
        DESTINATION bin/..)

IF (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  INSTALL(FILES 
  ${INIT_FILES_DIRECTORY}/Local_Run_bash.sh
        DESTINATION scripts)
   INSTALL(FILES 
  ${INIT_FILES_DIRECTORY}/Local_Run_bash_1516e.sh
        DESTINATION scripts)  
	INSTALL(FILES 
  ${INIT_FILES_DIRECTORY}/Local_Run_bash_1516e_External_Efcs.sh
        DESTINATION scripts)     

ELSEIF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
  INSTALL(FILES 
  ${INIT_FILES_DIRECTORY}/Local_Run_bash_mac.sh
        DESTINATION scripts)
  INSTALL(DIRECTORY
  ${INIT_FILES_DIRECTORY}/OSX_Scripts
        DESTINATION scripts)

ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

ELSE()

# PATHS CONFIGURATION FOR THE WHOLE PROJECT ##################
# FOR 01_JOYSTICK FEDERATE #
SET(1_JOYSTICK_FED_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/01_JOYSTICK)
ADD_SUBDIRECTORY(${1_JOYSTICK_FED_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${1_JOYSTICK_FED_SOURCE_DIRECTORY}
					${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR
					)
					
# FOR COMMON SOURCES FILES #
SET(COMMON_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/COMMON)
ADD_SUBDIRECTORY(${COMMON_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${COMMON_SOURCE_DIRECTORY})

# FOR SDSE_CONFIGURATOR TOOL #
SET(SDSE_CONFIGURATOR_SOURCE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/SDSE_CONFIGURATOR)
ADD_SUBDIRECTORY(${SDSE_CONFIGURATOR_SOURCE_DIRECTORY})
INCLUDE_DIRECTORIES(${SDSE_CONFIGURATOR_SOURCE_DIRECTORY})

# INIT CONFIGURATION PATH #
SET(INIT_FILES_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/INIT_FILES)
					
#################################################################################
#  SETUP 1_JOYSTICK FEDERATE (FED 1) #
SET(EXE_NAME JoystickFederateHla13)
SET(SRC_LIST ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/JoystickFederateMainHla13.cc
             ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/JoystickFederateHla13.cc
             ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/Joystick.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

# ONLY COMPILE FULLY ON LINUX (because of linux/joystick API)
# Compilation on Mac gives a federate which publish 0 for all commands
  ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
  TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI_LIBRARIES} ${LIBRT})
  INSTALL(TARGETS ${EXE_NAME} 
          RUNTIME DESTINATION bin)
          
SET(EXE_NAME JoystickFederateHla1516e)
SET(SRC_LIST ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/JoystickFederateMainHla1516e.cc
             ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/JoystickFederateHla1516e.cc
             ${1_JOYSTICK_FED_SOURCE_DIRECTORY}/Joystick.cc
             ${COMMON_SOURCE_DIRECTORY}/MessageBuffer.cc
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/tinyxml2.cpp
             ${SDSE_CONFIGURATOR_SOURCE_DIRECTORY}/SdseParametersParser.cc
             )

# ONLY COMPILE FULLY ON LINUX (because of linux/joystick API)
# Compilation on Mac gives a federate which publish 0 for all commands
ADD_EXECUTABLE(${EXE_NAME} ${SRC_LIST})
TARGET_LINK_LIBRARIES (${EXE_NAME} ${RTI1516e_LIBRARIES} ${LIBRT})
INSTALL(TARGETS ${EXE_NAME} 
	  RUNTIME DESTINATION bin)
          
INSTALL(FILES 
	${INIT_FILES_DIRECTORY}/sdse_init_parameters.xml
        DESTINATION bin)

INSTALL(FILES 
	${INIT_FILES_DIRECTORY}/sdse_hla13.fed
        DESTINATION fom)
        
INSTALL(FILES 
	${INIT_FILES_DIRECTORY}/sdse_hla1516.xml
        DESTINATION fom)

INSTALL(FILES 
	${INIT_FILES_DIRECTORY}/open_sdse_logo.png
        DESTINATION bin/..)

IF (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  INSTALL(FILES 
  ${INIT_FILES_DIRECTORY}/Local_Run_bash.sh
        DESTINATION scripts)
   INSTALL(FILES 
  ${INIT_FILES_DIRECTORY}/Local_Run_bash_1516e.sh
        DESTINATION scripts)       

ELSEIF (${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
  INSTALL(FILES 
  ${INIT_FILES_DIRECTORY}/Local_Run_bash_mac.sh
        DESTINATION scripts)
  INSTALL(DIRECTORY
  ${INIT_FILES_DIRECTORY}/OSX_Scripts
        DESTINATION scripts)

ENDIF (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

ENDIF (NOT JOYSTICK_FED_ONLY)


MESSAGE(STATUS "************************************************************************")
MESSAGE(STATUS "************************************************************************")
MESSAGE(STATUS "**********                                                    **********")
MESSAGE(STATUS "********** ${CMAKE_PROJECT_NAME} has been successfully configured **********")
MESSAGE(STATUS "**********                                                    **********")
MESSAGE(STATUS "************************************************************************")
MESSAGE(STATUS "************************************************************************")
