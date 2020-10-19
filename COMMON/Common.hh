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

#ifndef __COMMON_HH_DEF__
#define __COMMON_HH_DEF__

// To get compilation and git version information
#include "CompilationInfo.hh" 
#include "GitVersionInfo.hh" 

#define SYNCHRO_INITIALISATION 1
#define SYNCHRO_SIMULATION 1

/******* TIME_MANAGEMENT FLAG *********/
// 0 => Data Flow execution mode 
// 1 => Time Management execution mode  
// 2 => A completer : Time management avec NERA/TARA 
#define TIME_MANAGEMENT 1

//#define DEBUG_STEP_BY_STEP

#define DEBUG_JOYSTICK
#define DEBUG_JOYSTICK_FED

#define DEBUG_EFCS
#define DEBUG_EFCS_FED

#define DEBUG_CONTROL_SURFACES
#define DEBUG_CONTROL_SURFACES_FED

#define DEBUG_ENGINES
#define DEBUG_ENGINES_FED

#define DEBUG_FLIGH_DYNAMICS
#define DEBUG_FLIGH_DYNAMICS_FED

#define DEBUG_SENSORS
#define DEBUG_SENSORS_FED

#define DEBUG_VISUALIZATION
#define DEBUG_VISUALIZATION_FED

#define DEBUG_ENVIRONMENT
#define DEBUG_ENVIRONMENT_FED

#define DEBUG_COCKPIT
#define DEBUG_COCKPIT_FED

#define DEBUG_DATA_LOGGER
#define DEBUG_DATA_LOGGER_FED

#define TRACE_INIT 0
#define TRACE_SIMU 1
#define TRACE_CYCLES 0
#define JOYSTICK_ENABLED 1

#define NB_CYCLES_50_HZ 75000
#define NB_CYCLES_100_HZ 150000

// XML Parser
#include "SdseParametersParser.hh"
using namespace parser;

#endif // __COMMON_HH_DEF__
