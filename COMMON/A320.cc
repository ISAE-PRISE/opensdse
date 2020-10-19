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
//
// ----------------------------------------------------------------------------
// Note: A lot of coefficients in this file has been borrowed from:
// - JsBSim: http://jsbsim.sourceforge.net/
// - Flightgear: http://www.flightgear.org/
// ----------------------------------------------------------------------------
//

#include "A320.hh"

// Dimensions
const double A320::wingarea  = 122.6; // in m^2 
const double A320::wingspan  = 34.09; // in m
const double A320::chord     = 4.29;  // in m
const double A320::htailarea = 31.0;  // in m^2
const double A320::htailarm  = 13.53; // in m 
const double A320::vtailarea = 21.48; // in m^2 
const double A320::vtailarm  = 0.0;   // in m 

// Mass and Inertia
// EmptyInertia from flight gear A320-211 model
const double A320::EmptyMass = 42175.0;    // in kg
const double A320::FullMass  = 73500.0;    // in kg
const double A320::EmptyIxx  = 829629.07;  // in kg * m^2
const double A320::EmptyIyy  = 2352871.55; // in kg * m^2
const double A320::EmptyIzz  = 3120134.35; // in kg * m^2
const double A320::EmptyIxy  = 0.0;        // in kg * m^2
const double A320::EmptyIxz  = 8677.24;    // in kg * m^2
const double A320::EmptyIyz  = 0.0;        // in kg * m^2
const double A320::FullIxx   = 1278371.16; // in kg * m^2
const double A320::FullIyy   = 3781272.53; // in kg * m^2
const double A320::FullIzz   = 4877656.08; // in kg * m^2
const double A320::FullIxy   = 0.0;        // in kg * m^2
const double A320::FullIxz   = 13558.19;   // in kg * m^2
const double A320::FullIyz   = 0.0;        // in kg * m^2

// CG Location
const FGColumnVector3 A320::CG = FGColumnVector3(17.0,0.0,-1.0);  // in m

// Engine location
const FGColumnVector3 A320::LeftEngineLocation  = FGColumnVector3(17.0,-5.0,-1.0);  // in m
const FGColumnVector3 A320::RightEngineLocation = FGColumnVector3(17.0,5.0,-1.0);  // in m

// Engine Specific Fuel Consumption
const double A320::SFC = 1.69E-5; // in kg/(s*N)

// Aerodynamic coefficients

// Drag 
const double A320::CD0 = 0.016;
const double A320::CDalpha[27][5] = {
  { 0.0000,  0.0000,  1.0000, 25.0000, 40.0000},
  {-0.0873,  0.0041,  0.0000,  0.0005,  0.0014},
  {-0.0698,  0.0013,  0.0004,  0.0025,  0.0041},
  {-0.0524,  0.0001,  0.0023,  0.0059,  0.0084},
  {-0.0349,  0.0003,  0.0057,  0.0108,  0.0141},
  {-0.0175,  0.0020,  0.0105,  0.0172,  0.0212},
  { 0.0000,  0.0052,  0.0168,  0.0251,  0.0399},
  { 0.0175,  0.0099,  0.0248,  0.0346,  0.0502},
  { 0.0349,  0.0162,  0.0342,  0.0457,  0.0621},
  { 0.0524,  0.0240,  0.0452,  0.0583,  0.0755},
  { 0.0698,  0.0334,  0.0577,  0.0724,  0.0904},
  { 0.0873,  0.0442,  0.0718,  0.0881,  0.1068},
  { 0.1047,  0.0566,  0.0874,  0.1053,  0.1248},
  { 0.1222,  0.0706,  0.1045,  0.1240,  0.1443},
  { 0.1396,  0.0860,  0.1232,  0.1442,  0.1654},
  { 0.1571,  0.0962,  0.1353,  0.1573,  0.1790},
  { 0.1745,  0.1069,  0.1479,  0.1708,  0.1930},
  { 0.1920,  0.1180,  0.1610,  0.1849,  0.2075},
  { 0.2094,  0.1298,  0.1746,  0.1995,  0.2226},
  { 0.2269,  0.1424,  0.1892,  0.2151,  0.2386},
  { 0.2443,  0.1565,  0.2054,  0.2323,  0.2564},
  { 0.2618,  0.1727,  0.2240,  0.2521,  0.2767},
  { 0.2793,  0.1782,  0.2302,  0.2587,  0.2835},
  { 0.2967,  0.1716,  0.2227,  0.2507,  0.2753},
  { 0.3142,  0.1618,  0.2115,  0.2388,  0.2631},
  { 0.3316,  0.1475,  0.1951,  0.2214,  0.2451},
  { 0.3491,  0.1097,  0.1512,  0.1744,  0.1966}
}; 
const double A320::CDde = 0.05; 
const double A320::CDbeta = 0.2;
const double A320::CDgear = 0.04;
const double A320::CDspeedbrake = 0.04;

// Side
const double A320::CYb[3][2] = {
  {-0.3500,  0.5000},
  { 0.0000,  0.0000},
  { 0.3500, -0.5000}
};
  
// Lift
const double A320::CLalpha[18][6] = {
  { 0.0000,  0.0000,  1.0000,  9.0000, 10.0000, 40.0000},
  {-0.0900, -0.2200, -0.2200, -0.1200, -0.1200,  0.3200},
  { 0.0000,  0.2500,  0.2500,  0.3500,  0.3500,  0.7500},
  { 0.0900,  0.7300,  0.7300,  0.8300,  0.8300,  1.2300},
  { 0.1000,  0.8300,  0.8300,  0.9300,  0.9300,  1.3300},
  { 0.1200,  0.9200,  0.9200,  1.0200,  1.0200,  1.4200},
  { 0.1400,  1.0200,  1.0200,  1.1200,  1.1200,  1.5200},
  { 0.1600,  1.0800,  1.0800,  1.1800,  1.1800,  1.5800},
  { 0.1700,  1.1300,  1.1300,  1.2300,  1.2300,  1.6300},
  { 0.1900,  1.1900,  1.1900,  1.2900,  1.2900,  1.6900},
  { 0.2100,  1.2500,  1.2500,  1.3500,  1.3500,  1.7700},
  { 0.2400,  1.3500,  1.3700,  1.4700,  1.4800,  1.9300},
  { 0.2600,  1.4400,  1.4700,  1.5700,  1.6200,  2.1200},
  { 0.2800,  1.4700,  1.5100,  1.6100,  1.7800,  2.4000},
  { 0.3000,  1.5000,  1.5600,  1.6600,  1.9000,  2.3000},
  { 0.3200,  1.4700,  1.6100,  1.6000,  1.7000,  2.0300},
  { 0.3400,  1.3500,  1.5000,  1.4100,  1.5000,  1.5300},
  { 0.3600,  1.1500,  1.2000,  1.2000,  1.2000,  1.2000}
};
const double A320::CLde = 0.1930;

// Roll
const double A320::Clb[3][2] = {
  {-0.3500,  0.01000},
  { 0.0000,  0.00000},
  { 0.3500, -0.01000}
};

const double A320::Clp = -0.5; 
const double A320::Clr = 0.005;
const double A320::Clda = 0.2;
const double A320::Cldr = 0.005;

// Pitch
const double A320::Cm0[2][2] = {
  { 0.0000,  0.04000},
  {40.0000, -0.10000},
};
const double A320::Cmalpha = -0.83;   // Modified by David on 21/03/11 (=-4.0 Flight Gear) 
const double A320::Cmde = -1.5;
const double A320::Cmq = -30.0;       // Modified by David on 21/03/11 (=-10.0 Flight Gear)
const double A320::Cmalphadot = -12.0;

// Yaw
const double A320::Cnr = -0.04;
const double A320::Cnb = 0.2;
const double A320::Cndr = -0.5;

// Control Surfaces 
// Angle in Degree, freq in Hz and rate in Degree/s
const double A320::ra_damping = 0.70;
const double A320::ra_frequency = 50.0;
const double A320::ra_minpos = -25.0;
const double A320::ra_maxpos = 25.0;
const double A320::ra_maxrate = 35.0;

const double A320::la_damping = 0.70;
const double A320::la_frequency = 50.0;
const double A320::la_minpos = -25.0;
const double A320::la_maxpos = 25.0;
const double A320::la_maxrate = 35.0;

const double A320::el_damping = 0.85;
const double A320::el_frequency = 25.0;
const double A320::el_minpos = -30.0;
const double A320::el_maxpos = 17.0;
const double A320::el_maxrate = 32.0;

const double A320::ru_damping = 0.70;
const double A320::ru_frequency = 25.0;
const double A320::ru_minpos = -25.0;
const double A320::ru_maxpos = 25.0;
const double A320::ru_maxrate = 35.0;

const double A320::fl_damping = 0.85;
const double A320::fl_frequency = 25.0;
const double A320::fl_minpos = 0.0;
const double A320::fl_maxpos = 30.0;
const double A320::fl_maxrate = 2.2;

const double A320::sp_damping = 0.85;
const double A320::sp_frequency = 25.0;
const double A320::sp_minpos = 0.0;
const double A320::sp_maxpos = 45.0;
const double A320::sp_maxrate = 35.0;

const double A320::st_damping = 0.85;
const double A320::st_frequency = 25.0;
const double A320::st_minpos = -13.5;
const double A320::st_maxpos = 4.0;
const double A320::st_maxrate = 0.5;

const double A320::ge_damping = 0.85;
const double A320::ge_frequency = 35.0;
const double A320::ge_minpos = 0.0;
const double A320::ge_maxpos = 58.0;
const double A320::ge_maxrate = 20.0;
