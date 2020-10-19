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

#include "ECAMwindow.hh"
#include "ui_ECAMwindow.h"
#include "math.h"

// ----------------------------------------------------------------------------
// Constructor 
ECAMwindow::ECAMwindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ECAMwindow)
{
    ui->setupUi(this);
    ui->N1left->init();
    ui->N1right->init();
    max_thrust_engine=250000;
    pressureSL=101325.0;
    machratio = 1;
    pressureratio = 1;
    N1powerleft = 0;
    N1powerright = 0;
}

// ----------------------------------------------------------------------------
// Destructor 
ECAMwindow::~ECAMwindow()
{
    delete ui;
}

// ----------------------------------------------------------------------------
// 
void ECAMwindow::consider_new_data()
{
    if (SDSE_mach < 0.5) machratio = 1 - 0.8 * SDSE_mach;
    else machratio = 0.7 - 0.2 * SDSE_mach;

    pressureratio = SDSE_pressure/pressureSL * pow(1+0.2*SDSE_mach*SDSE_mach,3.5);

    N1powerleft = 100*SDSE_left_engine_thr/(max_thrust_engine*machratio*pressureratio);
    N1powerright = 100*SDSE_right_engine_thr/(max_thrust_engine*machratio*pressureratio);

    ui->N1left->set(N1powerleft);
    ui->N1right->set(N1powerright);
}
