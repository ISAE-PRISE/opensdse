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

#include "FCUwindow.hh"
#include "ui_FCUwindow.h"
#include "Common.hh"
#include <math.h>
#include <iostream>

#include <QPixmap>

using namespace std;

// ----------------------------------------------------------------------------
// Constructor 
FCUwindow::FCUwindow(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FCUwindow)
{
    ui->setupUi(this);
    initialize();
}

// ----------------------------------------------------------------------------
// Destructor
FCUwindow::~FCUwindow()
{
	delete ui;
}

// ----------------------------------------------------------------------------
//
void FCUwindow::initialize()
{
	int HeadingRef, AltitudeRef, IasRef;
	// Get values from xml file
	XMLDocument* params = loadFileToDoc(DEFAULT_XML_PATH);
	// Get values from xml file
	vector<string> attribute_path(3);
	attribute_path[0] = "COCKPIT";
	attribute_path[1] = "FcuInit";
	attribute_path[2] = "HeadingRef";
	HeadingRef = getIntValue(params, attribute_path);
	attribute_path[2] = "AltitudeRef";
	AltitudeRef = getIntValue(params, attribute_path);
	attribute_path[2] = "IasRef";
	IasRef = getIntValue(params, attribute_path);
	delete params;
	
    // Init primary variables
    Autopilot_AP = true ;
    Autopilot_athr = true ;

    Autopilot_alt = AltitudeRef ;
    Autopilot_hdg = HeadingRef ;
    Autopilot_spd = IasRef ;
    Autopilot_vs = 0 ;
    Autopilot_mach = 0 ;
    Alti = 0 ;

    // Init secondary variables
    Autopilot_ap1 = true ;
    Autopilot_ap2 = false ;
    Autopilot_exped = false ;
    Autopilot_appr = false ;
    Autopilot_loc = false ;

    HDG_dial_value = 0 ;
    SPD_dial_value = 0 ;
    ALT_dial_value = 0 ;
    ALT_select_sensitivity=1000;
    SPD_MACH_dial_value = 0; //0 for spd, 1 for mach

    //Create Icons
    create_icons();

    // Init Display
    update_all_disp();

}

// ----------------------------------------------------------------------------
//
void FCUwindow::create_icons()
{
    QPixmap pix_loc_on(":/FCUrc/Images/btn_loc_on.jpg");
    icon_loc_on = (pix_loc_on);
    QPixmap pix_loc_off(":/FCUrc/Images/btn_loc_off.jpg");
    icon_loc_off = (pix_loc_off);
    QPixmap pix_ap1_on(":/FCUrc/Images/btn_ap1_on.jpg");
    icon_ap1_on = (pix_ap1_on);
    QPixmap pix_ap1_off(":/FCUrc/Images/btn_ap1_off.jpg");
    icon_ap1_off = (pix_ap1_off);
    QPixmap pix_ap2_on(":/FCUrc/Images/btn_ap2_on.jpg");
    icon_ap2_on = (pix_ap2_on);
    QPixmap pix_ap2_off(":/FCUrc/Images/btn_ap2_off.jpg");
    icon_ap2_off = (pix_ap2_off);
    QPixmap pix_athr_on(":/FCUrc/Images/btn_athr_on.jpg");
    icon_athr_on = (pix_athr_on);
    QPixmap pix_athr_off(":/FCUrc/Images/btn_athr_off.jpg");
    icon_athr_off = (pix_athr_off);
    QPixmap pix_exped_on(":/FCUrc/Images/btn_exped_on.jpg");
    icon_exped_on = (pix_exped_on);
    QPixmap pix_exped_off(":/FCUrc/Images/btn_exped_off.jpg");
    icon_exped_off = (pix_exped_off);
    QPixmap pix_appr_on(":/FCUrc/Images/btn_appr_on.jpg");
    icon_appr_on = (pix_appr_on);
    QPixmap pix_appr_off(":/FCUrc/Images/btn_appr_off.jpg");
    icon_appr_off = (pix_appr_off);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_all_disp()
{
    update_SPD_disp();
    update_HDG_disp();
    update_ALT_disp();
    update_VS_disp();
    update_ATHR_disp();
    update_AP1_disp();
    update_AP2_disp();
    update_APPR_disp();
    update_LOC_disp();
    update_EXPED_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_SPD_disp()
{
    if     (SPD_MACH_dial_value == 0)  ui->lcdn_SPD->display(Autopilot_spd);
    else if(SPD_MACH_dial_value == 1)  ui->lcdn_SPD->display(Autopilot_mach);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_HDG_disp()
{
    ui->lcdn_HDG->display(Autopilot_hdg);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_ALT_disp()
{
    ui->lcdn_ALT->display(Autopilot_alt);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_VS_disp()
{
    ui->lcdn_VS->display(Autopilot_vs);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_ATHR_disp()
{
    if (Autopilot_athr) ui->button_athr->setIcon(icon_athr_on);
    else ui->button_athr->setIcon(icon_athr_off);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_LOC_disp()
{
    if (Autopilot_loc) ui->button_loc->setIcon(icon_loc_on);
    else ui->button_loc->setIcon(icon_loc_off);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_AP1_disp()
{
    if (Autopilot_ap1) ui->button_ap1->setIcon(icon_ap1_on);
    else ui->button_ap1->setIcon(icon_ap1_off);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_AP2_disp()
{
    if (Autopilot_ap2) ui->button_ap2->setIcon(icon_ap2_on);
    else ui->button_ap2->setIcon(icon_ap2_off);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_EXPED_disp()
{
    if (Autopilot_exped) ui->button_exped->setIcon(icon_exped_on);
    else ui->button_exped->setIcon(icon_exped_off);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_APPR_disp()
{
    if (Autopilot_appr) ui->button_appr->setIcon(icon_appr_on);
    else ui->button_appr->setIcon(icon_appr_off);
}

// ----------------------------------------------------------------------------
//
void FCUwindow::update_AP_state()
{
    if (!Autopilot_ap1 && !Autopilot_ap2) Autopilot_AP=false;
    else {
        Autopilot_AP=true;
        Autopilot_athr=true;
        }
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_dial_HDG_valueChanged(int value)
{
    if (value < HDG_dial_value && value !=0) Autopilot_hdg = Autopilot_hdg - 1;
    if (value > HDG_dial_value && value !=30) Autopilot_hdg = Autopilot_hdg + 1;
    if (Autopilot_hdg > 360) Autopilot_hdg=1;
    if (Autopilot_hdg < 1) Autopilot_hdg=360;
    HDG_dial_value=value;
    update_HDG_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_dial_SPD_valueChanged(int value)
{

    //update mach value if in mach mode
    if(SPD_MACH_dial_value == 1)
    {
        //change mach value
        if (value < SPD_dial_value && value !=0) Autopilot_mach = Autopilot_mach - 0.01;
        if (value > SPD_dial_value && value !=50) Autopilot_mach = Autopilot_mach + 0.01;
        if (Autopilot_mach > Max_mach/100.0) Autopilot_mach = Max_mach/100.0;
        if (Autopilot_mach < Min_mach/100.0) Autopilot_mach = Min_mach/100.0;
        SPD_dial_value = value;

        //calculate new speed
        set_spd_from_Mach();


    }

    //update speed value if in speed mode (mach limitations are used here)
    if(SPD_MACH_dial_value == 0)
    {
        //change speed value
        if (value < SPD_dial_value && value !=0) Autopilot_spd = Autopilot_spd - 1;
        if (value > SPD_dial_value && value !=50) Autopilot_spd = Autopilot_spd + 1;
        SPD_dial_value = value;
        //check mach limitations
        set_Mach_from_spd();
        if(Autopilot_mach > Max_mach/100.0)
        {
            Autopilot_mach = Max_mach/100.0;
            set_spd_from_Mach();
        }
        if(Autopilot_mach < Min_mach/100.0)
        {
            Autopilot_mach = Min_mach/100.0;
            set_spd_from_Mach();
        }
    }

    update_SPD_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_slider_ALT_SELECT_valueChanged(int value)
{
    if (value == 1) ALT_select_sensitivity = 1000;
    if (value == 0) ALT_select_sensitivity = 100;
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_dial_ALT_valueChanged(int value)
{
    if (value < ALT_dial_value && value !=0) Autopilot_alt = Autopilot_alt - ALT_select_sensitivity;
    if (value > ALT_dial_value && value !=10) Autopilot_alt = Autopilot_alt + ALT_select_sensitivity;
    if (Autopilot_alt > Max_alt) Autopilot_alt = Max_alt;
    if (Autopilot_alt < Min_alt) Autopilot_alt = Min_alt;
    ALT_dial_value = value;
    update_ALT_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_dial_VS_valueChanged(int value)
{
    if (value < VS_dial_value && value !=0) Autopilot_vs = Autopilot_vs - 100;
    if (value > VS_dial_value && value !=30) Autopilot_vs = Autopilot_vs + 100;
    if (Autopilot_vs > Max_vs) Autopilot_vs = Max_vs;
    if (Autopilot_vs < Min_vs) Autopilot_vs = Min_vs;
    VS_dial_value = value;
    update_VS_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_button_athr_clicked()
{
    if (!Autopilot_AP) 
    {
        Autopilot_athr=!Autopilot_athr;
        update_ATHR_disp();
    }
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_button_loc_clicked()
{
    Autopilot_loc=!Autopilot_loc;
    update_LOC_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_button_ap1_clicked()
{
    Autopilot_ap1=!Autopilot_ap1;
    update_AP_state();
    update_AP1_disp();
    update_ATHR_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_button_ap2_clicked()
{
    Autopilot_ap2=!Autopilot_ap2;
    update_AP_state();
    update_AP2_disp();
    update_ATHR_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_button_exped_clicked()
{
    Autopilot_exped=!Autopilot_exped;
    update_EXPED_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_button_appr_clicked()
{
    Autopilot_appr=!Autopilot_appr;
    update_APPR_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::on_dial_switch_SPD_MACH_valueChanged(int value)
{
    QString spd  = "SPD";
    QString mach = "MACH";
    SPD_MACH_dial_value = 1 - SPD_MACH_dial_value;
    if     (SPD_MACH_dial_value == 0){ui->label_SPD->setText(spd);}
    else if(SPD_MACH_dial_value == 1){ui->label_SPD->setText(mach);}

    //if now in mach mode, update mach with last speed value
    if(SPD_MACH_dial_value == 1) set_Mach_from_spd();

    update_SPD_disp();
}

// ----------------------------------------------------------------------------
//
void FCUwindow::set_Alti(double input)
{
     Alti = input*0.3048;

     //update speed if in mach mode
     if(SPD_MACH_dial_value == 1) set_spd_from_Mach();

     //check mach limitations if in speed mode
     if(SPD_MACH_dial_value == 0)
     {
         set_Mach_from_spd();
         if(Autopilot_mach > Max_mach/100.0)
         {
             Autopilot_mach = Max_mach/100.0;
             set_spd_from_Mach();
         }
         if(Autopilot_mach < Min_mach/100.0)
         {
             Autopilot_mach = Min_mach/100.0;
             set_spd_from_Mach();
         }
     }

     update_SPD_disp();

}

// ----------------------------------------------------------------------------
//
void FCUwindow::set_Mach_from_spd()
{
    double Temperature = 15 - 6.5*Alti/1000;
    double SoundSpeed = 331.3*sqrt(1+Temperature/273.15);
    Autopilot_mach = Autopilot_spd*0.514444444/SoundSpeed;
}

// ----------------------------------------------------------------------------
//
void FCUwindow::set_spd_from_Mach()
{
    double Temperature = 15 - 6.5*Alti/1000;
    double SoundSpeed = 331.3*sqrt(1+Temperature/273.15);
    Autopilot_spd = Autopilot_mach*SoundSpeed/0.514444444;
}
