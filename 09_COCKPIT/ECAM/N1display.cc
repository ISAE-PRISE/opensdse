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
#include "N1display.hh"

#include <QtSvg/QSvgRenderer>
#include <QtSvg/QSvgGenerator>
#include <QDebug>

#include <math.h>
#include <iostream>

// ----------------------------------------------------------------------------
// Constructor 
N1display::N1display( QWidget *parent ) :
    QGraphicsView( parent ),
    m_renderer(Native),
    item_background    ( 0 ),
    item_scale  ( 0 ),
    item_window_green( 0 ),
    item_window_red( 0 ),
    item_arrow_green ( 0 ),
    item_arrow_red ( 0 ),
    color_mode(true)
{
    actual_pwr = 0.0;
    previous_pwr = 0.0;
    red_step=10.0;

    this->setHorizontalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
    this->setVerticalScrollBarPolicy( Qt::ScrollBarAlwaysOff );
    this->QGraphicsView::setSceneRect(0,0,147,147);

    setScene(new QGraphicsScene(this));
    setRenderHint(QPainter::HighQualityAntialiasing, true);
}

// ----------------------------------------------------------------------------
// Destructor
N1display::~N1display( void ) 
{
    clean();
}

// ----------------------------------------------------------------------------
// 
void N1display::init( void ) 
{
    clean();

    item_background     = new QGraphicsSvgItem(":/ECAMrc/Images/black_background.svg");
    item_scale   = new QGraphicsSvgItem(":/ECAMrc/Images/ecam_scale.svg");
    item_window_green = new QGraphicsSvgItem(":/ECAMrc/Images/ecam_window_green.svg");
    item_window_red = new QGraphicsSvgItem(":/ECAMrc/Images/ecam_window_red.svg");
    item_arrow_green  = new QGraphicsSvgItem(":/ECAMrc/Images/arrow_green.svg");
    item_arrow_red  = new QGraphicsSvgItem(":/ECAMrc/Images/arrow_red.svg");

    double scaleX = (double) width()/324;
    double scaleY = (double) height()/324;

    item_background->setFlags(QGraphicsItem::ItemClipsToShape);
    item_background->setCacheMode(QGraphicsItem::NoCache);
    item_background->setTransform(QTransform::fromScale(scaleX,scaleY),true);
    item_background->setZValue(0);

    item_window_green->setFlags(QGraphicsItem::ItemClipsToShape);
    item_window_green->setCacheMode(QGraphicsItem::NoCache);
    item_window_green->setTransform(QTransform::fromScale(scaleX,scaleY),true);
    item_window_green->setZValue(2);

    item_window_red->setFlags(QGraphicsItem::ItemClipsToShape);
    item_window_red->setCacheMode(QGraphicsItem::NoCache);
    item_window_red->setTransform(QTransform::fromScale(scaleX,scaleY),true);
    item_window_red->setZValue(2);

    qreal startX = 190;
    qreal startY = 177;
    QTransform matrix;
    matrix.translate(startX,startY);
    item_window_text = new QGraphicsTextItem("000");
    item_window_text->setDefaultTextColor(QColor(50,255,0));
    item_window_text->setFont(QFont("Arial",36));
    item_window_text->setParentItem(item_window_green);
    item_window_text->setTransform(matrix,false);

    item_scale->setFlags(QGraphicsItem::ItemClipsToShape);
    item_scale->setCacheMode(QGraphicsItem::NoCache);
    item_scale->setTransform(QTransform::fromScale(scaleX,scaleY),true);
    item_scale->setZValue(1);

    item_arrow_green->setFlags(QGraphicsItem::ItemClipsToShape);
    item_arrow_green->setCacheMode(QGraphicsItem::NoCache);
    item_arrow_green->setTransform(QTransform::fromScale(scaleX,scaleY),true);
    item_arrow_green->setTransformOriginPoint( 160, 160 );
    item_arrow_green->setZValue(50);

    item_arrow_red->setFlags(QGraphicsItem::ItemClipsToShape);
    item_arrow_red->setCacheMode(QGraphicsItem::NoCache);
    item_arrow_red->setTransform(QTransform::fromScale(scaleX,scaleY),true);
    item_arrow_red->setTransformOriginPoint( 160, 160 );
    item_arrow_red->setZValue(50);

    scene()->clear();

    scene()->addItem( item_background );
    scene()->addItem( item_scale );
    scene()->addItem( item_window_green );
    scene()->addItem( item_window_red );
    scene()->addItem( item_arrow_green );
    scene()->addItem( item_arrow_red );
    item_window_red->hide();
    item_arrow_red->hide();

}

// ----------------------------------------------------------------------------
// 
void  N1display::set( double power ) 
{
    if ( power > 100.0 )
        actual_pwr = 100.0;
    else if ( power < 0.0 )
        actual_pwr = 0.0;
    else
        actual_pwr = power;
    update_view( );
}

// ----------------------------------------------------------------------------
// 
void N1display::clean( void ) 
{
    if ( item_background )     delete item_background;     item_background     = 0;
    if ( item_scale )   delete item_scale;   item_scale   = 0;
    if ( item_window_green ) delete item_window_green; item_window_green = 0;
    if ( item_arrow_green )  delete item_arrow_green;  item_arrow_green  = 0;
    if ( item_window_red ) delete item_window_red; item_window_red = 0;
    if ( item_arrow_red )  delete item_arrow_red;  item_arrow_red  = 0;
}

// ----------------------------------------------------------------------------
// 
void N1display::update_view( void ) 
{

    if ( previous_pwr != actual_pwr ) 
    {
        double spin_cur = -27+actual_pwr*1.8;
        double spin_prv = -27+previous_pwr*1.8;

        previous_pwr = actual_pwr;

        item_arrow_green->setRotation( -spin_prv );
        item_arrow_green->setRotation( spin_cur );
        item_arrow_red->setRotation( -spin_prv );
        item_arrow_red->setRotation( spin_cur );

        QString pwr;
        pwr=pwr.setNum(floor(actual_pwr));
        if ( (actual_pwr < red_step && color_mode) || (actual_pwr >= red_step && !color_mode) ) switch_color_mode();

        item_window_text->setPlainText(QString(pwr));
        scene()->update();
    }
}

// ----------------------------------------------------------------------------
// 
void N1display::switch_color_mode( void ) 
{

    if ( color_mode ) 
    {
        item_window_text->setDefaultTextColor(QColor(255,0,0));
        item_window_green->hide();
        item_window_red->show();
        item_arrow_green->hide();
        item_arrow_red->show();
        item_window_text->setParentItem(item_window_red);
    }
    else 
    {
        item_window_text->setDefaultTextColor(QColor(50,255,0));
        item_window_red->hide();
        item_window_green->show();
        item_arrow_red->hide();
        item_arrow_green->show();
        item_window_text->setParentItem(item_window_green);
    }
    color_mode=!color_mode;
}
