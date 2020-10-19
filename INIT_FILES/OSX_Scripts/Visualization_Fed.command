#!/bin/bash
PRISE_RUN_DIR=`echo "$(dirname "$0")" | sed -e "s/\/OSX_Scripts//"`
cd $PRISE_RUN_DIR
./VisualizationFederateHla13
