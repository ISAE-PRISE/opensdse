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

# CHANGE HERE ACCORDING TO YOUR ENVIRONNMENT
export CERTI_RUN_DIR=~/Dev/CERTI/certi_4.0.0/install/
export PATH=/usr/local/Qt5_older/5.2.0/gcc_64:$PATH
export LD_LIBRARY_PATH=/usr/local/Qt5_older/5.2.0/gcc_64/lib:${LD_LIBRARY_PATH}
export SDSE_BIN_DIR=$PWD/../bin


source $CERTI_RUN_DIR/share/scripts/myCERTI_env.sh



# On laisse au systeme le temps d'ouvrir des shells
SLEEP_TIME=0.2

sleep $SLEEP_TIME
gnome-terminal -t "RTIG" -e $CERTI_RUN_DIR/bin/rtig &
sleep $SLEEP_TIME
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 1 : JOYSTICK_FED" -e "$SDSE_BIN_DIR/JoystickFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 2 : EFCS_FED" -e "$SDSE_BIN_DIR/EfcsFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="EXTERNAL EFCS" -e "$SDSE_BIN_DIR/ExternalEfcs 1" &
sleep $SLEEP_TIME
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 3 : CONTROL_SURFACES_FED" -e "$SDSE_BIN_DIR/ControlSurfacesFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 4 : ENGINES_FED" -e "$SDSE_BIN_DIR/EnginesFederateHla1516e" &
sleep $SLEEP_TIME 
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 6 : SENSORS_FED" -e "$SDSE_BIN_DIR/SensorsFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 7 : VISUALIZATION_FED" -e "$SDSE_BIN_DIR/VisualizationFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 8 : ENVIRONMENT_FED" -e "$SDSE_BIN_DIR/EnvironmentFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 10 : DATA_LOGGER_FED" -e "$SDSE_BIN_DIR/DataLoggerFederateHla1516e" &
sleep $SLEEP_TIME
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 9 : COCKPIT_FED" -e "$SDSE_BIN_DIR/CockpitFederateHla1516e" &
sleep 1
gnome-terminal --working-directory=$SDSE_BIN_DIR --title="FED 5 : FLIGHT_DYNAMICS_FED" -e "$SDSE_BIN_DIR/FlightDynamicsFederateHla1516e" &
