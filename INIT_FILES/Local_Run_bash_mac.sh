# CHANGE HERE ACCORDING TO YOUR ENVIRONNMENT

SDSE_BIN_DIR=$(cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)
export CERTI_RUN_DIR=/usr/local
#export QT_ENV_SCRIPT_DIR=$HOME
#export SDSE_BIN_DIR=$HOME/PRISE/run/bin

# Export CERTI paths
source $CERTI_RUN_DIR/share/scripts/myCERTI_env.sh

# Configuration environnement QT
#source $QT_ENV_SCRIPT_DIR/my_QT_env.sh

# Emplacements des scripts OSX
OSXSCRIPTS="OSX_Scripts/"

# On laisse au systeme le temps d'ouvrir des shells
SLEEP_TIME=0.1

sleep $SLEEP_TIME
open $OSXSCRIPTS/rtig.command
sleep $SLEEP_TIME
open $OSXSCRIPTS/Cockpit_Fed.command
sleep $SLEEP_TIME
open $OSXSCRIPTS/EFCS_Fed.command
sleep $SLEEP_TIME
open $OSXSCRIPTS/Control_Surfaces_Fed.command
sleep $SLEEP_TIME
open $OSXSCRIPTS/Engines_Fed.command
sleep $SLEEP_TIME 
open $OSXSCRIPTS/Sensors_Fed.command
sleep $SLEEP_TIME
open $OSXSCRIPTS/Environment_Fed.command
sleep $SLEEP_TIME
open $OSXSCRIPTS/Visualization_Fed.command
sleep $SLEEP_TIME
open $OSXSCRIPTS/Joystick_Fed.command
sleep $SLEEP_TIME
open $OSXSCRIPTS/Flight_Dynamics_Fed.command
