#!/bin/bash


#CUR_DIR=$(pwd)
CUR_SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SIM=Simulation
SIM_DIR=${CUR_SCRIPT_DIR}/${SIM}

# run to change directory and exit
ExitSetup()
{
  cd "$CUR_DIR"
  echo -e "\n ****  Exiting Simulation Setup  ****\n"
  exit $?
}


echo 
echo "Checking MORSE installation..."

PKG_OK=$(dpkg-query -W --showformat='${Status}\n' morse-simulator|grep "install ok installed")
if [ "" == "$PKG_OK" ]; then
	echo "No morse-simulator found"
	ExitSetup
else 
    echo $PKG_OK
fi

echo 
echo "Checking MORSE configuration..."

SETUP_OK=$(morse check 2>&1| grep --only-matching "correctly setup to run MORSE")
if [ "" == "$SETUP_OK" ]; then
	echo "MORSE is not correctly configured"
	ExitSetup
else 
    echo $SETUP_OK
fi



echo -e "\n **** Running Accmet Simulation using MORSE **** "
echo -e "================================================= \n \n"

morse import -f ${SIM_DIR} >> /dev/null 2>&1
morse run Simulation

exit 0
