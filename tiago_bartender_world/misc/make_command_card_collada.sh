#!/bin/bash
# simple script to create new command-card collada meshes
# from the 'master' template. This assumes that the needed
# texture files already exist.
# 
# Usage: 
# roscd tiage_bartender_world/meshes
# ../misc/make_comand_card_collada.sh <commandname> <atag-id>
#
# Example:
# ../msic/make_command_card_collada.sh cancel 402
#
echo $1
echo $2
export fname='cc_'$1'.dae'
echo $fname
export atagfname='cc_'$1'_atag_'$2'.dae'
echo $atagfname
#
export texname='cc_'$1'.png'
export atagtexname='cc_'$1'_atag_'$2'.png'
echo $texname $atagtexname


sed -e"s/cc_card.png/$texname/" cc_card.dae > $fname
sed -e"s/cc_card.png/$atagtexname/" cc_card.dae > $atagfname
#
