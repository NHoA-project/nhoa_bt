#!/bin/bash
HOME_PATH="${HOME}" # Modify with your home directory.
DAUTH_PATH=$HOME_PATH/docker/.xauth_docker
XAUTH=$DAUTH_PATH/share/.docker.xauth 
if [ -f $XAUTH ]
then
  rm $XAUTH    
fi 
if [ ! -f $XAUTH ]
then
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' > $DAUTH_PATH/tempAuth.txt   	
    xauth -f $XAUTH nmerge $DAUTH_PATH/tempAuth.txt        
    rm $DAUTH_PATH/tempAuth.txt        
    chmod a+r $XAUTH
fi
