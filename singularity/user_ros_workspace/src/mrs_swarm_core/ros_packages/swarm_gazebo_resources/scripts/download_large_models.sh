#!/bin/bash

MY_PATH=`dirname "$0"`
MY_PATH=`( cd ../"$MY_PATH" && pwd )`

FOREST_MODEL=forest_rumburk_large.zip

[ ! -d $MY_PATH/models ] && mkdir $MY_PATH/models
cd $MY_PATH/models

echo "Downloading models from NAS"
wget --no-check-certificate -O $FOREST_MODEL https://nasmrs.felk.cvut.cz/index.php/s/MgqizVHlAjr5OCn/download

echo "Unzipping models"
unzip $FOREST_MODEL

echo "Cleaning up"
rm $FOREST_MODEL

echo "Done"
