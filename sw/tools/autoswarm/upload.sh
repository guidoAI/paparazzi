#!/bin/bash

for var in "$@"
do
if (make --no-print-directory -C $PAPARAZZI_SRC -f Makefile.ac AIRCRAFT=bebop2$var ap.upload 2>/dev/null | grep "Upload and Start of ap.elf to Bebop succesful")
then
echo "SUCCES: bebop 2$var"
else
if (make --no-print-directory -C $PAPARAZZI_SRC -f Makefile.ac AIRCRAFT=bebop2$var ap.upload 2>/dev/null | grep "Upload and Start of ap.elf to Bebop succesful")
then
echo "SUCCES: bebop 2$var"
else
if (make --no-print-directory -C $PAPARAZZI_SRC -f Makefile.ac AIRCRAFT=bebop2$var ap.upload 2>/dev/null | grep "Upload and Start of ap.elf to Bebop succesful")
then
echo "SUCCES: bebop 2$var"
else
echo "FAIL:   bebop 2$var"
fi
fi
fi
done
