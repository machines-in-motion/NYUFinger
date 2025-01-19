#!/bin/bash

GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting DDS type generation...${NC}"
# Clean
# rm -r ../unitree_go
rm -r ../FingerMsgs
rm -r FingerMsgs

# Make
for file in ../FingerPy/msg_definitions/*.idl
do
    echo "Processing $file file..."
    idlc -l py $file
done
mv FingerMsgs ..
# mv go2py_messages ../