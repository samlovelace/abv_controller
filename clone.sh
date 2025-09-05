#!/bin/bash

sudo apt install python3-pip -y
pip install vcstool 
cd ..
vcs import < abv_controller/repos.yaml
