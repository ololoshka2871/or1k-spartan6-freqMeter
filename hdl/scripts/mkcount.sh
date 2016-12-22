#!/bin/bash

for ((i=0;i<${1};++i)); do printf "%08X\n" $i; done
