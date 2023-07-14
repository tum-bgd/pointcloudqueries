#!/bin/bash

while true; do
    time python3 kijkduin.py
    wc -l < benchmark.json
done
