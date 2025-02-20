#!/bin/bash

python markerserver/swarmserverclient.py &  # Run in the background

timeout 3

python -m Search.main shared_params.params &
# python -m Search.main shared_params.params11 &
# python -m Search.main shared_params.params12 &
# python -m Search.main shared_params.params13 &