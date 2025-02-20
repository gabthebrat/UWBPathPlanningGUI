#!/bin/bash

python markerserver/swarmserverclient.py &  # Run in the background

timeout 3

python -m UnknownArea_v2.main shared_params.params &
# python -m UnknownArea_v2.main shared_params.params11 &
# python -m UnknownArea_v2.main shared_params.params12 &
# python -m UnknownArea_v2.main shared_params.params13 &

