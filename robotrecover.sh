#!/bin/bash
set -e

elems=(juliet kilo lima mike november oscar papa quebec romeo sierra tango uniform victor whiskey xray yankee zulu alpha2 beta2 charlie2)

for elem in "${elems[@]}"; do
    # echo "$elem"
   curl 'https://authentication-db.swarmfarm.com/api/set-permissions' \
       -H 'Pragma: no-cache' \
       -H 'Cache-Control: no-cache' \
       -H 'Content-Type: application/json' \
       -H 'Accept: */*' \
       --data-raw "{\"username\":\"branyon.apel\",\"robot\":\"$elem\",\"isAllowed\":true}"
done
