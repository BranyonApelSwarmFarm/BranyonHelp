#!/bin/bash

script_name=$(basename "$0")

function print_help {
    echo "Bundle of tools to make life easier when working with swarmbotCommon"
    echo "Usage: $script_name [clone|fetch|log|pull]"
    echo ""
    echo "Arguments"
    echo "       clone                      Clones the workspace into its usual location"
    echo "       fetch                      Runs a git fetch"
    echo "       log                        Runs a git log"
    echo "       pull                       Runs a git pull"
}

if [ $1 == "clone" ]; then
    git clone git@github.com:swarmfarm/swarmbotCommon.git /var/lib/swarmfarm/swarmbot/swarmbotCommon
else
    git --git-dir /var/lib/swarmfarm/swarmbot/swarmbotCommon/.git $@
fi