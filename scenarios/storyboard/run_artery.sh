#!/bin/bash
OPP_RUNALL=/home/v2x/work/omnetpp-5.6.2/bin/opp_runall
OPP_RUN=/home/v2x/work/omnetpp-5.6.2/bin/opp_run_dbg
NED_FOLDERS="/home/v2x/work/SoarArtery/arteryGit/src/artery:/home/v2x/work/SoarArtery/arteryGit/src/traci:/home/v2x/work/SoarArtery/arteryGit/extern/veins/examples/veins:/home/v2x/work/SoarArtery/arteryGit/extern/veins/src/veins:/home/v2x/work/SoarArtery/arteryGit/extern/inet/src:/home/v2x/work/SoarArtery/arteryGit/extern/inet/examples:/home/v2x/work/SoarArtery/arteryGit/extern/inet/tutorials:/home/v2x/work/SoarArtery/arteryGit/extern/inet/showcases"
LIBRARIES="-l/home/v2x/work/SoarArtery/arteryGit/build/src/artery/libartery_core.so -l/home/v2x/work/SoarArtery/arteryGit/build/src/traci/libtraci.so -l/home/v2x/work/SoarArtery/arteryGit/build/extern/libveins.so -l/home/v2x/work/SoarArtery/arteryGit/build/extern/libINET.so -l/home/v2x/work/SoarArtery/arteryGit/build/src/artery/storyboard/libartery_storyboard.so -l/home/v2x/work/SoarArtery/arteryGit/build/src/artery/envmod/libartery_envmod.so"

RUNALL=false
for arg do
    shift
    [[ "$arg" == -j* ]] && RUNALL=true && J=$arg && continue
    [[ "$arg" == -b* ]] && RUNALL=true && B=$arg && continue
    # run opp_runall with default values for -j* and -b* options by just specifying '--all'
    [[ "$arg" == "--all" ]] && RUNALL=true && continue
    set -- "$@" "$arg"
done

if [ "$RUNALL" = true ] ; then
    $OPP_RUNALL $J $B $OPP_RUN -n $NED_FOLDERS $LIBRARIES $@
else
    $OPP_RUN -n $NED_FOLDERS $LIBRARIES $@
fi
