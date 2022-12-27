# [PATHS]
export HOMEFOLDER=$HOME
export MAINFOLDER="$HOMEFOLDER/geth-argos"
export DOCKERFOLDER="$MAINFOLDER/argos-blockchain"
export ARGOSFOLDER="$MAINFOLDER/argos-python"

# export EXPERIMENTFOLDER="$MAINFOLDER/thesis" gets the directory name in which is stored the experimetconfig file
path=`readlink -f "${BASH_SOURCE:-$0}"`
export EXPERIMENTFOLDER=`dirname ${path}`
export BLOCKCHAINPATH="$HOMEFOLDER/eth_data_para/data"

# [FILES]
export GENESISNAME="genesis_poa"
export GENESISFILE="${DOCKERFOLDER}/geth/files/$GENESISNAME.json"

export CONTRACTNAME="MarketForaging"
export CONTRACTADDRESS="${EXPERIMENTFOLDER}/scs/contractAddress.txt"
export CONTRACTABI="${EXPERIMENTFOLDER}/scs/build/$CONTRACTNAME.abi"
export CONTRACTBIN="${EXPERIMENTFOLDER}/scs/build/$CONTRACTNAME.bin-runtime"

export SCNAME="resource_market_esoft"
export SCFILE="${EXPERIMENTFOLDER}/scs/${SCNAME}.sol" 
export SCTEMPLATE="${EXPERIMENTFOLDER}/scs/${SCNAME}.x.sol" 

export ARGOSFILE="${EXPERIMENTFOLDER}/experiments/market-foraging.argos"
export ARGOSTEMPLATE="${EXPERIMENTFOLDER}/experiments/market-foraging.x.argos"

# [DOCKER]
export SWARMNAME=ethereum
export CONTAINERBASE=${SWARMNAME}_eth

# [ARGOS]
export NUM1=10
export CON1="${EXPERIMENTFOLDER}/controllers/main_collab.py"

export NUM2=0
export CON2="${EXPERIMENTFOLDER}/controllers/main_greedy.py"

export NUMBOX=5
export NUMCYL=5

export RABRANGE="2"
export WHEELNOISE="0"
export TPS=10
export DENSITY="3"

export NUMROBOTS=$(echo $NUM1+$NUM2 | bc)
export ARENADIM=5
export ARENADIMH=2
# export ARENADIM=$(echo "scale=3 ; sqrt($NUMROBOTS/$DENSITY)" | bc)
# export ARENADIMH=$(echo "scale=3 ; $ARENADIM/2" | bc)
export STARTDIM=$(echo "scale=3 ; $ARENADIM*2/5" | bc)


# [GETH]
export BLOCKPERIOD=2

# [SC]
export MAXWORKERS=10
export EPSILON=15
export WINSIZE=5

# [OTHER]
export SEED=350
export TIMELIMIT=15
export SLEEPTIME=5
export REPS=5




