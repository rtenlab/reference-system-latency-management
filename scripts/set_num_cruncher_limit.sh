#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <limit> <num_trials>"
    exit 1
fi

echo "Setting number cruncher limit to $1"
SOURCE=${BASH_SOURCE[0]}
while [ -L "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  TARGET=$(readlink "$SOURCE")
  if [[ $TARGET == /* ]]; then
    #echo "SOURCE '$SOURCE' is an absolute symlink to '$TARGET'"
    SOURCE=$TARGET
  else
    DIR=$( dirname "$SOURCE" )
    #echo "SOURCE '$SOURCE' is a relative symlink to '$TARGET' (relative to '$DIR')"
    SOURCE=$DIR/$TARGET # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
  fi
done
#echo "SOURCE is '$SOURCE'"
RDIR=$( dirname "$SOURCE" )
DIR=$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )
# if [ "$DIR" != "$RDIR" ]; then
#   echo "DIR '$RDIR' resolves to '$DIR'"
# fi
# echo "DIR is '$DIR'"

cd $DIR
sed -i "s/^ *static constexpr uint64_t DEFAULT_NUMBER_CRUNCHER_LIMIT = [0-9]*;/  static constexpr uint64_t DEFAULT_NUMBER_CRUNCHER_LIMIT = $1;/" ../autoware_reference_system/include/autoware_reference_system/system/timing/default.hpp
echo "Executing number cruncher benchmark with limit: $1"
#source ~/ros2_humble/install/setup.bash
#source ../install/setup.bash

#cd ..
#colcon build --symlink-install --cmake-args -DPICAS=FALSE --packages-select autoware_reference_system reference_system
../build/autoware_reference_system/number_cruncher_benchmark $1 $2

#cat ../autoware_reference_system/include/autoware_reference_system/system/timing/default.hpp
