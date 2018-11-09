#!/bin/bash

# targets, in order:
#  Cortex-M0 and Cortex-M0+
#  Cortex-M3
#  Cortex-M4 and Cortex-M7 (no FPU)
declare -a targets=("thumbv6m-none-eabi"
                    "thumbv7m-none-eabi"
                    "thumbv7em-none-eabi")

for target in "${targets[@]}"
do
    echo "performing build and tests for target: $target"
    for fullfile in $(pwd)/src/bin/*.rs; do
        filename=$(basename -- "$fullfile")
        extension="${filename##*.}"
        filename="${filename%.*}"

        # build test using cargo
        $(cargo build --target $target --release --bin $filename)
        echo "built test $filename for target: $target"

        echo "running test: $filename on target: $target"
        # deploy test using qemu
        test_result=$(qemu-arm target/$target/release/$filename)

        # capture any failures
        if [[ $? -ne 0 ]]
        then
            echo "fail"
            err_code=1;
        fi
        echo $test_result
    done
done

# fail if any of the tests failed on any targets
if [ -n "$err_code" ]
then
    echo "overall fail"
    exit 1;
fi