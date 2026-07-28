#!/bin/bash

# バイナリの相対パス
CURRENT=$(cd $(dirname $0);pwd)
echo $CURRENT
DIR_NAME=`echo "$CURRENT" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
echo $DIR_NAME
binary_path="$CURRENT/build/$DIR_NAME.bin"

# STM32_Programmer_CLIを使用して書き込みを行う
# STLink Flashing
STM32_Programmer_CLI -c port=SWD -w "$binary_path" 0x08000000 -s