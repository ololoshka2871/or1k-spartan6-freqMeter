#!/bin/bash

CMD=`basename $0`

export LANG=C

genrandom_color() {
    echo -n `python -c "import random; print('{:3f} {:3f} {:3f}'.format(random.random(), random.random(), random.random()))"`
}

show_help()
{
    echo "Usage: $CMD <BINARY> <TOOLPREFIX> <JUMP_INST> [DEBUG={0*/1}] | dot -Tsvg -ocallgraph.svg"
}

if [ $# -ne 3 -a $# -ne 4 ]; then
    echo "Fail! -- Expecting 3 or 4 arguments! ==> $@"
    show_help
    exit 1
fi

EXEC=$1

TOOLPREFIX=$2
READELF=${TOOLPREFIX}readelf
OBJDUMP=${TOOLPREFIX}objdump
CPPFILT=${TOOLPREFIX}c++filt
JUMP_INST=$3

DEBUG=$4

if [ ! -f "$EXEC" ]; then
    echo "Error: $EXEC doesn't exist!"
    exit 1
fi

if [ -z "$JUMP_INST" ]; then
    echo "Warning: $JUMP_INST doesn't exist, assuming \"callq\""
    JUMP_INST="callq"
fi

escaped_JMP_INST=`echo $JUMP_INST | sed 's/\./\\\\./g'`
call_parcesr_sed_script="s/$escaped_JMP_INST.* \([0-9a-f]*\) .*/\1/"

if [ -z "$DEBUG" ]; then
    DEBUG=0
fi

if [ -z "`which ${READELF}`" ]; then
    echo "Error: Requires \"${READELF}\""
    exit 1
fi

if [ -z "`which ${OBJDUMP}`" ]; then
    echo "Error: Requires \"${OBJDUMP}\""
    exit 1
fi

if [ -z "`which ${CPPFILT}`" ]; then
    echo "Error: Requires \"${CPPFILT}\""
    exit 1
fi

if [ -z "`which dot`" ]; then
    echo "Error: Requires \"dot\""
    exit 1
fi

#${READELF} $EXEC --all
GEN_SYM_FILE_CMD="${READELF} $EXEC --headers --symbols"

#http://stackoverflow.com/questions/1737095/how-do-i-disassemble-raw-x86-code
#http://stackoverflow.com/questions/19071461/disassemble-raw-x64-machine-code

#${OBJDUMP} -D -b binary -mi386 -Maddr16,data16 $EXEC
#GEN_ASM_FILE_CMD="${OBJDUMP} -D -b binary -m$MASHINE $EXEC"
GEN_ASM_FILE_CMD="${OBJDUMP} -D $EXEC"

if [ "$DEBUG" == 1 ]; then
    echo "${READELF} command: $GEN_SYM_FILE_CMD" 1>&2
    echo "${OBJDUMP} command: $GEN_ASM_FILE_CMD" 1>&2
    echo "" 1>&2
fi

SYM_FILE_CONTENTS="`$GEN_SYM_FILE_CMD`"
ASM_FILE_CONTENTS="`$GEN_ASM_FILE_CMD`"

if [ "$DEBUG" == 1 ]; then
    DEBUG_SYM_FILE="`mktemp`"
    DEBUG_ASM_FILE="`mktemp`"
    #trap "rm $DEBUG_SYM_FILE $DEBUG_ASM_FILE" EXIT
    echo "$SYM_FILE_CONTENTS" > $DEBUG_SYM_FILE
    echo "$ASM_FILE_CONTENTS" > $DEBUG_ASM_FILE
    echo "Cached ${READELF} output: $DEBUG_SYM_FILE" 1>&2
    echo "Cached ${OBJDUMP} output: $DEBUG_ASM_FILE" 1>&2
    echo "" 1>&2
fi

ENTRY_POINT_LINE="`echo \"$SYM_FILE_CONTENTS\" | grep \"Entry point address:\"`"
ENTRY_POINT_ADDR="`echo \"$ENTRY_POINT_LINE\" | cut -d':' -f2 | tr -d ' ' | sed 's/^0x4[0]*//g'`"

echo "Generating function address pairs.. (Step 1 of 3)" 1>&2
FUNC_TRIPLE_LIST=""
FOUND_SYMTAB=0
n="`echo \"$SYM_FILE_CONTENTS\" | wc -l`"
i=1
while read SYM_FILE_LINE; do
    PROGRESS=$(( $i * 100 / $n ))
    if (( $n > 100 )); then
        printf "\r$PROGRESS%%" 1>&2
    fi

    if [ "$FOUND_SYMTAB" == 0 ]; then
        if [[ "$SYM_FILE_LINE" =~ "Symbol table '.symtab'" ]]; then
            FOUND_SYMTAB=1
        else
            continue
        fi
    fi
    SYM_TUPLE="`echo \"$SYM_FILE_LINE\" | sed 's/[ ]\+/ /g'`"
    if [ "`echo \"$SYM_TUPLE\" | cut -d' ' -f4`" == "FUNC" ] &&
       [ "`echo \"$SYM_TUPLE\" | cut -d' ' -f5`" == "GLOBAL" ] &&
       [ "`echo \"$SYM_TUPLE\" | cut -d' ' -f7`" != "UND" ];
    then
        FUNC_PAIR="`echo \"$SYM_TUPLE\" | cut -d' ' -f2,8 | sed 's/^00000000004[0]*//g'`"
        FUNC_ADDR="`echo \"$FUNC_PAIR\" | cut -d' ' -f1`"
        FUNC_ADDR_DEC="`printf \"%d\" 0x$FUNC_ADDR`"
        FUNC_TRIPLE="$FUNC_ADDR_DEC $FUNC_PAIR"
        FUNC_TRIPLE_LIST="$FUNC_TRIPLE_LIST\n$FUNC_TRIPLE"
    fi

    i=$(( $i + 1 ))
done <<< "$SYM_FILE_CONTENTS"
if (( $n > 100 )); then
    echo -e "\r100%" 1>&2
fi
if [ "$FOUND_SYMTAB" == 0 ]; then
    echo "Error: Can't find symtab section in \"$EXEC\"."
    exit
fi
SORTED_FUNC_PAIR_LIST="`echo -e \"$FUNC_TRIPLE_LIST\" | sort -g | grep -v '^$' | cut -d' ' -f2,3`"

if [ "$DEBUG" == 1 ]; then
    DEBUG_FUNC_PAIR_FILE="`mktemp`"
    echo "$SORTED_FUNC_PAIR_LIST" > $DEBUG_FUNC_PAIR_FILE
    echo "Generated function address pairs: $DEBUG_FUNC_PAIR_FILE" 1>&2
    echo "" 1>&2
fi

echo "digraph \"`basename $EXEC`\" {"
echo "rankdir=LR;"
echo "node [shape=ellipse];"

echo "Generating nodes.. (Step 2 of 3)" 1>&2
n="`echo \"$SORTED_FUNC_PAIR_LIST\" | wc -l`"
i=1
while read -r FUNC_PAIR; do
    PROGRESS=$(( $i * 100 / $n ))
    if (( $n > 100 )); then
        printf "\r$PROGRESS%%" 1>&2
    fi

    FUNC_ADDR="`echo \"$FUNC_PAIR\" | cut -d' ' -f1`"
    FUNC_NAME="`echo \"$FUNC_PAIR\" | cut -d' ' -f2`"
    FUNC_NAME_DEMANGLED="`echo $FUNC_NAME | ${CPPFILT}`"
    if [ "$FUNC_ADDR" == "$ENTRY_POINT_ADDR" ]; then
        SHAPE_SPEC_STR=", shape=\"box\""
    else
        SHAPE_SPEC_STR=""
    fi
    echo "$FUNC_NAME [label=\"0x$FUNC_ADDR: $FUNC_NAME_DEMANGLED\"$SHAPE_SPEC_STR];"

    i=$(( $i + 1 ))
done <<< "$SORTED_FUNC_PAIR_LIST"
if (( $n > 100 )); then
    echo -e "\r100%" 1>&2
fi

echo "Generating edges.. (Step 3 of 3)" 1>&2
n="`echo \"$SORTED_FUNC_PAIR_LIST\" | wc -l`"
i=1
while read -r FUNC_PAIR; do
    PROGRESS=$(( $i * 100 / $n ))
    if (( $n > 100 )); then
        printf "\r$PROGRESS%%" 1>&2
    fi

    FUNC_ADDR="`echo \"$FUNC_PAIR\" | cut -d' ' -f1`"
    FUNC_NAME="`echo \"$FUNC_PAIR\" | cut -d' ' -f2`"

    FUNC_ASM_LINE_NO="`echo \"$ASM_FILE_CONTENTS\" | grep -n \"^[ ]*$FUNC_ADDR:\" | head -1 | cut -d':' -f1`"
    if [ -z "$FUNC_ASM_LINE_NO" ]; then
        i=$(( $i + 1 ))
        continue
    fi

    NEXT_FUNC_INDEX=$(( $i + 1 ))
    NEXT_FUNC_PAIR="`echo \"$SORTED_FUNC_PAIR_LIST\" | head -$NEXT_FUNC_INDEX | tail -1`"

    NEXT_FUNC_ADDR="`echo \"$NEXT_FUNC_PAIR\" | cut -d' ' -f1`"
    if [ -z "$NEXT_FUNC_ADDR" ]; then
        i=$(( $i + 1 ))
        continue
    fi
    NEXT_FUNC_NAME="`echo \"$NEXT_FUNC_PAIR\" | cut -d' ' -f2`"

    NEXT_FUNC_ASM_LINE_NO="`echo \"$ASM_FILE_CONTENTS\" | grep -n \"^[ ]*$NEXT_FUNC_ADDR:\" | head -1 | cut -d':' -f1`"
    FUNC_ASM_LAST_LINE_NO=$(( $NEXT_FUNC_ASM_LINE_NO - 1 ))
    FUNC_ASM_BODY_LEN=$(( $NEXT_FUNC_ASM_LINE_NO - $FUNC_ASM_LINE_NO ))
    FUNC_ASM_BODY="`echo \"$ASM_FILE_CONTENTS\" | head -$FUNC_ASM_LAST_LINE_NO | tail -$FUNC_ASM_BODY_LEN`"
    CALLEE_ASM_LINES_LIST="`echo \"$FUNC_ASM_BODY\" | grep $JUMP_INST`"
    if [ -z "$CALLEE_ASM_LINES_LIST" ]; then
        i=$(( $i + 1 ))
        continue
    fi

    echo $CALLEE_ASM_LINES_LIST 1>&2
    read ttt

    while read -r CALLEE_ASM_LINE; do
        CALLEE_ADDR_PART="`echo \"$CALLEE_ASM_LINE\" | cut -d'	' -f1`"
        CALL_ADDR="`echo \"$CALLEE_ADDR_PART\" | cut -d':' -f1`"
        CALLEE_CMD="`echo \"$CALLEE_ASM_LINE\" | cut -d'	' -f3`"
        CALLEE_ADDR="`echo \"$CALLEE_CMD\" | sed \"$call_parcesr_sed_script\"`"
        CALLEE_NAME="`echo \"$SORTED_FUNC_PAIR_LIST\" | grep \"$CALLEE_ADDR\" | cut -d' ' -f2`"
        if [ -z "$CALLEE_NAME" ]; then
            continue
        fi
        echo "$FUNC_NAME -> $CALLEE_NAME [label=\"0x$CALL_ADDR\" color=\"`genrandom_color`\"]"
    done <<< "$CALLEE_ASM_LINES_LIST"

    i=$(( $i + 1 ))
done <<< "$SORTED_FUNC_PAIR_LIST"
if (( $n > 100 )); then
    echo -e "\r100%" 1>&2
fi

echo "}"

echo "Done!" 1>&2

