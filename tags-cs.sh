#!/bin/bash
# Generate cscope files

echo > cscope.files

find `pwd` -type f                          \
    -iname '*.[ch]'                         \
    -o -iname '*.hpp'  -o -iname '*.cpp'    \
    -o -iname '[Mm]akefile' -o -name '*.mk' \
    -o -iname '*.sh'                        \
    -o -iname '*.s'                         \
    -o -iname '*.xml'                       \
    >> cscope.files

cscope -qkb -i cscope.files

