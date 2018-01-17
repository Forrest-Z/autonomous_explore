#!/usr/bin/env bash
find . -iname '*.hpp' -o -iname '*.cpp' -o -iname '*.cc' | xargs clang-format-3.8 -i

