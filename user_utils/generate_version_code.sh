#!/bin/bash

# Get the latest Git tag
version_code=$(git describe --tags)

# Output the version code
echo "Generated version code: $version_code"
echo -e "/* This file is generated automatically, do not edit */\n#define VERSION \"$version_code\"" > ./main/version.h
