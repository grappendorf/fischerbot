#!/bin/bash
PROJECT_DIR=fischerbot
VERSION=$(cat version.txt)
mkdir -p target
cd ..
zip ${PROJECT_DIR}/target/fischerbot-${VERSION}.zip ${PROJECT_DIR} -r \
    --exclude ${PROJECT_DIR}/build/\* \
    --exclude ${PROJECT_DIR}/target/\* \
    --exclude ${PROJECT_DIR}/.svn/\* \
    --exclude ${PROJECT_DIR}/.settings/\* \
    --exclude ${PROJECT_DIR}/\*/eagle.epf
cd ${PROJECT_DIR}
