#!/usr/local/bin/bash

# SPDX-License-Identifier: BSD-3-Clause
#
# Copyright (c) 2023 Google LLC
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

SOURCE_DIR="$(dirname "$0")/.."
BUILD_DIR="build"
INSTALL=false

while getopts i name
do
    case ${name} in
    i) INSTALL=true;;
    ?) "Supply -i to install the new driver as a boot module"
       exit 2;;
    esac
done

pushd "${SOURCE_DIR}"
trap "popd" EXIT

./build_src.sh && \
  make -C "${BUILD_DIR}" clean && \
  make -C "${BUILD_DIR}"

if [[ $? -ne 0 ]]; then
  echo "Driver build failed!"
  exit 1
fi

kldstat | grep gve
if [[ $? -eq 0 ]]; then
  echo "Unloading existing driver"
  kldunload gve || exit 1
fi

kldload "${BUILD_DIR}/gve.ko"

if [[ ${INSTALL} = true ]]; then
  echo "Installing the new driver to /boot/modules"
  cp "${BUILD_DIR}/gve.ko" "/boot/modules/gve.ko"
fi
