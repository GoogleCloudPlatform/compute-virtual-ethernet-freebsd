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

NUM_FLOWS=1
TMPFILE="/tmp/a"
DURATION=60
SERVER_IP="192.168.100.1"
DATAGRAM_SIZE="1408"

while getopts f:l:s:m: name
do
    case ${name} in
    s)   SERVER_IP="$OPTARG";;
    f)   NUM_FLOWS="$OPTARG";;
    l)   DURATION="$OPTARG";;
    m)   DATAGRAM_SIZE="$OPTARG";;
    ?)   printf "Usage: %s: [-s <server ip>] [-f <num flows>] [-l <test len seconds>] [-m <datagram size>]\n" $0
          exit 2;;
    esac
done

pkill netperf
: > ${TMPFILE}

echo "$(date): Launching ${NUM_FLOWS} netperf client processes..."
for i in $(seq 1 ${NUM_FLOWS}); do
        echo "$(netperf -P 0 -H ${SERVER_IP} -l ${DURATION} -t UDP_STREAM -- -R 1 -m ${DATAGRAM_SIZE} -O local_send_throughput,remote_recv_throughput 2>&1) $(date)" | tee -a "${TMPFILE}" &
done
echo "$(date): ... done"

echo "$(date): Starting wait for netperfs"
wait
echo "$(date): Netperfs concluded"

echo "Reporting throughput for $(cat ${TMPFILE} | wc -l) flows"
echo "Total Local Send throughput is: $(awk '{sum += $1}END{print sum}' ${TMPFILE})"
echo "Total Remote Recv throughput is: $(awk '{sum += $2}END{print sum}' ${TMPFILE})"
