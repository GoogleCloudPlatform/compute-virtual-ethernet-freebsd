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
TMPFILE_PREFIX="/tmp/iperf_out"
DURATION=60
SERVER_IP="192.168.100.1"
DATAGRAM_SIZE="1470"
SERVER=false
START_PORT="10271"
TOTAL_BW_MBPS="100000"
V6_ADDENDUM=""

while getopts f:l:s:m:S6 name
do
    case ${name} in
    s)   SERVER_IP="$OPTARG";;
    f)   NUM_FLOWS="$OPTARG";;
    l)   DURATION="$OPTARG";;
    m)   DATAGRAM_SIZE="$OPTARG";;
    S)   SERVER=true;;
    6)   V6_ADDENDUM="--ipv6_domain";;
    ?)   printf "Usage: %s: [-S] [-s <server ip>] [-f <num flows>] [-l <test len seconds>] [-m <datagram size>]\n" $0
         printf "    -s <server-ip> is ignored if -S is supplied"
         exit 2;;
    esac
done

pkill iperf
rm ${TMPFILE_PREFIX}*

if [[ ${SERVER} = true ]]; then
  echo "$(date): Launching ${NUM_FLOWS} iperf server processes..."
  for i in $(seq 1 ${NUM_FLOWS}); do
    # Let the server run for an extra 30s to give the operator time to launch clients
    echo "$(iperf -s -u -p $((START_PORT+i-1)) -t $((DURATION+30)) -l ${DATAGRAM_SIZE} ${V6_ADDENDUM} 2>&1) $(date)" | tee -a "${TMPFILE_PREFIX}_${i}" &
  done
  echo "$(date): ... done"

  echo "$(date): Starting wait for iperf servers -- go and launch the clients, quick!"
  wait
  echo "$(date): iperfs concluded"
  exit 0
fi

echo "$(date): Launching ${NUM_FLOWS} iperf client processes..."
for i in $(seq 1 ${NUM_FLOWS}); do
  echo "$(iperf -c ${SERVER_IP} -u -p $((START_PORT+i-1)) -t ${DURATION} -l ${DATAGRAM_SIZE} ${V6_ADDENDUM} -b $((TOTAL_BW_MBPS/NUM_FLOWS))M -fm 2>&1) $(date)" | tee -a "${TMPFILE_PREFIX}_${i}" &
done
echo "$(date): ... done"

echo "$(date): Starting wait for iperf clients"
wait
echo "$(date): iperfs concluded"

total_snd_xput=0
total_rcv_xput=0
snd_xput=0
rcv_xput=0

for i in $(seq 1 ${NUM_FLOWS}); do
  # Extracts the 52.4 from this line: [  1] 0.00-10.00 sec  62.5 MBytes  52.4 Gbits/sec
  snd_xput="$(sed -nr 's/\[[[:space:]]+1\][[:space:]]+[0-9.]+-[0-9.]+[[:space:]]+sec[[:space:]]+[0-9.]+[[:space:]]+[M]Bytes[[:space:]]+([0-9.]+)[[:space:]]+[M]bits\/sec$/\1/p' ${TMPFILE_PREFIX}_${i} | awk -F. '{print $1}')"

  # The UDP client program also prints a "Server Report" from which we extract
  # the receiver stats. For example, we extract the 52.99 from this line:
  # [  1] 0.00-10.00 sec  62.5 MBytes  52.99 Gbits/sec   0.001 ms 0/44586 (0%)
  rcv_xput="$(sed -nr 's/\[[[:space:]]+1\][[:space:]]+[0-9.]+-[0-9.]+[[:space:]]+sec[[:space:]]+[0-9.]+[[:space:]]+[M]Bytes[[:space:]]+([0-9.]+)[[:space:]]+[M]bits\/sec[[:space:]]+[0-9.]+[[:space:]]ms.*$/\1/p' ${TMPFILE_PREFIX}_${i} | awk -F. '{print $1}')"

  total_rcv_xput=$((total_rcv_xput + rcv_xput))
  total_snd_xput=$((total_snd_xput + snd_xput))
done

echo "Total receive throughput in Mbps is ${total_rcv_xput}"
echo "Total send throughput in Mbps is ${total_snd_xput}"
