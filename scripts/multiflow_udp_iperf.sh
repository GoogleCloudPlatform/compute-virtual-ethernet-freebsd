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
DATAGRAM_SIZE="1430"
SERVER=false
START_PORT="10271"
NC_PORT="6688"
TOTAL_BW_MBPS="100000"
V6_ADDENDUM=""
COOLDOWN_SECS="5"
BIDI_ADDENDUM=""
IPERF_BIN="iperf"
SILENCE_ADDENDUM=""

while getopts f:l:s:m:S6d3 name
do
    case ${name} in
    s)   SERVER_IP="$OPTARG";;
    f)   NUM_FLOWS="$OPTARG";;
    l)   DURATION="$OPTARG";;
    m)   DATAGRAM_SIZE="$OPTARG";;
    S)   SERVER=true;;
    6)   V6_ADDENDUM="--ipv6_domain";;
    d)   BIDI_ADDENDUM=" -d ";;
    3)   IPERF_BIN="iperf3";;
    ?)   printf "Usage: %s: [-S] [-d] [-3] [-s <server ip>] [-f <num flows>] [-l <test len seconds>] [-m <datagram size>]\n" $0
         printf "    Supply -S to run in server mode\n"
         printf "    Supply -d to the client to run bidirectional flows\n"
         printf "    Supply -3 to use iperf3 instead of iperf2\n"
         printf "    -s, -l, and -d are ignored if -S is supplied\n"
         exit 2;;
    esac
done

# iperf2 requires a v6 flag for both server and client
V6_ADDENDUM_SERVER="${V6_ADDENDUM}"
V6_ADDENDUM_CLIENT="${V6_ADDENDUM}"

# iperf2's server requires some flags that are client-only in iperf3
UDP_ADDENDUM_SERVER="-u -l ${DATAGRAM_SIZE}"

# Some translations when using iperf3
if [[ "${IPERF_BIN}" == "iperf3" ]]; then
  if [[ "${BIDI_ADDENDUM}" != "" ]]; then
    BIDI_ADDENDUM=" --bidir  "
  fi
  if [[ "${V6_ADDENDUM}" != "" ]]; then
    V6_ADDENDUM_SERVER=""
    V6_ADDENDUM_CLIENT="--version6"
  fi
  UDP_ADDENDUM_SERVER=""
  SILENCE_ADDENDUM="-i ${DURATION}"
fi

pkill "${IPERF_BIN}"
rm ${TMPFILE_PREFIX}*
trap "pkill ${IPERF_BIN}" EXIT

function launch_servers() {
  # kills previous iperf servers
  pkill "${IPERF_BIN}"

  if uname | grep -iq linux; then
	  older_self="$(netstat -tulpn | grep ":${NC_PORT}" | awk '{print $7}' | cut -d/ -f1)"
  else
	  older_self="$(sockstat -l | grep ":${NC_PORT}" | awk '{print $3}')"
  fi

  if [[ "${older_self}" != "" ]]; then
    echo "$(date): killing existing server script ${older_self}"
    kill -9 "${older_self}"
  fi

  echo "$(date): Sleeping for ${COOLDOWN_SECS}s for the ports to become free"
  sleep ${COOLDOWN_SECS}

  for i in $(seq 1 ${NUM_FLOWS}); do
    echo "$(${IPERF_BIN} -s ${UDP_ADDENDUM_SERVER} -p $((START_PORT+i-1)) ${V6_ADDENDUM_SERVER} 2>&1) $(date)" > "${TMPFILE_PREFIX}_${i}" &
  done

  ip_addr="$(ifconfig | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}')"
  echo "$(date): Launched ${NUM_FLOWS} ${IPERF_BIN} server processes, can point client at ${ip_addr}"

  # Client asks the server to relaunch its iperf processes after a run.
  nc_msg=$(nc -l ${NC_PORT}) # nc blocks till client writes "relaunch"
  case ${nc_msg} in
    "relaunch") echo "$(date): Relaunching servers"  && launch_servers;;
    *) echo "Unknown nc cmd from client: ${nc_msg}" && exit 1;;
  esac
}

if [[ ${SERVER} = true ]]; then
  launch_servers # Runs recursively forever till EXIT
  echo "$(date): Unexpected end of server loop" && exit 1
fi

for i in $(seq 1 ${NUM_FLOWS}); do
  ${IPERF_BIN} -c ${SERVER_IP} -u -p $((START_PORT+i-1)) -t ${DURATION} -l ${DATAGRAM_SIZE} ${BIDI_ADDENDUM} ${V6_ADDENDUM_CLIENT} ${SILENCE_ADDENDUM} -b $((TOTAL_BW_MBPS/NUM_FLOWS))M -fm 2>&1 > "${TMPFILE_PREFIX}_${i}" &
done
echo "$(date): Launched ${NUM_FLOWS} ${IPERF_BIN} client processes, waiting on them"

wait
echo "$(date): ${IPERF_BIN}s concluded"

total_local_snd_xput=0
total_remote_rcv_xput=0
total_local_rcv_xput=0 # bidi only

local_snd_xput=0
remote_rcv_xput=0
local_rcv_xput=0 # bidi only

for i in $(seq 1 ${NUM_FLOWS}); do
  if [[ "${IPERF_BIN}" == "iperf" ]]; then
    # Extracts the 52.4 from this line: [  1] 0.00-10.00 sec  62.5 MBytes  52.4 Gbits/sec
    local_snd_xput="$(sed -nr 's/\[[[:space:]]+1\][[:space:]]+[0-9.]+-[0-9.]+[[:space:]]+sec[[:space:]]+[0-9.]+[[:space:]]+[M]Bytes[[:space:]]+([0-9.]+)[[:space:]]+[M]bits\/sec$/\1/p' ${TMPFILE_PREFIX}_${i} | awk -F. '{print $1}')"
  else
    # Extracts the 10000 from these kind of lines:
    # [  5][TX-C]   0.00-10.00  sec  11.6 GBytes  10000 Mbits/sec  0.000 ms  0/1411468 (0%)  sender
    # [  5]   0.00-10.00  sec  11.6 GBytes  10000 Mbits/sec  0.000 ms  0/1411468 (0%)  sender
    local_snd_xput="$(sed -nr 's/\[[[:space:]]+5\](\[TX-C\])?[[:space:]]+[0-9.]+-[0-9.]+[[:space:]]+sec[[:space:]]+[0-9.]+[[:space:]]+[GM]Bytes[[:space:]]+([0-9.]+)[[:space:]]+[M]bits\/sec[[:space:]]+[0-9.]+[[:space:]]ms.*sender$/\2/p' ${TMPFILE_PREFIX}_${i} | awk -F. '{print $1}')"
  fi

  if [[ "${IPERF_BIN}" == "iperf" ]]; then
    # The UDP client program also prints a "Server Report" from which we extract
    # the receiver stats. For example, we extract the 52.99 from this line:
    # [  1] 0.00-10.00 sec  62.5 MBytes  52.99 Gbits/sec   0.001 ms 0/44586 (0%)
    remote_rcv_xput="$(sed -nr 's/\[[[:space:]]+1\][[:space:]]+[0-9.]+-[0-9.]+[[:space:]]+sec[[:space:]]+[0-9.]+[[:space:]]+[M]Bytes[[:space:]]+([0-9.]+)[[:space:]]+[M]bits\/sec[[:space:]]+[0-9.]+[[:space:]]ms.*$/\1/p' ${TMPFILE_PREFIX}_${i} | awk -F. '{print $1}')"
  else
    # Extracts the 7173 from these kind of lines:
    # [  5][TX-C]   0.00-10.00  sec  8.35 GBytes  7173 Mbits/sec  0.004 ms  398951/1411468 (28%)  receiver
    # [  5]   0.00-10.00  sec  8.35 GBytes  7173 Mbits/sec  0.004 ms  398951/1411468 (28%)  receiver
    remote_rcv_xput="$(sed -nr 's/\[[[:space:]]+5\](\[TX-C\])?[[:space:]]+[0-9.]+-[0-9.]+[[:space:]]+sec[[:space:]]+[0-9.]+[[:space:]]+[GM]Bytes[[:space:]]+([0-9.]+)[[:space:]]+[M]bits\/sec[[:space:]]+[0-9.]+[[:space:]]ms.*receiver$/\2/p' ${TMPFILE_PREFIX}_${i} | awk -F. '{print $1}')"
  fi

  if [[ "${remote_rcv_xput}" -eq "0" ]]; then
    echo "No remote receive report for flow ${i}, ${IPERF_BIN} output:"
    cat ${TMPFILE_PREFIX}_${i}
    exit 1
  fi

  if [[ "${BIDI_ADDENDUM}" != "" ]]; then
    if [[ "${IPERF_BIN}" == "iperf" ]]; then
      # The UDP client program also prints the local receive stats when running
      # in bidirectional mode. For example, we extract the 52.99 from this line:
      # [  2] 0.00-10.00 sec  62.5 MBytes  52.99 Gbits/sec   0.001 ms 0/44586 (0%)
      local_rcv_xput="$(sed -nr 's/\[[[:space:]]+2\][[:space:]]+[0-9.]+-[0-9.]+[[:space:]]+sec[[:space:]]+[0-9.]+[[:space:]]+[M]Bytes[[:space:]]+([0-9.]+)[[:space:]]+[M]bits\/sec[[:space:]]+[0-9.]+[[:space:]]ms.*$/\1/p' ${TMPFILE_PREFIX}_${i} | awk -F. '{print $1}')"
    else
      # Extracts the 7361 from this line:
      # [  7][RX-C]   0.00-10.00  sec  8.57 GBytes  7361 Mbits/sec  0.003 ms  372386/1411445 (26%)  receiver
      local_rcv_xput="$(sed -nr 's/\[[[:space:]]+7\]\[RX-C\][[:space:]]+[0-9.]+-[0-9.]+[[:space:]]+sec[[:space:]]+[0-9.]+[[:space:]]+[GM]Bytes[[:space:]]+([0-9.]+)[[:space:]]+[M]bits\/sec[[:space:]]+[0-9.]+[[:space:]]ms.*receiver$/\1/p' ${TMPFILE_PREFIX}_${i} | awk -F. '{print $1}')"
    fi

    if [[ "${local_rcv_xput}" -eq "0" ]]; then
      echo "No local receive report for flow ${i}, ${IPERF_BIN} output:"
      cat ${TMPFILE_PREFIX}_${i}
      exit 1
    fi
  fi

  total_remote_rcv_xput=$((total_remote_rcv_xput + remote_rcv_xput))
  total_local_snd_xput=$((total_local_snd_xput + local_snd_xput))
  total_local_rcv_xput=$((total_local_rcv_xput + local_rcv_xput))
done

if [[ "${BIDI_ADDENDUM}" != "" ]]; then
  echo "Total local send throughput in Mbps is ${total_local_snd_xput}"
  echo "Total remote receive throughput in Mbps is ${total_remote_rcv_xput}"
  echo "Total local receive throughput in Mbps is ${total_local_rcv_xput}"
  echo "Total receive throughput in Mbps is $((total_remote_rcv_xput + total_local_rcv_xput))"
else
  echo "Total send throughput in Mbps is ${total_local_snd_xput}"
  echo "Total receive throughput in Mbps is ${total_remote_rcv_xput}"
fi

# Ask the server to relaunch its iperfs, this enables us to run the client
# in a loop: If the servers do not relaunch, they sometimes fail to send
# a server report back due to a possible iperf bug.
# nc on Fbsd sticks around even if stdin concludes, hence the "-w1" timeout.
echo "relaunch" | nc ${SERVER_IP} ${NC_PORT} -w1

# Wait for the servers to relaunch, only makes sense when running the client
# in a loop, but left it here for one-time runs too for the sake of simplicity.
echo "$(date): Sleeping for $((COOLDOWN_SECS+2))s to let the server respawn iperfs"
sleep $((COOLDOWN_SECS+2))
