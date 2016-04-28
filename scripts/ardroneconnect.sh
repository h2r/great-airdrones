#!/usr/bin/env bash

usage() {
cat << EOF

     __     __  __  __      __
 /\ |__)   |  \|__)/  \|\ ||_
/--\| \ .  |__/| \ \__/| \||__

WPA/WPA2 support

This script connects the AR Drone to a WPA/WPA2 secured network.

Usage:
     script/connect "<essid>" -p "<password>" [-a <address>] [-d <droneip>]
     Note that order of arguments matters.

  <essid>
          Name of the WPA2 network to connect the drone to.

  <password>
          Password of the network.

  <address>
          (optional)  Address to  be set  on the  drone when  connected to  the
          network. Use  a different address  than the router's default.  Set to
          "auto" (default) to let the router DHCP server auto-assign an IP.

  <droneip>
          (optional) Current drone's ip address. Default is 192.168.1.1
EOF

exit 1
}

   ESSID=$1
PASSWORD=$3
   DHCPC=""

 DRONEIP=${7:-"192.168.1.1"}
 ADDRESS=${5:-"auto"}

[[ -z $ESSID    ]] && usage;
[[ -z $PASSWORD ]] && usage;

set -ue

echo "ESSID: $ESSID"
echo "PASSWORD: $PASSWORD"
echo "ADDRESS: $ADDRESS"
echo "DRONE_IP: $DRONEIP"

SCRIPT="\
wpa_passphrase \"$ESSID\" \"$PASSWORD\" >/etc/wpa_supplicant.conf || exit 1;\n\
echo \"create config...\";\n\
wpa_supplicant -B -D wext -i ath0 -dd -c /etc/wpa_supplicant.conf || exit 1;\n\
echo \"connect...\";\n\
\n\
sleep 5;\n\
if [[ \"$ADDRESS\" == \"auto\" ]]; then\n\
    /sbin/udhcpc -i ath0;\n\
else\n\
    ifconfig ath0 $ADDRESS || exit 1;\n\
fi\n\
echo \"assign ip...\";\n\
"

echo "echo -e '$SCRIPT' >/data/video/wpa-wifi.sh &&\
      chmod +x /data/video/wpa-wifi.sh &&\
      /data/video/wpa-wifi.sh &>/data/video/wpa-wifi.log &&\
      wpi_cli status &>/data/video/wpa-wifi.log"\
  | telnet $DRONEIP &>/dev/null
