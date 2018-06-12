#!/bin/sh
# auto-boot is achieved in /etc/rc.local

cd /home/pi/tp-recon/

# confirmed that this script runs at boot with tp-recon as the working directory
echo "running tp-recon/boot.sh" > /tmp/tp_test.txt
echo "running tp-recon/boot.sh" > boot_test.txt

#sudo lxterminal -e python3 daemon.py

sudo python3 daemon.py