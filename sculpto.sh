#!/bin/bash

cp ConfigSamples/Sculptoboard/config src/config.default
./BuildShell make -j8
make combined
ssh -i ~/.ssh/auto root@$1 "killall sh; killall socat"
scp -i ~/.ssh/auto LPC1768/main-combined.hex root@$1:
ssh -i ~/.ssh/auto root@$1 "/root/PrinterAPI/tools/lpc_flasher/flash.sh /root/main-combined.hex"
