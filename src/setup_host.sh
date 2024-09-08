#run this first

socat -d -d UNIX-LISTEN:/tmp/vbox,fork /dev/ttyUSB0,raw
chmod 777 /dev/ttyUSB0
chmod 777 /tmp/vbox

# set serial port in vbox as COM1, Host Pipe, check Connect to existing, path is /tmp/vbox
#run vm
#in guest run chmod 777 /dev/ttyS0
#run your software
