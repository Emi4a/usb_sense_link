#!/bin/sh

# http://wiki.ubuntuusers.de/Benutzer_und_Gruppen
# http://www.togaware.com/linux/survivor/Standard_Groups.html

# Execute this shell script once if you do
# not want to execute core/lms with sudo.

# To see USB devices in /dev/ the current
# user must be member of tty.
sudo addgroup $USER tty

# To use serial devices the current user
# must be member of dialout.
sudo addgroup $USER dialout

