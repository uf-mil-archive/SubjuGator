Sub notes:


Put

    sudo -u ANY_USER_WITH_A_ROS_INSTALL -i screen -dmS roscore bash -i -c roscore

in `/etc/rc.local`.


Add

    UseDNS no

to the end of `/etc/ssh/sshd_config`.


Run `visudo` and change

    %sudo   ALL=(ALL:ALL) ALL

to

    %sudo   ALL=NOPASSWD:ALL


In `/etc/default/rcS`, change

    # automatically repair filesystems with inconsistencies during boot
    #FSCKFIX=no

to

    # automatically repair filesystems with inconsistencies during boot
    FSCKFIX=yes


Add

    192.168.1.21 sub sub-pc
    192.168.1.22 sub-gumstix

to the end of `/etc/hosts`.
