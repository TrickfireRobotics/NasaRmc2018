#this is a demo to show how to setup nodes that need superuser access
cd devel/lib/tfr_control    # cd to the directory with your node
chown root:root pwm_bridge # change ownship to root
chmod a+rx  pwm_bridge    # set as executable by all
chmod u+s pwm_bridge       # set the setuid bit
