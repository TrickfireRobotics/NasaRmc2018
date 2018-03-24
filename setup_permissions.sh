#this is a demo to show how to setup nodes that need superuser access
cd devel/lib/tfr_control    # cd to the directory with your node
chown root:root  control # change ownship to root
chmod a+rx control # set as executable by all
chmod u+s control # set the setuid bit
chown root:root  adc_service # change ownship to root
chmod a+rx adc_service # set as executable by all
chmod u+s adc_service # set the setuid bit
