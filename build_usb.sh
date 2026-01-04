echo "start copy usb.rules to /etc/udev/rules.d/"
sudo cp usb.rules /etc/udev/rules.d

service udev reload
sleep 2
service udev restart
echo "Finish!!!"
