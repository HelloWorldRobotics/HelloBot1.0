echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout",  SYMLINK+="HYJ_DiPan"' >/etc/udev/rules.d/HYJ_DiPan.rules

service udev reload
sleep 2
service udev restart
