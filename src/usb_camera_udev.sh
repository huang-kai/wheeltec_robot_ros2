echo  'KERNEL=="video*", ATTRS{idVendor}=="1bcf", ATTRS{idProduct}=="2284", ATTR{index}=="0", MODE:="0777", OWNER:="root", GROUP:="video", SYMLINK+="aoni_a50"' >>/etc/udev/rules.d/camera_1.rules

service udev reload
sleep 2
service udev restart