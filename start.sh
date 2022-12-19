if v4l2-ctl --list-devices | grep -q 'Motorized Camera'; then
   echo "Camera exists"
   python pan_tilt_tracking.py
else 
    sudo modprobe v4l2loopback devices=1 card_label="Motorized Camera" exclusive_caps=1
    python pan_tilt_tracking.py
fi

