# ROS Communication with CAN
- Raspberry Pi CAN communication  
    
sudo slcand -o -c -f s4 /dev/ttyUSB0 slcan0    
    
sudo ifconfig slcan0 up    
    
- CAN Communication Monitor   
    
candump slcan0
___
- install slcan module to Raspberry Pi    
https://wiki.linklayer.com/index.php/CANtact_on_Raspberry_Pi
