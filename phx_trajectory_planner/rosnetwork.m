 % vorher in /etc/hosts ip adresse von phoenix-nuc auf eigenem pc ändern
 
 %phoenix wifi
%  setenv('ROS_MASTER_URI','http://192.168.0.53:11311');
%  setenv('ROS_IP','192.168.0.104');
%  rosinit('http://192.168.0.53:11311', 'NodeHost','192.168.1.1', 'NodeName', 'Test') % replace own IP Adress
 
 %copter hotspot
 setenv('ROS_MASTER_URI','http://10.42.0.1:11311');
 setenv('ROS_IP','10.42.0.141');
 rosinit('http://10.42.0.1:11311', 'NodeHost','192.168.1.1', 'NodeName', 'Test') % replace own IP Adress

%local test
%  setenv('ROS_MASTER_URI','http://192.168.56.1:11311');
%  setenv('ROS_IP','10.183.94.190');
%  rosinit('http://192.168.56.1:11311', 'NodeHost','192.168.1.1', 'NodeName', 'Test') % replace own IP Adress

 %Sub = rossubscriber('/chatter')
%  Sub = rossubscriber('/phx/fc/attitude')
%  scandata = receive(Sub,3)