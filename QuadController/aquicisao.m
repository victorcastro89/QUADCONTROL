% Communications Mat        Lab <--> Arduino
% Matlab file 1 for use with Arduino file 1
clc;
clear all;
numSec=30           ;
Step = 0.006;
t=[];
v=[];

s1 = serial('COM16');    % define serial port
s1.BaudRate=57600;               % define baud rate
set(s1, 'terminator', 'CR');    % define the terminator for println
%fclose(s1);
fopen(s1);

%try                             % use try catch to ensure fclose
                                % signal the arduino to start collection

    display(['Collecting data']);
  % fprintf(s1,'%s\n','A');     % establishContact just wants 
                                % something in the buffer
%end

  %   t = zeros(1,(numSec/Step) +1);
  %  for i = 1: ((numSec/Step) +1)
flag =0;
i =0;
while(flag==0)
w=fscanf(s1,'%s');              % must define the input % d or %s, etc.


if (w=='A')
 
        i=i+1;
         if(i==1)
   
   aux= fscanf(s1,'%d')/1000000;
    tempo(i) = 0;
   
         else
     tempo(i)= fscanf(s1,'%d')/1000000 -aux ;

         end
 
%   Gx(i)=fscanf(s1,'%f');      
%   Gy(i)=fscanf(s1,'%f');  
%   Gz(i)=fscanf(s1,'%f');  
%   Gx_(i)=fscanf(s1,'%f');      
%   Gy_(i)=fscanf(s1,'%f');  
%   Gz_(i)=fscanf(s1,'%f');
  Roll(i)=fscanf(s1,'%f');  
  Pitch(i)=fscanf(s1,'%f');
  Yaw(i)=fscanf(s1,'%f');
  Roll_(i)=fscanf(s1,'%f');  
  Pitch_(i)=fscanf(s1,'%f');
  Yaw_(i)=fscanf(s1,'%f');
 if(tempo(i) > numSec)
    flag =1;
 end
end
end
fclose(s1);
 display(['Collected']);
 subplot(3,1,1)

plot(tempo,Roll* 57.2958,'r')  ;
title('Rolll');
hold on
plot(tempo,Roll_* 57.2958,'b')  ;

subplot(3,1,2)
plot(tempo,Pitch* 57.2958,'r')  ;title('Pitch')
hold on
plot(tempo,Pitch_* 57.2958,'b')  ;

subplot(3,1,3)
plot(tempo,Yaw* 57.2958,'r')  ;title('Yaw')
hold on
plot(tempo,Yaw_* 57.2958,'b')  ;

% subplot(4,1,1)
% 
% plot(tempo,Gx,'r')  ;
% title('Rolll');
% hold on
% plot(tempo,Gx_,'b')  ;
% subplot(4,1,2)
% 
% plot(tempo,Gy,'r')  ;title('Pitch')
% hold on
% plot(tempo,Gy_,'b')  ;
% subplot(4,1,3)
% 
% plot(tempo,Gz,'r')  ;title('Yaw')
% hold on
% plot(tempo,Gz_,'b')  ;
% 
% subplot(4,1,4)
% plot(tempo,Roll,'r')  ;title('Roll -red Pitch Blue')
% hold on
% plot(tempo,Pitch,'b')  ;