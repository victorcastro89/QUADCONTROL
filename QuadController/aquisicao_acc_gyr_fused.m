% Communications Mat        Lab <--> Arduino
% Matlab file 1 for use with Arduino file 1
clc;
clear all;
numSec=3        ;
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
 Pitch_acc(i)=fscanf(s1,'%f');  
  Pitch_gyr(i)=fscanf(s1,'%f');
 Pitch_fused(i)=fscanf(s1,'%f');
  Roll_acc(i)=fscanf(s1,'%f');  
  Roll_gyr(i)=fscanf(s1,'%f');
  Roll_fused(i)=fscanf(s1,'%f');
 if(tempo(i) > numSec)
    flag =1;
 end
end
end
fclose(s1);
 display(['Collected']);
 subplot(2,1,1)

plot(tempo,Pitch_acc* 57.2958,'r')  ;
title('Pitch');
hold on
plot(tempo,Pitch_gyr* 57.2958,'b')  ;
hold on
plot(tempo,Pitch_fused* 57.2958,'g')  ;

subplot(2,1,2)
plot(tempo,Roll_acc* 57.2958,'r')  ;
title('Roll');
hold on
plot(tempo,Roll_gyr* 57.2958,'b')  ;
hold on
plot(tempo,Roll_fused* 57.2958,'g')  ;