% Communications MatLab <--> Arduino
% Matlab file 1 for use with Arduino file 1
clc;
clear all;
numSec=5;
Step = 0.006;
t=[];
v=[];

s1 = serial('COM16');    % define serial port
s1.BaudRate=57600;               % define baud rate
set(s1, 'terminator', 'CR');    % define the terminator for println
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
 
  Gx(i)=fscanf(s1,'%d');      
  Gy(i)=fscanf(s1,'%d');  
  Gz(i)=fscanf(s1,'%d');  
 if(tempo(i) > numSec)
    flag =1;
 end
end

 

end
fclose(s1);
for i =1 :+1 :length(Gx)
    
    if i==1
    Fx(i) = Gx(i);
    Fy(i) = Gy(i);
    Fz(i) = Gz(i);
     else 
     Fx(i) =  Gx(i)*0.08+ Gx(i-1)*0.92;
     Fy(i) =  Gy(i)*0.08+ Gy(i-1)*0.92;
     Fz(i) =  Gz(i)*0.08+ Gz(i-1)*0.92;
    end
end
subplot(3,1,1)
plot(tempo,Gx,'r')  ;
hold on
plot(tempo,Fx,'b')  ;
subplot(3,1,2)
plot(tempo,Gy,'r')  ;
subplot(3,1,3)
plot(tempo,Gz,'r')  ;