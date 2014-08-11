% Communications MatLab <--> Arduino
% Matlab file 1 for use with Arduino file 1
clc;
clear all;
numSec=6;
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
    aux0= fscanf(s1,'%d')/1000000;
  
    tempo(i) = 0;
   
         else
     tempo(i)= fscanf(s1,'%d')/1000000 - aux0;

         end
  Mx(i)=fscanf(s1,'%f');      
  My(i)=fscanf(s1,'%f');  
 
  if(tempo(i) > numSec)
    flag =1;
 end

end


end
fclose(s1);

plot(Mx,My,'.')  ;
%hold on

%plot(tempo,RollRate,'b')  ;	
