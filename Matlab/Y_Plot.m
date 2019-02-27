clear all;
clc;
close all;
delete(instrfindall);



 
x1=1;
x2=1;

while(1)
    
    
    comport1 = serial('/dev/ttyUSB1','BaudRate',115200,'DataBits',8);
    
   
    set(comport1,'Parity','none');

  
    fopen(comport1);
    
    axis([0 100 -200 200]);
    
    
    
    x2=x2+1;
    
     
    if(x2>100)
       f2=100;
    end 
    if(x2<101)
       f2=x2;
    end
    
    
    AxisY = fscanf(comport1,'%s');
     
    Y = strfind(AxisY,'y','ForceCellOutput',false);
    
    
      
    
    FindAxisY = isempty(Y); %boolean. True or Not
    
    figure(2);
    
    y2(f2)=fscanf(comport1,'%f');

    plot(y2,'r.','linewidth',1);
    grid on;    
    hold on;
    %drawnow;
    fclose(comport1);
    
    drawnow;
    
   
    end

%%%%%    
  %  if(x>=100)
   %    loop=0;
    %   while(loop<99)
     %      loop=loop+1;
      %    y1(loop)=y1(loop+1);
       %   y2(loop)=y2(loop+1);
       %end

    %%end
%%%%%%
 
    fclose(comport1);
delete(comport1);