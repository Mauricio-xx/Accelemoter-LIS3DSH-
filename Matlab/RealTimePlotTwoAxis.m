clear all;
clc;
close all;
delete(instrfindall);



 
x1=1;
x2=1;

while(1)
    
    comport = serial('/dev/ttyUSB0','BaudRate',115200,'DataBits',8);
    set(comport,'Parity','none');

    fopen(comport);
    
    axis([0 100 -200 200]);
    
    
    x1=x1+1;
    
    x2=x2+1;
    
    
    if(x1>100)
       f1=100;
    end 
    if(x1<101)
       f1=x1;
    end
    
    
    if(x2>100)
       f2=100;
    end 
    if(x2<101)
       f2=x2;
    end
    
    
    AxisXorY = fscanf(comport,'%s');
     
    Y = strfind(AxisXorY,'y','ForceCellOutput',false);
    X = strfind(AxisXorY,'x','ForceCellOutput',false);  
    
      
    
    FindAxisY = isempty(Y);
    FindAxisX = isempty(X);
    
          
    if (FindAxisX == 0)
    
        figure(1);
        
    y1(f1)=fscanf(comport,'%f');

    plot(y1,'b.','linewidth',1);
    grid on;    
    hold on;
    %drawnow;
    fclose(comport);
    %end
    

    
    elseif(FindAxisY == 0)
    
    figure(2);
    
    y2(f2)=fscanf(comport,'%f');

    plot(y2,'r.','linewidth',1);
    grid on;    
    hold on;
    %drawnow;
    fclose(comport);
    end
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
    
    fclose(comport);
delete(comport);