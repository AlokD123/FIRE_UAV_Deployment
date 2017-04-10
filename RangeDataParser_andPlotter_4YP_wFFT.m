clear
delete(instrfindall);

%Definitions
pastValsBufferSize = 10;
threshold=50;

readSerialPort=0;
inpVec=zeros(2,1);
X=zeros(2,1);
Y=zeros(2,1);

%Initialization
if(readSerialPort==1)
    % Open serial port
    id = serial('COM7');
    set(id,'BaudRate',115200);
    fopen(id);
else
    % Open file
    id=fopen('LIDAR_data.txt');
end
%Initialize variables
value=-4;
boolSmoke=0;
halfPer=0;
boolObstacle=0;

%Start parsing
boolCW=1; %Assume starting direction CW (increasing pos)
i=1; %Assume starting position 1 (NOT 0)
j=1; y=-3;
boolFirstTime=1;

%Timing
%tic
%z=0;

str='No obstacles';

figure('units','normalized','outerposition',[0 0 1 1]);

while(value~=0)
    %MAKE SURE YOU START SCRIPT BEFORE DATA TRANSMISSION
    
    %Breakpoint continue code (Note: ONLY put breakpoints after while loop)
%     while(y~=-3)
%         g=fgetl(id);
%         if(~ischar(g))
%             value=0;
%         else
%             y=str2num(g);
%         end
%     end
%     y=0;
%     i=1;
    %Skip first values until reached a starting position of 1
    if(boolFirstTime==1)
        while(str2num(fgetl(id))~=-3)
        
        end
    end
    while((value~=-3) && (value~=0))
    %Get one 1/2 period of data    
        x=fgetl(id);
        %z=[z;toc];
        fprintf('i=%d       ',i);
        if(x==-1)
            value=0;
        else
            value=str2num(x);
        end
        if(value~=-3)
            if(value~=0)
                inpVec(i)=value;
                fprintf('Value=%d\n',inpVec(i));
            end
            if(boolCW==1)
                i=i+1;
            else
                i=i-1;
            end
        else
            boolSmoke=value;
        end
    end
    %Get half-period (for CW only), and change direction
    if(boolCW==1)
        if(halfPer~=i-1)
            halfPer=i-1;
            fprintf('Half period=%d\n',halfPer);
        end
        i=halfPer;
    else
        i=i+1; %Normally restart at i=1
    end
    boolCW =~boolCW;
    %Append, to matrix Q, the column vector inp(1:halfPer)
    if(boolFirstTime==1)
        Q=inpVec(1:halfPer);
        boolFirstTime=0;
    else
        Q=[Q inpVec(1:halfPer)];
    end
    if(value~=0) %If not at end...
        value=-4; %Reset value to continue
    end
    
    %Calculate X,Y values
    for(a=1:halfPer)
        X(a)=inpVec(a)*sin(angle(a)*pi/180);
        Y(a)=inpVec(a)*cos(angle(a)*pi/180);
    end
    
    %Get typical distance at each angle ("average" over past bufferSize length)
    for(a=1:halfPer)
        setOfAveDist=getTypicalDistance(Q(a,:),pastValsBufferSize);
        distPolar(a,1)=setOfAveDist(1,j);
        if(distPolar(a,1)<threshold)
            boolObstacle=1;
        end
    end
    
    L=length(Q(a,:));
    dft=fft(Q(a,:));
        P2 = abs(dft/L);
        P1 = P2(1:L/2+1);
        P1(2:end-1) = 2*P1(2:end-1);
    
    f = (0:(L/2))/L;
    stem(f,P1) 
    ylim([0 10])
    title('Single-Sided Amplitude Spectrum of Time-series data')
    xlabel('Normalized frequency')
    ylabel('Magnitude of data')
%     hold on
%     if(mod(j,pastValsBufferSize)==0)
%         hold off
%     end;

    
    pause(0.00001) 
    
    j=j+1;
end


%Finalization
if(readSerialPort==1)
    % Close serial port
    fclose(id);
else
    % Close file
    fclose(id);
end
