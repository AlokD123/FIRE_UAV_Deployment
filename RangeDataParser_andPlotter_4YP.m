clear
delete(instrfindall);

%Definitions
pastValsBufferSize = 10;    %Moving average filter length
threshold=250;               %Obstacle detection threshold distance

readSerialPort=1;
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
    id=fopen('LIDAR_data-even,90deg.txt');
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

%Figure visualization setup
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
    
    %Plot R-Theta data, sorted by angle AWAY FROM IN FRONT (0deg)
    a=1:halfPer;
    angle=a-ceil(length(a)/2);
    subplot(2,2,3)
    plot(angle,inpVec(a),'.')
    title(sprintf('Fused-sensor range detection (Polar coordinates)'))
    xlabel('Angle (degrees)')
    ylabel('Range of target (cm)')
    ylim([0 800])
    hold on
    if(mod(j,pastValsBufferSize)==0)
        hold off
    end;
    
    %Plot X,Y data
    subplot(2,2,4)
    plot(X,Y,'.')
    title(sprintf('Fused-sensor range detection (Cartesian coordinates)'))
    xlabel('Horizontal distance from center (cm)')
    ylabel('Distance in front (cm)')
    ylim([0 500])
    hold on
    if(mod(j,pastValsBufferSize)==0)
        hold off
    end;
    
    %Get typical distance at each angle ("average" over past bufferSize length)
    for(a=1:halfPer)
        setOfAveDist=getTypicalDistance(Q(a,:),pastValsBufferSize);
        distPolar(a,1)=setOfAveDist(1,j);
        %Detect obstacles closer than threshold value
        if(distPolar(a,1)<threshold)
            boolObstacle=1;
        end
    end

    if(boolObstacle==1)
        str='Obstacle detected!';
        boolObstacle=0;
    else
        str='No obstacles';
    end
    
    %Plot typical "averaged" data
    subplot(2,2,[1:2])
    plot(angle,distPolar)
    title(sprintf('Obstacle Detection System: %s    Smoke: not present\n\Fused-sensor range detection (polar coordinates)',str))
    xlabel('Angle (degrees)')
    ylabel('Average range of target (cm)')
    ylim([0 800])
 
    hold off;
    
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
