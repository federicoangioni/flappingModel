function [t_amploff, ampl, off, pitch_avg, dev_avg, imax, imin, AoA, strkpln] = get_ampl_off(time,stroke,pitch,deviation)

stroke1=[stroke(2:end);stroke(1)];
index=1:length(stroke);

nrising=zeros(size(stroke));
nrising(stroke<stroke1)=1;
nmaxmin=[0;diff(nrising)];

% indices of max and min
imax=index(nmaxmin<0);
imin=index(nmaxmin>0);

NN=sum(nmaxmin); % NN = 0 --> nmax = nmin
                 % NN = 1 --> nmax = nmin + 1
                 % NN = -1 --> nmax +1 = nmin

% halfstroke averaged stroke, pitch and deviation
off=NaN(length(imax)+length(imax)-1,1);
pitch_avg=NaN(length(imax)+length(imax)-1,1);
dev_avg=NaN(length(imax)+length(imax)-1,1);
strkpln=NaN(length(imax)+length(imax)-1,1);
    
AoA=NaN(size(time));

if NN==0
    ampl1=stroke(imax)-stroke(imin);
%     off1=(stroke(imax)+stroke(imin))/2;
    t_amploff1=(time(imax)+time(imin))/2;
    if imax(1)<imin(1)
        ampl2=stroke(imax(2:end))-stroke(imin(1:end-1));
%         off2=(stroke(imax(2:end))+stroke(imin(1:end-1)))/2;
        t_amploff2=(time(imax(2:end))+time(imin(1:end-1)))/2;
    else
        ampl2=stroke(imax(1:end-1))-stroke(imin(2:end));
%         off2=(stroke(imax(1:end-1))+stroke(imin(2:end)))/2;
        t_amploff2=(time(imax(1:end-1))+time(imin(2:end)))/2;
    end
    
    for i=1:length(imax)
        if imax(1)<imin(1)
            pitch_avg(2*i-1)=mean(pitch(imax(i):imin(i)));
            dev_avg(2*i-1)=mean(deviation(imax(i):imin(i)));
            off(2*i-1)=mean(stroke(imax(i):imin(i)));
            strkpln(2*i-1)=atan(sin(stroke(imax(i):imin(i)))\sin(deviation(imax(i):imin(i))));
            
            AoA(imax(i):imin(i)-1)=pi/2-pitch(imax(i):imin(i)-1);
            time1(imax(i):imin(i)-1)=time(imax(i):imin(i)-1);
            
            if i<length(imax)
                pitch_avg(2*i)=mean(pitch(imin(i):imax(i+1)));
                dev_avg(2*i)=mean(deviation(imin(i):imax(i+1)));
                off(2*i)=mean(stroke(imin(i):imax(i+1)));
                strkpln(2*i)=atan(sin(stroke(imin(i):imax(i)))\sin(deviation(imin(i):imax(i))));
            
                AoA(imin(i):imax(i+1)-1)=pitch(imin(i):imax(i+1)-1)-pi/2;
                time1(imin(i):imax(i+1)-1)=time(imin(i):imax(i+1)-1);
            
            end
            
        else
            pitch_avg(2*i-1)=mean(pitch(imin(i):imax(i)));
            dev_avg(2*i-1)=mean(deviation(imin(i):imax(i)));
            off(2*i-1)=mean(stroke(imin(i):imax(i)));
            
%             atand(sin(stroke(imin(i):imax(i)))\sin(deviation(imin(i):imax(i))))
%             figure,plot(sin(stroke(imin(i):imax(i))),sin(deviation(imin(i):imax(i)))), axis equal
            
            strkpln(2*i-1)=atan(sin(stroke(imin(i):imax(i)))\sin(deviation(imin(i):imax(i))));
            
            AoA(imin(i):imax(i)-1)=pi/2-pitch(imin(i):imax(i)-1);
            time1(imin(i):imax(i)-1)=time(imin(i):imax(i)-1);
            
            if i<length(imin)
                pitch_avg(2*i)=mean(pitch(imax(i):imin(i+1)));
                dev_avg(2*i)=mean(deviation(imax(i):imin(i+1)));
                off(2*i)=mean(stroke(imax(i):imin(i+1)));
                strkpln(2*i)=atan(sin(stroke(imax(i):imin(i+1)))\sin(deviation(imax(i):imin(i+1))));
            
                AoA(imax(i):imin(i+1)-1)=pitch(imax(i):imin(i+1)-1)-pi/2;
                time1(imax(i):imin(i+1)-1)=time(imax(i):imin(i+1)-1);
            end
        end
    end

    
elseif NN==1
    ampl1=stroke(imax(1:end-1))-stroke(imin);
%     off1=(stroke(imax(1:end-1))+stroke(imin))/2;
    t_amploff1=(time(imax(1:end-1))+time(imin))/2;
    
    ampl2=stroke(imax(2:end))-stroke(imin);
%     off2=(stroke(imax(2:end))+stroke(imin))/2;
    t_amploff2=(time(imax(2:end))+time(imin))/2;
    
    for i=1:length(imin)
        pitch_avg(2*i-1)=mean(pitch(imax(i):imin(i)));
        pitch_avg(2*i)=mean(pitch(imin(i):imax(i+1)));
        dev_avg(2*i-1)=mean(deviation(imax(i):imin(i)));
        dev_avg(2*i)=mean(deviation(imin(i):imax(i+1)));
        off(2*i-1)=mean(stroke(imax(i):imin(i)));
        off(2*i)=mean(stroke(imin(i):imax(i+1)));
        strkpln(2*i-1)=atan(sin(stroke(imax(i):imin(i)))\sin(deviation(imax(i):imin(i))));
        strkpln(2*i)=atan(sin(stroke(imin(i):imax(i+1)))\sin(deviation(imin(i):imax(i+1))));
        
        AoA(imax(i):imin(i)-1)=pi/2-pitch(imax(i):imin(i)-1);
        AoA(imin(i):imax(i+1)-1)=pitch(imin(i):imax(i+1)-1)-pi/2;
        time1(imax(i):imin(i)-1)=time(imax(i):imin(i)-1);
        time1(imin(i):imax(i+1)-1)=time(imin(i):imax(i+1)-1);
    end
    
elseif NN==-1
    ampl1=stroke(imax)-stroke(imin(1:end-1));
%     off1=(stroke(imax)+stroke(imin(1:end-1)))/2;
    t_amploff1=(time(imax)+time(imin(1:end-1)))/2;
    
    ampl2=stroke(imax)-stroke(imin(2:end));
%     off2=(stroke(imax)+stroke(imin(2:end)))/2;
    t_amploff2=(time(imax)+time(imin(2:end)))/2;
    
    for i=1:length(imax)
        pitch_avg(2*i-1)=mean(pitch(imin(i):imax(i)));
        pitch_avg(2*i)=mean(pitch(imax(i):imin(i+1)));
        dev_avg(2*i-1)=mean(deviation(imin(i):imax(i)));
        dev_avg(2*i)=mean(deviation(imax(i):imin(i+1)));
        off(2*i-1)=mean(stroke(imin(i):imax(i)));
        off(2*i)=mean(stroke(imax(i):imin(i+1)));
        strkpln(2*i-1)=atan(sin(stroke(imin(i):imax(i)))\sin(deviation(imin(i):imax(i))));
        strkpln(2*i)=atan(sin(stroke(imax(i):imin(i+1)))\sin(deviation(imax(i):imin(i+1))));
        
        AoA(imin(i):imax(i)-1)=pi/2-pitch(imin(i):imax(i)-1);
        AoA(imax(i):imin(i+1)-1)=pitch(imax(i):imin(i+1)-1)-pi/2;
        time1(imin(i):imax(i)-1)=time(imin(i):imax(i)-1);
        time1(imax(i):imin(i+1)-1)=time(imax(i):imin(i+1)-1);
    end
end

t_amploff=[t_amploff1; t_amploff2];
ampl=[ampl1; ampl2];
% off=[off1; off2];

[t_amploff, isorted]=sort(t_amploff);
ampl=ampl(isorted);
% off=off(isorted);

pitch_avg=pi/2-abs(pi/2-pitch_avg); % to get pitch from 0 to 90 deg
AoA=pi/2-AoA;

