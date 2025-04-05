function plot_rectangles(indexUP,indexDOWN,timesUP,timesDOWN,time,limMIN,limMAX)

if indexUP(1)>indexDOWN(1)
    time(1)
    limMIN
    timesDOWN(1)
    limMAX-limMIN
    rectangle('Position',[time(1) limMIN timesDOWN(1)-time(1) limMAX-limMIN],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none'),hold on
    if length(indexDOWN)>length(indexUP)
        for i=1:length(indexUP)
            rectangle('Position',[timesUP(i) limMIN timesDOWN(i+1)-timesUP(i) limMAX-limMIN],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none'),hold on
        end
    else
        for i=1:length(indexUP)-1
            rectangle('Position',[timesUP(i) limMIN timesDOWN(i+1)-timesUP(i) limMAX-limMIN],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none'),hold on
        end
        rectangle('Position',[timesUP(end) limMIN time(end)-timesUP(end) limMAX-limMIN],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none'),hold on
    end
else
    if length(indexUP)>length(indexDOWN)
        for i=1:length(indexUP)-1
            rectangle('Position',[timesUP(i) limMIN timesDOWN(i)-timesUP(i) limMAX-limMIN],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none'),hold on
        end
        rectangle('Position',[timesUP(end) limMIN time(end)-timesUP(end) limMAX-limMIN],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none'),hold on
    else
        for i=1:length(indexUP)
            rectangle('Position',[timesUP(i) limMIN timesDOWN(i)-timesUP(i) limMAX-limMIN],'FaceColor',[0.9 0.9 0.9],'EdgeColor','none'),hold on
        end
    end
end