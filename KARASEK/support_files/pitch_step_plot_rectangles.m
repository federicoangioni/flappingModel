% tt0=(45700+14*36-106)/fps;
% tt1=(45700+14*36)/fps;
% tt2=(45700+14*36+21*36)/fps;
% tt3=(45700+14*36-106+407)/fps;

rectangle('Position',[-1 -10000 1 20000],'FaceColor',[0.8 1 0.8],'EdgeColor','none'),hold on
rectangle('Position',[0 -10000 tt3-tt0 20000],'FaceColor',[1 1 0.8],'EdgeColor','none'),hold on
% rectangle('Position',[0 -10000 TIMEman 20000],'FaceColor',[0.8 0.8 1],'EdgeColor','none'),hold on
% rectangle('Position',[TIMEman -10000 0.43 20000],'FaceColor',[1 0.8 0.8],'EdgeColor','none'),hold on
% rectangle('Position',[0.43 -10000 0.8 20000],'FaceColor',[1 1 0.8],'EdgeColor','none'),hold on
rectangle('Position',[tt3-tt0 -10000 3-(tt3-tt0) 20000],'FaceColor',[0.8 1 0.8],'EdgeColor','none'),hold on