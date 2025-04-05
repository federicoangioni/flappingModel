function wingbeat_axis_top(ax,N,show_labels)

if show_labels==1
    labels=repmat({''},1,floor(N)+1);
    labels(1)={0};
    labels(end)={floor(N)};
else
    labels=[];
end

axes('Position',ax.Position,'XAxisLocation','top','YAxisLocation','right','XLim',[0,N],'XTick',[0:1:floor(N)],'XTickLabels',labels,'Ytick',[],'YColor','none','Color','none');

if show_labels==1
    xlabel('wingbeats (-)')
end
