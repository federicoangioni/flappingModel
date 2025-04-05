function yF=LP_filter(b,a,y)

y1=y;
y1(isnan(y1))=0;
yF=filtfilt(b,a,y1);