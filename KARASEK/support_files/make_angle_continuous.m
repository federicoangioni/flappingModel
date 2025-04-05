function output=make_angle_continuous(input)

iprev=1;
for i=1:length(input)
    if ~isnan(input(i))
        if i>1
            while abs(input(i)-input(iprev))>pi
                if input(i)-input(iprev)>pi
                    input(i)=input(i)-2*pi;
                elseif input(i)-input(iprev)<-pi
                    input(i)=input(i)+2*pi;
                end
            end
            iprev=i;
        end
    end
end

output=input;