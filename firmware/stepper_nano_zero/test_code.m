function r=test_code()
    x=[2^14-100:1:2^14-1 0:99];%;;
    r=zeros(1,numel(x));
    for i=1:numel(x)
        y = x(i)*ones(1,100)+randn(1,100)*0.1;
        y(y>2^14)= y(y>2^14)-2^14;
        y(y<0)= y(y<0)+2^14;
        r(i) = sampleMeanEncoder(100,y);
        Y((i-1)*100+1:i*100)=y;
    end
    plot(linspace(0,numel(x),numel(Y)),Y*4);
    hold on
    plot(r,'-o');
end

function m=sampleMeanEncoder(numSamples,X)
    x=0;
    last=0;
    mean=0;
    max=0;
    min=0;
    for i=1:numSamples
        last=x;
        x=(X(i)*4);

        if i==1
            last=x;
            min=x;
            max=x;
        end

        if abs(last-x)>65536/2
            if last>x
                x=x+65536;
            else
                x=x-65536;
            end

        end


        if x>max
            max=x;
        end
        if x<min
            min=x;
        end

        mean=mean+(x);
     end

    m = mean/numSamples;
 end