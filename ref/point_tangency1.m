function y= point_tangency( x )
        global r;       
        x0=0;
        y0=0;
        r0=r;
        x1=x(1);
        y1=x(2);
        %%%找两个切点（x_1,y_1),(x_2,y_2)
        k1=(y0*x0+y1*x1-y0*x1-y1*x0+(r0^2*(-2*y0*y1-2*x0*x1+y1^2+y0^2+x0^2-r0^2+x1^2))^(1/2))/(-r0^2+x0^2-2*x0*x1+x1^2);
        k2= (y0*x0+y1*x1-y0*x1-y1*x0-(r0^2*(-2*y0*y1-2*x0*x1+y1^2+y0^2+x0^2-r0^2+x1^2))^(1/2))/(-r0^2+x0^2-2*x0*x1+x1^2);
        x_1=(-k1*y1+x0+k1^2*x1+y0*k1)/(1+k1^2);
        y_1 =-(-y1-k1*x0-y0*k1^2+k1*x1)/(1+k1^2);
        x_2=(-k2*y1+x0+k2^2*x1+y0*k2)/(1+k2^2);
        y_2 =-(-y1-k2*x0-y0*k2^2+k2*x1)/(1+k2^2);
        %%%%判断逆时针是先到哪个切点%%%%%%%%
        w=[x0 y0 1];
        r1=[x_1-x0 y_1-y0 0];
        v1=cross(w,r1);
        r2=[x_2-x0 y_2-y0 0];
        v2=cross(w,r2);
        s=[x1-x0 y1-y0 0];
        s1=s*v1';
        s2=s*v2';
        plot(x1,y1,'o');
        if(s1>0)
            y(1)=x_1;
            y(2)=y_1;
        end
        if(s2>0)
            y(1)=x_2;
            y(2)=y_2;
        end

end
