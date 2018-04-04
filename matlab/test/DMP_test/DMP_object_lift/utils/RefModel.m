%% Reference model

classdef RefModel < handle
    properties
        
        t
        y
        dy
        ddy

    end

    methods
        %% Reference model constructor
        function this = RefModel(y0, g, tau, a6, a7)

            this.init(y0, g, tau, a6, a7);

        end

        function init(this, y0, g, tau, a6, a7)

            t = 0:0.002:tau;
            
            this.t = t;
            
            dt = diff(t);
            tau = t(end);
            t = t/tau;

            a0 = y0;
            a1 = 0;
            a2 = 0;

            A = [1 1 1; 3 4 5; 6 12 20];
            b = [g-a0-a1-a2-a6-a7; 0-a1-2*a2-6*a6-7*a7; 0-2*a2-30*a6-42*a7];
            a345 = A\b;
            a3 = a345(1);
            a4 = a345(2);
            a5 = a345(3);

            %% Create data
            y = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5 + a6*t.^6 + a7*t.^7;
            dy = (a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4 + 6*a6*t.^5 + 7*a7*t.^6)/tau;
            ddy = (2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3 + 30*a6*t.^4 + 42*a7*t.^5)/tau^2;

%             figure;
%             hold on;
%             plot(t*tau,dy, t*tau,ddy);
%             
%             dy = [diff(y)./dt 0];
%             ddy = [diff(dy)./dt 0];
%             
%             plot(t*tau,dy, t*tau,ddy);
%             legend('dy_1','ddy_1','dy_2','ddy_2')
%             hold off;
%             
%             dy(1) = 0.0;
%             ddy(1) = 0.0;
%             
            dy(end) = 0.0;
            ddy(end) = 0.0;
            
%             y = [y y+y(end) 2*y(end)-y];
%             dy = 3*[dy dy -dy];
%             ddy = 9*[ddy ddy -ddy];
%             
%             dt = tau/(length(y)-1);
%             this.t = 0:dt:tau;

            this.y = y;
            this.dy = dy;
            this.ddy = ddy;


        end
        
        function [yr, dyr, ddyr] = getRef(this, t1)
            
            if (t1 > this.t(end))
                yr = this.y(end);
                dyr = this.dy(end);
                ddyr = this.ddy(end);
            else
                yr = interp1(this.t, this.y, t1);
                dyr = interp1(this.t, this.dy, t1);
                ddyr = interp1(this.t, this.ddy, t1);
            end
            
        end

    end
end
