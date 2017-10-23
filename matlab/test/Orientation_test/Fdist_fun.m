function Fdist = Fdist_fun(t)

global Fdist_min Fdist_max t1 t2
Fdist_min = 10;
Fdist_max = 80;

t1 = 0.4;
t2 = 1.5;

tf = t2 - t1;

tb = tf*0.15;
a = (Fdist_max - Fdist_min) / tb;

Fdist = Fdist_min;

if (t>t1 && t<t2)
    if (t < t1+tb)
        Fdist = a*(t-t1) + Fdist_min;
    elseif (t < t2-tb)
        Fdist = Fdist_max;
    else
        Fdist = -a*(t-t2) + Fdist_min;
    end
end

end

