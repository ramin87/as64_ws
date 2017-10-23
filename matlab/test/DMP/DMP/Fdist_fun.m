function Fdist = Fdist_fun(t)

global cmd_args

t1 = cmd_args.t1_fdist;
t2 = cmd_args.t2_fdist;

tf = t2 - t1;

tb = tf*0.15;
a = (cmd_args.Fdist_max - cmd_args.Fdist_min) / tb;

Fdist = cmd_args.Fdist_min;

if (t>t1 && t<t2)
    if (t < t1+tb)
        Fdist = a*(t-t1) + cmd_args.Fdist_min;
    elseif (t < t2-tb)
        Fdist = cmd_args.Fdist_max;
    else
        Fdist = -a*(t-t2) + cmd_args.Fdist_min;
    end
end

end

