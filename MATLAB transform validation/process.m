format long
syms tilt pan
syms sp
sp = sin(pan);
st = sin(tilt);
cp = cos(pan);
ct = cos(tilt);

Tderivedorig_new = [[cp * ct sp -st * cp 0.19];[-sp * ct cp sp * st 0];[st 0 ct 0.395];[0 0 0 1]];
Tcam_in_tilt = [[0 0 1 0];[-1 0 0 0];[0 -1 0 0];[0 0 0 1]];
Tcam_in_body = Tderivedorig_new * Tcam_in_tilt;

for i = 1:size(poses_base)
    Tbase = eye(4);
    Tpan = eye(4);
    Ttilt = eye(4);
    
    pan = pans(i);
    tilt = tilts(i);
    Tderivedcopy = Tderivedorig_new;
    Tderivedcopy = subs(Tderivedcopy);
    Tderivedcopy = vpa(Tderivedcopy);
    
    base_o = poses_base(i).Orientation;
    base_p = poses_base(i).Position;
    q = [base_o.W base_o.X base_o.Y base_o.Z];
    p = [base_p.X ; base_p.Y ; base_p.Z];
    rotm = quat2rotm(q);
    Tbase(1:3,1:3) = rotm;
    Tbase(1:3,4) = p;
    
    pan_o = poses_pan(i).Orientation;
    pan_p = poses_pan(i).Position;
    q = [pan_o.W pan_o.X pan_o.Y pan_o.Z];
    p = [pan_p.X ; pan_p.Y ; pan_p.Z];
    rotm = quat2rotm(q);
    Tpan(1:3,1:3) = rotm;
    Tpan(1:3,4) = p;
    
    tilt_o = poses_tilt(i).Orientation;
    tilt_p = poses_tilt(i).Position;
    q = [tilt_o.W tilt_o.X tilt_o.Y tilt_o.Z];
    p = [tilt_p.X ; tilt_p.Y ; tilt_p.Z];
    rotm = quat2rotm(q);
    Ttilt(1:3,1:3) = rotm;
    Ttilt(1:3,4) = p;
    
    
    display('tilt in base');
    inv(Tbase) * Ttilt
    Tderivedcopy
end