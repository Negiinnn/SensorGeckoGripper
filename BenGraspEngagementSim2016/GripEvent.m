function [value, isterminal, direction] = GripEvent(t, x, Gripper, Object)

    normal = [cos(x(3)); -sin(x(3))]; %vector normal to gripper plane
    tangent = [sin(x(3)); cos(x(3))]; %vector tangent to gripper plane

    %define position vectors
    r_Ocm_N0_N = x(7:8); % position vector from N0 to Ocm in N basis
    r_Gcm_N0_N = x(1:2);
    r_Gp_Gcm_N = Gripper.w*normal;
    r_Ocm_Gp_N = r_Ocm_N0_N - r_Gcm_N0_N - r_Gp_Gcm_N;
    
    d_tangent = dot(r_Ocm_Gp_N, tangent); % "offset" distance of contact point from Gp
    d_normal = dot(r_Ocm_Gp_N, normal) - Object.R;

    if abs(d_tangent) < Gripper.d_tan/2 && d_normal < Gripper.d_norm && abs(x(11))<(500*pi/180)
        value = -1;
    else
        value = 1;
    end

    isterminal = 1;
    direction = -1;

end

