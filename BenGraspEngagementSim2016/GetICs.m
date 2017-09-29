function [x_o, y_o, x_o_dot, y_o_dot] = GetICs(offset, angle, vel, Object, Gripper, Dist)

    y_o = offset + Dist*sin(angle);
    x_o = Dist*cos(angle) + Object.R + Gripper.w;
    x_o_dot = -vel*cos(angle);
    y_o_dot = -vel*sin(angle);

end

