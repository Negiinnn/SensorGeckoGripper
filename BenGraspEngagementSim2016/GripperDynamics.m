function dx = GripperDynamics(t,x,Gripper,Object)

    dx = zeros(12,1);
    
    %convenient variable names
    xG = x(1);      yG = x(2);      theta = x(3);
    xG_dot = x(4);  yG_dot = x(5);  theta_dot = x(6);
    xO = x(7);      yO = x(8);      xO_dot = x(9);     yO_dot = x(10); 
    omega = x(11);
    
    normal = [cos(theta); -sin(theta); 0]; %vector normal to gripper plane
    tangent = [sin(theta); cos(theta); 0]; %vector tangent to gripper plane
    
    %define position vectors
    r_Ocm_N0_N = [xO; yO; 0]; % position vector from N0 to Ocm in N basis
    r_Gcm_N0_N = [xG; yG; 0];
    r_Gp_Gcm_N = Gripper.w*normal;
    r_Ocm_Gp_N = r_Ocm_N0_N - r_Gcm_N0_N - r_Gp_Gcm_N;
    
    d = dot(r_Ocm_Gp_N, tangent); % "offset" distance of contact point from Gp
    
    r_GO_Gcm_N = r_Gp_Gcm_N + d*tangent;
    r_OG_Ocm_N = -Object.R*normal;
    
    %define velocity vectors
    w_G_N = [0; 0; -theta_dot];
    w_O_N = [0; 0; omega];
    
    V_Gcm_N = [xG_dot; yG_dot; 0];
    V_Ocm_N = [xO_dot; yO_dot; 0];
    V_GO_N = V_Gcm_N + cross(w_G_N, r_GO_Gcm_N);
    %V_Ocm_G0_N = V_Ocm_N - V_GO_N;
    V_OG_N = V_Ocm_N + cross(w_O_N, r_OG_Ocm_N);
    V_OG_GO_N = V_OG_N - V_GO_N;
    
    %calculate contact conditions
    P = -( dot(r_Ocm_Gp_N, normal) - Object.R); % (positive) penetration depth 
    P_dot = -dot(V_OG_GO_N, normal); % (positive) penetration velocity
    V_tan = dot(V_OG_GO_N, tangent); % tangent velocity of contact

    
    %Calculate contact forces
    FN_spring = Gripper.Ksurf*P*(sign(P)>0);
    FN_damp = Gripper.Csurf*P_dot*(sign(P)>0)*(sign(P_dot)>0);
    FN = (FN_spring + FN_damp)*(abs(d)<Gripper.span/2);
    
    Ff = FN*V_tan/(abs(P_dot)+.0001);
    
    Ff = Ff*(Ff<=(Gripper.mu_s1*FN)) + FN*Gripper.mu_k1*(Ff>(Gripper.mu_s1*FN));
    
%     Ff_min = .5;
%     if Ff~=0 && abs(Ff)<Ff_min
%         Ff = Ff_min*sign(Ff);
%     end
        
    
    %FN*Gripper.mu*sign(V_tan)*(sign(P)>0);
    
    F_contact = -FN*normal + Ff*tangent;
    
    Fx_spring = -xG*Gripper.Kx;
    Fx_damp = -xG_dot*Gripper.Cx;
    Fx = Fx_spring + Fx_damp;
%     Fx = 4*(sign(xG)<=0)*(sign(xG_dot)<=0) + ... %constant force compression
%          1*(sign(xG)<=0)*(sign(xG_dot)>=0) + ... %constant force release
%          -xG*5000*(sign(xG)>0) + ... % "hard" stop at front of gripper
%          (-.035-xG)*10000*(xG<=-.035); % "hard" stop at back of gripper
    Fy_spring = -yG*Gripper.Ky;
    Fy_damp = -yG_dot*Gripper.Cy;
    Fy = Fy_spring + Fy_damp;
    F_Gripper = [Fx;Fy] + F_contact(1:2);
    
    M_spring = Gripper.KM*theta;
    M_damp = Gripper.CM*theta_dot;
    M_Gripper = [0; 0; M_spring + M_damp] + cross(r_GO_Gcm_N, F_contact);
    M_Object = cross(r_OG_Ocm_N, -F_contact);
    
    
    dx(1:3) = x(4:6);
    dx(4:5) = F_Gripper/Gripper.m;
    dx(6) = -M_Gripper(3)/Gripper.I;
    dx(7:8) = x(9:10);
    dx(9:10) = -F_contact(1:2)/Object.m;
    dx(11) = M_Object(3)/Object.I;
    dx(12) = omega;

end

