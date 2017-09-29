function [Tout,Xout,caught_i] = GripperDynamicsEuler(T,dt,X_0,Gripper,Object)
    
    caught_i = 0;
    isContact = false;
    
    Tout = 0:dt:T;
    N = length(Tout);
    n_states = length(X_0);
    Xout = zeros(N,n_states);
    Xout(1,:) = X_0;
    

    
    S1 = 2.5;
    N1 = -1.5;
    
    dx = zeros(1,12);
    
    Ft_min = 1;
    Vlim = 400; %rpm
    
    for i = 2:N
        
  
        %convenient variable names
        xG = Xout(i-1,1);      yG = Xout(i-1,2);      theta = Xout(i-1,3);
        xG_dot = Xout(i-1,4);  yG_dot = Xout(i-1,5);  theta_dot = Xout(i-1,6);
        xO = Xout(i-1,7);      yO = Xout(i-1,8);      xO_dot = Xout(i-1,9);     yO_dot = Xout(i-1,10); 
        omega = Xout(i-1,11);  phi = Xout(i-1,12);
    
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
        
    
        P = dot(r_Ocm_Gp_N, normal) - Object.R; % normal distance of contact
        P_dot = dot(V_OG_GO_N, normal); %penetration velocity
        V_tan = dot(V_OG_GO_N, tangent); % tangent velocity of contact
        
     
        %spring/damper forces/moments on gripper
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
        F_Gripper = [Fx;Fy;0];
        F_Object = [0;0;0];
        
        M_spring = Gripper.KM*theta;
        M_damp = Gripper.CM*theta_dot;
        M_Gripper = [0; 0; M_spring + M_damp];
        M_Object = [0;0;0];
        
        
        if ~isContact && P<0 && abs(d)<Gripper.span/2
            isContact = true;
        end
        
        Fn_spring = -Gripper.Ksurf*P*isContact;
        Fn_damp = -Gripper.Csurf*P_dot*isContact*(sign(P_dot)<0);
        
        Fn = Fn_spring + Fn_damp;
        
        Ft = (Ft_min+ Gripper.mu_k1*abs(Fn))*isContact*sign(V_tan); 
    
        %Ft = Gripper.mu_s1*abs(Fn)*sign(V_tan)*isContact; 
        
        if isContact && (Fn<0 || abs(d)<Gripper.span/2)
            isContact = false;
        end
        
        F_contact = -Fn*normal + Ft*tangent;
        F_Gripper = F_Gripper + F_contact;
        M_Gripper = M_Gripper + cross(r_GO_Gcm_N, F_contact);
        M_Object = cross(r_OG_Ocm_N, -F_contact);
        F_Object = -F_contact;
        
%         Ff = Ff*(Ff<=(Gripper.mu_s1*FN)) + FN*Gripper.mu_k1*(Ff>(Gripper.mu_s1*FN));

        
        % check for initial engagement with gripper and attach springs
%         if ~isContact && P<0 && abs(d)<Gripper.span/2
%             isContact = true;
%             d_contact = d;
%             angle_contact = phi + theta;
%         end
%             
%         
%         if isContact
%             
%             Fn = -Gripper.Ksurf*P;
%             
%             r_GO_Gcm_N = r_Gp_Gcm_N + d*tangent;
%             r_OG_Ocm_N = -Object.R*normal;
%             
%             w_G_N = [0; 0; -theta_dot];
%             w_O_N = [0; 0; omega];
% 
%             V_Gcm_N = [xG_dot; yG_dot; 0];
%             V_Ocm_N = [xO_dot; yO_dot; 0];
%             V_GO_N = V_Gcm_N + cross(w_G_N, r_GO_Gcm_N);
%             %V_Ocm_G0_N = V_Ocm_N - V_GO_N;
%             V_OG_N = V_Ocm_N + cross(w_O_N, r_OG_Ocm_N);
%             V_OG_GO_N = V_OG_N - V_GO_N;
% 
%             %calculate contact conditions
%             P_dot = -dot(V_OG_GO_N, normal); % (positive) penetration velocity
%             V_tan = dot(V_OG_GO_N, tangent); % tangent velocity of contact
%             
%             Ft = Gripper.mu_s1*abs(Fn)*sign(V_tan);
% 
%             
%             %dtan = (d - d_contact) - Object.R*(phi+theta - angle_contact);
%             %Ft = Gripper.Ksurf*dtan;
%             
%             %are Fn and Ft under the adhesion curve
%             if Fn < 0 || abs(d)>Gripper.span/2%-N1/S1^2*(Ft - S1)^2 + N1
%                 isContact = false;
%                 Fn = 0;
%                 Ft = 0;
%             end
%             
%             F_contact = -Fn*normal + Ft*tangent;
%             F_Gripper = F_Gripper + F_contact;
%             M_Gripper = M_Gripper + cross((r_Gp_Gcm_N + d_contact*tangent), F_contact);
%             M_Object = cross(-Object.R*normal, -F_contact);
%             F_Object = -F_contact;
%             
%         end

        dx(1:3) = Xout(i-1,4:6);
        dx(4:5) = F_Gripper(1:2)/Gripper.m;
        dx(6) = -M_Gripper(3)/Gripper.I;
        dx(7:8) = Xout(i-1,9:10);
        dx(9:10) = F_Object(1:2)/Object.m;
        dx(11) = M_Object(3)/Object.I;
        dx(12) = omega;

        Xout(i,:) = Xout(i-1,:) + dx*dt;
        


        if abs(d) < Gripper.d_tan/2 && P < Gripper.d_norm && abs(omega)<(400*pi/180)
            %i_end = i;
            caught_i = i;
            break
        end
   
    end
        
end