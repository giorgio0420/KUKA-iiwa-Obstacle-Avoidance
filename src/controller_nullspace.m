function q_dot = controller_nullspace(robot, q, ep_world, ei_world, R_base, obs_world, cp_pos_world, k0)
    eeName = 'Cylinder';
    lambda = 0.05; 
    
    % --- Parametri di Sicurezza (Invariati come richiesto) ---
    rho_influence = 0.45; 
    rho_critical  = 0.15; 

    d_min = inf;
    for i = 1:size(cp_pos_world, 2)
        for j = 1:size(obs_world, 2)
            d_min = min(d_min, norm(cp_pos_world(:,i) - obs_world(:,j)));
        end
    end
    alpha = smoothstep(d_min, rho_critical, rho_influence);

    % 1. Task Primario
    J_full_b = geometricJacobian(robot, q, eeName);
    J_w = R_base * J_full_b(4:6, :);
    J_w_pinv = J_w' / (J_w*J_w' + lambda^2 * eye(3));
    
    % Somma errore proporzionale + spinta integrale di sblocco
    v_target = ep_world + ei_world; 
    q_dot_main = J_w_pinv * (alpha * v_target);

    % 2. Task Secondario (STRUTTURA INVARIATA)
    cp_urdf = {'link4_resp','link5_resp','link6_resp','link7_resp','link8_resp','Cylinder'};
    q0_dot = zeros(7,1);
    for i = 1:length(cp_urdf)
        p_cp_w = cp_pos_world(:, i);
        for oi = 1:size(obs_world, 2)
            dv = p_cp_w - obs_world(:, oi);
            d  = norm(dv);
            if d < rho_influence
                J_cp_b = geometricJacobian(robot, q, cp_urdf{i});
                Jv_cp_w = R_base * J_cp_b(4:6, :);
                f_mag = k0 * (1/d - 1/rho_influence) * (1/d^2);
                f_rep = f_mag * (dv/d);
                f_vortex = k0 * 0.5 * cross([0;0;1], dv/d); 
                q0_dot = q0_dot + Jv_cp_w' * (f_rep + f_vortex);
            end
        end
    end

    N = eye(7) - J_w_pinv * J_w;
    q_dot_null = N * q0_dot;
    if norm(q_dot_null) > 2.0, q_dot_null = (q_dot_null / norm(q_dot_null)) * 2.0; end

    q_dot = q_dot_main + q_dot_null;
end

function s = smoothstep(x, min_d, max_d)
    if x < min_d, s = 0.1; 
    elseif x > max_d, s = 1.0;
    else, s = (x - min_d) / (max_d - min_d);
    end
end