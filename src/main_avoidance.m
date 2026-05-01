clc; clear; close all;

%% 1. Inizializzazione Robot e Simulazione
try
    robot = importrobot('kukanomesh.urdf', 'DataFormat', 'column');
catch
    error('File URDF non trovato! Assicurati che kukanomesh.urdf sia nella cartella.');
end

client = RemoteAPIClient();
sim = client.require('sim');
sim.setStepping(true);
sim.startSimulation();

% Handles
h_base = sim.getObject('/LBRiiwa7R800');
h_joints = zeros(7,1);
for i = 1:7, h_joints(i) = sim.getObject(sprintf('/joint_%d', i)); end
h_obs = [sim.getObject('/Obstacle_1'), sim.getObject('/Obstacle_2')];
h_ee = sim.getObject('/Cylinder');
h_cp = [sim.getObject('/link4_resp'), sim.getObject('/link5_resp'), ...
        sim.getObject('/link6_resp'), sim.getObject('/link7_resp'), ...
        sim.getObject('/link8_resp'), sim.getObject('/Cylinder')];

target_names = {'/first', '/second', '/third', '/fourth'};

%% 2. Parametri di Controllo
Kp = 10.0; k0 = 30.0; Ts = 0.02; tol = 0.03; v_max = 1.2; 
stuck_threshold = 0.002; stuck_time_limit = 0.4;  
ei_world = [0;0;0]; prev_dist = inf; stuck_timer = 0;
giri = 0; t_idx = 1; keep_running = true;

% Inizializzazione Telemetria
history = struct('time',[],'error',[],'min_dist',[],'q_dot',[],'target_idx',[]);

fprintf('=== Avvio Simulazione: Il robot farà 2 giri completi ===\n');

%% 3. Loop Principale
while keep_running
    h_tgt = sim.getObject(target_names{t_idx});
    p_tgt = getP(sim, h_tgt);
    ei_world = [0;0;0];
    stuck_timer = 0;
    fprintf('>>> Puntando a: %s\n', target_names{t_idx});
    
    while keep_running
        % A. Leggi Stato
        q = zeros(7,1);
        for i = 1:7, q(i) = sim.getJointPosition(h_joints(i)); end
        p_ee = getP(sim, h_ee);
        
        m_base = sim.getObjectMatrix(h_base, -1);
        R_base = [m_base{1}, m_base{2}, m_base{3}; 
                  m_base{5}, m_base{6}, m_base{7}; 
                  m_base{9}, m_base{10}, m_base{11}];
        
        p_obs = [getP(sim, h_obs(1)), getP(sim, h_obs(2))];
        p_cp = zeros(3, 6);
        for i=1:6, p_cp(:,i) = getP(sim, h_cp(i)); end
        
        % B. Errore e Stuck Detection
        curr_dist = norm(p_tgt - p_ee);
        if abs(prev_dist - curr_dist) < stuck_threshold
            stuck_timer = stuck_timer + Ts;
        else
            stuck_timer = 0;
            ei_world = ei_world * 0.9; 
        end
        prev_dist = curr_dist;

        % C. Integratore di Sblocco
        if stuck_timer > stuck_time_limit
            ei_world = ei_world + (p_tgt - p_ee) * 30.0 * Ts;
            if norm(ei_world) > 1.5, ei_world = (ei_world/norm(ei_world))*1.5; end
        end

        % D. Controllo
        ep_world = Kp * (p_tgt - p_ee);
        q_dot = controller_nullspace(robot, q, ep_world, ei_world, R_base, p_obs, p_cp, k0);
        
        if norm(q_dot) > v_max, q_dot = (q_dot / norm(q_dot)) * v_max; end
        
        % --- REGISTRAZIONE DATI (PRIMA dello step) ---
        history.time(end+1) = sim.getSimulationTime();
        history.error(end+1) = curr_dist;
        history.q_dot(end+1, :) = q_dot';
        history.target_idx(end+1) = t_idx;
        dm = inf;
        for i=1:6, for j=1:2, dm = min(dm, norm(p_cp(:,i)-p_obs(:,j))); end; end
        history.min_dist(end+1) = dm;

        % E. Attuazione
        q_next = q + q_dot * Ts;
        for i = 1:7, sim.setJointPosition(h_joints(i), q_next(i)); end
        
        % F. Verifica Fine Target
        if curr_dist < tol
           if t_idx == 4
                giri = giri + 1;
                fprintf('*** Giro %d completato ***\n', giri);
           end
           if giri == 2
                keep_running = false; 
                break; 
           end
           break; 
        end
        sim.step();
    end
    t_idx = mod(t_idx, length(target_names)) + 1;
end

%% 4. Fine e Grafici
sim.stopSimulation();
fprintf('Simulazione terminata. Generazione grafici in corso...\n');

if isempty(history.time)
    fprintf(2, 'ERRORE: Nessun dato raccolto! Controlla i loop.\n');
else
    % Creazione Finestra Grafica
    hFig = figure('Name', 'Analisi KUKA Obstacle Avoidance', 'Color', 'w', 'NumberTitle', 'off');
    set(hFig, 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]); % Grandezza 80% schermo

    % 1. Errore
    subplot(3,1,1);
    plot(history.time, history.error, 'b', 'LineWidth', 1.5); hold on; grid on;
    yline(tol, '--r', 'Target Tol');
    title('Errore di Posizione (Distanza Target-EE)'); ylabel('Metri');

    % 2. Ostacoli
    subplot(3,1,2);
    plot(history.time, history.min_dist, 'Color', [0.85 0.325 0.098], 'LineWidth', 1.5); hold on; grid on;
    yline(0.15, '--r', 'Rho Critical');
    title('Distanza Minima Robot-Ostacoli'); ylabel('Metri');
    patch([history.time fliplr(history.time)], [history.min_dist zeros(size(history.min_dist))], ...
          [0.85 0.325 0.098], 'FaceAlpha', 0.1, 'EdgeColor', 'none');

    % 3. Velocità Joint
    subplot(3,1,3);
    plot(history.time, history.q_dot, 'LineWidth', 1); grid on;
    title('Velocità delle Joint'); ylabel('rad/s'); xlabel('Tempo [s]');
    legend('J1','J2','J3','J4','J5','J6','J7', 'Location', 'eastoutside');

    drawnow; % Forza il disegno
    shg;     % Porta la finestra in primo piano
    fprintf('=== GRAFICI GENERATI CON SUCCESSO ===\n');
end

function p = getP(sim, h)
    r = sim.getObjectPosition(h, -1);
    p = [r{1}; r{2}; r{3}];
end