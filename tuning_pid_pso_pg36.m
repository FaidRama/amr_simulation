%% =========================================================================
%% TUNING PID DENGAN PSO UNTUK MOTOR DC PG36 (MOTOR KIRI & KANAN)
%% =========================================================================
% Penulis: Antigravity Assistant
% Deskripsi: Script tuning parameter Kp, Ki, Kd menggunakan Particle Swarm
%            Optimization (PSO) untuk motor PG36 differential drive.
% =========================================================================

clear; clc; close all;

%% 1. MODEL SISTEM MOTOR PG36 (Transfer Function)
% Motor DC dengan gearbox umumnya dimodelkan sebagai orde 1: G(s) = K / (tau*s + 1)
% Input  : PWM / Tegangan (0 - 12V atau 0 - 255)
% Output : Kecepatan (RPM atau rad/s)
%
% *Catatan: Nilai K dan tau di bawah adalah representasi tipikal motor PG36.
%  Silakan sesuaikan jika Anda memiliki hasil uji 'system identification' riil.

% Model Motor Kiri
K_kiri   = 1.85;   % Gain steady-state motor kiri
tau_kiri = 0.12;   % Time constant motor kiri (detik)
G_kiri   = tf(K_kiri, [tau_kiri 1]);

% Model Motor Kanan (biasanya ada sedikit selisih mekanik/gesekan)
K_kanan   = 1.80;  % Gain steady-state motor kanan
tau_kanan = 0.13;  % Time constant motor kanan (detik)
G_kanan   = tf(K_kanan, [tau_kanan 1]);

%% 2. PARAMETER PSO
nParticles = 35;        % Jumlah partikel (swarm size)
maxIter    = 50;        % Maksimum iterasi
nVar       = 3;         % Variabel yang dicari: [Kp, Ki, Kd]

% Batas Pencarian [Kp, Ki, Kd]
lb = [0.01,  0.01,  0.0001];   % Lower bound
ub = [50.0, 100.0,   5.0000];  % Upper bound

% Parameter Inersia & Bobot Akselerasi
wMax = 0.9;             % Inersia awal (eksplorasi global)
wMin = 0.4;             % Inersia akhir (eksploitasi lokal)
c1   = 1.5;             % Cognitive coefficient (pengaruh best partikel sendiri)
c2   = 2.0;             % Social coefficient (pengaruh global best)

%% 3. PROSES TUNING UNTUK KEDUA MOTOR
fprintf('==============================================\n');
fprintf('   OPTIMASI PID MENGGUNAKAN PSO DIMULAI       \n');
fprintf('==============================================\n');

fprintf('>>> Mengoptimasi PID untuk Motor Kiri...\n');
[pid_kiri, cost_kiri, hist_kiri] = run_pso_pid(G_kiri, nParticles, maxIter, lb, ub, wMax, wMin, c1, c2);

fprintf('>>> Mengoptimasi PID untuk Motor Kanan...\n');
[pid_kanan, cost_kanan, hist_kanan] = run_pso_pid(G_kanan, nParticles, maxIter, lb, ub, wMax, wMin, c1, c2);

%% 4. TAMPILKAN HASIL TUNING DI COMMAND WINDOW
fprintf('\n==============================================\n');
fprintf('           HASIL TUNING PID OPTIMAL           \n');
fprintf('==============================================\n');
fprintf('Motor Kiri  -> Kp = %8.4f | Ki = %8.4f | Kd = %8.4f\n', pid_kiri(1), pid_kiri(2), pid_kiri(3));
fprintf('Motor Kanan -> Kp = %8.4f | Ki = %8.4f | Kd = %8.4f\n', pid_kanan(1), pid_kanan(2), pid_kanan(3));
fprintf('==============================================\n\n');

%% 5. SIMULASI DAN PLOT RESPON SISTEM
t = 0:0.001:1.5; % Rentang waktu simulasi (detik)

% Closed-loop Transfer Function
C_kiri  = pid(pid_kiri(1), pid_kiri(2), pid_kiri(3));
T_kiri  = feedback(C_kiri * G_kiri, 1);

C_kanan = pid(pid_kanan(1), pid_kanan(2), pid_kanan(3));
T_kanan = feedback(C_kanan * G_kanan, 1);

[y_kiri, t_kiri]   = step(T_kiri, t);
[y_kanan, t_kanan] = step(T_kanan, t);

figure('Name', 'Hasil Tuning PID PSO Motor PG36', 'Color', 'w');

% Subplot 1: Respon Step Motor Kiri & Kanan
subplot(2,1,1);
plot(t, ones(size(t)), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Setpoint'); hold on;
plot(t_kiri, y_kiri, 'b-', 'LineWidth', 1.8, 'DisplayName', sprintf('Motor Kiri [Kp=%.2f, Ki=%.2f, Kd=%.3f]', pid_kiri));
plot(t_kanan, y_kanan, 'r-', 'LineWidth', 1.8, 'DisplayName', sprintf('Motor Kanan [Kp=%.2f, Ki=%.2f, Kd=%.3f]', pid_kanan));
grid on;
title('Respon Kecepatan Step Motor Kiri & Kanan (Closed-Loop)');
xlabel('Waktu (detik)');
ylabel('Kecepatan Ternormalisasi');
legend('Location', 'Southeast');
ylim([0 1.3]);

% Subplot 2: Konvergensi Fitness PSO
subplot(2,1,2);
plot(1:maxIter, hist_kiri, 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Fitness Motor Kiri'); hold on;
plot(1:maxIter, hist_kanan, 'r-s', 'LineWidth', 1.5, 'DisplayName', 'Fitness Motor Kanan');
grid on;
title('Grafik Konvergensi Biaya (ITAE) Tiap Iterasi');
xlabel('Iterasi');
ylabel('Cost / Nilai Fitness');
legend('Location', 'Northeast');


%% =========================================================================
%% FUNGSI INTI PSO
%% =========================================================================
function [gBestPos, gBestCost, costHistory] = run_pso_pid(G, nPop, maxIter, lb, ub, wMax, wMin, c1, c2)
    nVar = length(lb);
    
    % Inisialisasi Posisi dan Kecepatan Partikel
    pos = repmat(lb, nPop, 1) + rand(nPop, nVar) .* repmat(ub - lb, nPop, 1);
    vel = zeros(nPop, nVar);
    
    pBestPos  = pos;
    pBestCost = inf(nPop, 1);
    
    gBestCost = inf;
    gBestPos  = zeros(1, nVar);
    
    costHistory = zeros(maxIter, 1);
    
    % Evaluasi Awal
    for i = 1:nPop
        cost = evaluate_fitness(pos(i,:), G);
        pBestCost(i) = cost;
        if cost < gBestCost
            gBestCost = cost;
            gBestPos  = pos(i,:);
        end
    end
    
    % Loop Iterasi PSO
    for iter = 1:maxIter
        % Update Bobot Inersia secara Dinamis (Linear Decreasing)
        w = wMax - ((wMax - wMin) * iter / maxIter);
        
        for i = 1:nPop
            % Update Kecepatan Partikel
            vel(i,:) = w * vel(i,:) ...
                     + c1 * rand(1, nVar) .* (pBestPos(i,:) - pos(i,:)) ...
                     + c2 * rand(1, nVar) .* (gBestPos - pos(i,:));
            
            % Update Posisi Partikel
            pos(i,:) = pos(i,:) + vel(i,:);
            
            % Batasan Posisi (Clamping Boundary)
            pos(i,:) = max(pos(i,:), lb);
            pos(i,:) = min(pos(i,:), ub);
            
            % Evaluasi Biaya (Cost Function)
            cost = evaluate_fitness(pos(i,:), G);
            
            % Update Personal Best (pBest)
            if cost < pBestCost(i)
                pBestCost(i) = cost;
                pBestPos(i,:) = pos(i,:);
                
                % Update Global Best (gBest)
                if cost < gBestCost
                    gBestCost = cost;
                    gBestPos  = pos(i,:);
                end
            end
        end
        costHistory(iter) = gBestCost;
    end
end

%% =========================================================================
%% FUNGSI EVALUASI FITNESS (ITAE + PENALTI OVERSHOOT)
%% =========================================================================
function J = evaluate_fitness(pid_params, G)
    Kp = pid_params(1);
    Ki = pid_params(2);
    Kd = pid_params(3);
    
    C = pid(Kp, Ki, Kd);
    T = feedback(C * G, 1);
    
    % Cek stabilitas sistem
    if any(real(pole(T)) >= 0)
        J = 1e6; % Penalti besar jika sistem tidak stabil
        return;
    end
    
    % Simulasi step response
    t = 0:0.005:1.5;
    try
        [y, t_out] = step(T, t);
    catch
        J = 1e6;
        return;
    end
    
    % Hitung Error
    ref = 1.0;
    e = ref - y;
    
    % 1. Kriteria ITAE: Integral(t * |e(t)| dt)
    itae = trapz(t_out, t_out .* abs(e));
    
    % 2. Penalti Overshoot (agar gerakan robot tidak menyentak)
    overshoot = max(0, max(y) - ref);
    penalty_overshoot = 50.0 * overshoot^2;
    
    % Total Cost Function
    J = itae + penalty_overshoot;
end
