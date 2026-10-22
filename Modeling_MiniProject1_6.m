clear; clc; close all;

% Defining Assumed Constants
N = 50; 
L = 0.1; 
Vf = 0.02;
Tn = 210; 
Tb = 60; 
Te = 25;
alpha = 5.8e-8; 
lambdac = 0.05; 
mu = 0.17;

dx = L/N;

tc = dx / Vf;                 
tp = L / Vf;                 
dt = 0.05;   
tv = 0:dt:(tp + 10); % time interval runs till 10 s after total print time

T = zeros(N, 1);
Th = zeros(N, length(tv));

for k = 1:length(tv)
    t = tv(k);
    nc = min(N, max(1, floor(t / tc)));  %Determining deposited cells
    
    if T(nc) == 0 % Enforcing Dirichlet BC
        T(nc) = Tn; 
    end

    dT = zeros(N, 1);
    for i = 1:nc
        % Extrusion Phase Dynamics
        if t <= tp
            ip = max(1, i - 1);
            in = min(nc, i + 1);
        else % Cooling Phsae Dynamics
            ip = mod(i - 2, N) + 1;
            in = mod(i, N) + 1;
        end

        %ODE formula
        d2T = (T(in) - 2*T(i) + T(ip)) / (dx^2);
        dT(i) = alpha * d2T - lambdac * (T(i) - Te) - mu * (T(i) - Tb);
    end
    
    if t <= tp % Enforcing Neumann BC
        dT(nc) = 0; 
    end
    
    T(1:nc) = T(1:nc) + dt * dT(1:nc);
    Th(:, k) = T;
end

figure;
imagesc(tv, (1:N)*dx, Th);
colorbar;
colormap(turbo);
xline(tp, 'w-', 'LineWidth', 2, 'label','Post Extrusion Phase Begins (Filament layer is Deposited)','FontSize',10,'Color','k');
xlabel('Time [s]'); 
ylabel('Filament Position x [m]');
% title('Filament Temperature Profile T(x,t)');