clear; clc; close all

% initial conditions
xs = 0; % x and y position memory for returning to print
ys = 0;
x0 = [0.15; 0; 0];
y0 = [0.1; 0; 0];
tt = [22; 0; 0];
q = 0; % initial mode
ti = 0; % initial time

% inputs defined here
ul = .5; % sin function multiplier for printing motion
ur = -.5; % cos function multiplier for printing motion
un = 22.5; % voltage for nozzle
L = 1; % New filament loaded?
C = 1; % New color needed?
F = 0; % New filament needed?
R = 1; % Excess filament extracted?
NH = 0; % Nozzle temp high?
NL = 0; % Nozzle temp low?
V = 1; % Vertical motion? (angular sensor)
H = 1; % Horizontal motion? (angular sensor)

% condense into a variable space
state = [xs; ys; x0; y0; tt; q; ti];
input = [ul; ur; un; L; C; F; R; NH; NL; V; H];

% printing history
xhistory = [];
yhistory = [];

% not printing history
nxhistory = [];
nyhistory = [];

while state(13,1)<=300 % seconds of simulation
    if state(9,1) >= 190 % NH and NL if statement
        input(8,1) = 1;
        input(9,1) = 0;
    elseif state(9,1) <= 25
        input(9,1) = 1;
        input(8,1) = 0;
    else
        input(9,1) = 0;
        input(8,1) = 0;
    end
    [g, state] = guards(state, input);
    state(12,1) = state(12,1) + g;
    %disp(state(12,1))
    [state, x, y, input] = dynamics(state, input); % simulate ODEs for 1 second period
    if state(12,1) ~= 0
        nxhistory = [nxhistory; x(:,1)];
        nyhistory = [nyhistory; y(:,1)];
        xhistory = [xhistory; NaN];
        yhistory = [yhistory; NaN];
    else
        xhistory = [xhistory; x(:,1)];
        yhistory = [yhistory; y(:,1)];
        nxhistory = [nxhistory; NaN];
        nyhistory = [nyhistory; NaN];
    end
end

figure;
plot(xhistory, yhistory, 'g-');
hold on;
plot(nxhistory, nyhistory, 'r-');
hold off;
xlabel('X Position [m]'); 
ylabel('Y position [m]');
legend('Printing (q0)', 'Not Printing (q1-q6)');
title({'Motion Path of Tool Head relocating to (0,0)','and Reloading with Filament'});

function xxd = xpath(xx, ux) % state space model of x movement

m = 0.5;     
Jm = 1e-5;    
rm = 0.01;    
Km = 0.05;    
mulx = 0.1;    
muly = 0.1;     
mur = 1e-4;    
rb = 0.005;   
Lm = 1e-3;   
Rm = 2.0;
xd = xx(2);
ix = xx(3);

xx1 = xx(2);
xx2 = (((Km * ix) / rm) - (2 * mulx * xd) - ((4 * mur * xd) / (rb ^ 2)))/(m + (2 * Jm / (rm ^ 2)));
xx3 = (1 / Lm) * (ux - (Rm * ix) - (2 * Km * xd / rm));

if (xx(1) <= 0 && xx(2) < 0) || (xx(1) >= 0.2 && xx(2) > 0) % 200mm wide printing area
    xx1 = 0;
end

xxd = [xx1; xx2; xx3];
end

function yyd = ypath(yy, uy) % state space model of y movement

m = 0.5;     
Jm = 1e-5;    
rm = 0.01;    
Km = 0.05;    
mulx = 0.1;    
muly = 0.1;     
mur = 1e-4;    
rb = 0.005;   
Lm = 1e-3;   
Rm = 2.0;
yd = yy(2);
iy = yy(3);

yy1 = yy(2);
yy2 = (((Km * iy) / rm) - (2 * muly * yd) - ((4 * mur * yd) / (rb ^ 2)))/(m + (2 * Jm / (rm ^ 2)));
yy3 = (1 / Lm) * (uy - (Rm * iy) - (2 * Km * yd / rm));

if (yy(1) <= 0 && yy(2) < 0) || (yy(1) >= 0.2 && yy(2) > 0) % 200mm long printing area
    yy1 = 0;
end

yyd = [yy1; yy2; yy3];
end

function ttd = tprof(tt , un) % state space model of the nozzle temperature

Te = 22;
Rn  = 14.4;
Ctn = 9.0; 
Rtn = 5.0; 
Ln  = 1e-3;  
Cn  = 100e-6;
Tn = tt(1);
in = tt(2);
urn = tt(3);

ttd1 = (urn ^ 2)/(Rn * Ctn) - (Tn - Te)/(Rtn * Ctn);
ttd2 = (un - urn)/Ln;
ttd3 = (in - (urn/Rn))/Cn;

ttd = [ttd1; ttd2; ttd3];

end

function [state, x, y, input] = dynamics(state, input)

xs = state(1,1);
ys = state(2,1);
x0 = state(3:5,1);
y0 = state(6:8,1);
tt = state(9:11,1);
q  = state(12,1);
ti = state(13,1);

tspan = linspace(ti, ti+1, 5);

if state(12,1) == 0
    ul = @(t) input(1,1)*sin(-3*t);
    ur = @(t) input(2,1)*cos(5*t);
    un = @(t) input(3,1);
elseif state(12,1) == 2
    ul = @(t) 0;
    ur = @(t) 0;
    un = @(t) input(3,1);
elseif state(12,1) == 3
    ul = @(t) 0;
    ur = @(t) 0;
    un = @(t) 0;
elseif state(12,1) == 4 % return to saved printing coordinates
    un = @(t) input(3,1);
    % implement a simple proportional controller to drive to xs and ys
    errorx = xs - x0(1);
    errory = ys - y0(1);
    ul = @(t) .5*(errorx + errory);
    ur = @(t) .5*(errorx - errory);
    input(5,1) = 0;
else
    % find (0,0)
    ul = @(t) -.5;
    ur = @(t) 0;
    un = @(t) input(3,1);
end

ux = @(t) ul(t) + ur(t);
uy = @(t) ul(t) - ur(t);

[tx, x] = ode45(@(t, x) xpath(x, ux(t)), tspan, x0);
[ty, y] = ode45(@(t, y) ypath(y, uy(t)), tspan, y0);
[txt, xt] = ode45(@(t, xt) tprof(xt, un(t)), tspan, tt);

if x(end,1) == state(3,1) % setting H angular sensor reading to true or false based on velocity
    input(11,1) = 0;
else
    input(11,1) = 1;
end

if y(end,1) == state(6,1) % setting V angular sensor reading to true or false based on velocity
    input(10,1) = 0;
else
    input(10,1) = 1;
end

state = [xs; ys; x(end,:)'; y(end,:)'; xt(end,:)'; q; ti+1];
end

function [g, state] = guards(state, input) % this function also contains the few resets present in the model
% states: 1:xs, 2:ys, 3:x, 6:y, 9:Tn, 12:q, 13:ti
if state(12,1) == 0
    if input(5,1) == 1 || input(6,1) == 1
        g = 1;
        state(1,1) = state(3,1);
        state(2,1) = state(6,1);
    else
        g = 0;
    end
elseif state(12,1) == 1
    if input(10,1) == 0
        g = 5;
        state(6,1) = 0;
    elseif input(11,1) == 0
        g = 4;
        state(3,1) = 0;
    else
        g = 0;
    end
elseif state(12,1) == 2
    if input(7,1) == 1
        g = 1;
    else
        g = 0;
    end
elseif state(12,1) == 3
    if input(4,1) == 1 && input(9,1) == 1
        g = 1;
    else
        g = 0;
    end
elseif state(12,1) == 4
    if input(8,1) == 1 && abs(state(1,1) - state(3,1)) < 0.01 && abs(state(2,1) - state(6,1)) < 0.01
        g = -4;
    else
        g = 0;
    end
elseif state(12,1) == 5
    if input(10,1) == 0
        g = -3;
    else
        g = 0;
    end
else
    if input(11,1) == 0
        g = -4;
    else
        g = 0;
    end
end
end
