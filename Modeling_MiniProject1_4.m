clear; clc; close all
tspan = linspace(0, 20, 100);

xd0 = [0; 0; 0];
yd0 = [0; 0; 0];
ux = zeros(size(tspan));
uy = zeros(size(tspan));

ul = @(t) sin(t);
ur = @(t) cos(t);

ux = @(t) ul(t) + ur(t);
uy = @(t) ul(t) - ur(t);

x0 = [0, 0, 0];
y0 = [0, 0, 0];

[tx, x] = ode45(@(t, x) xpath(x, ux(t)), tspan, xd0);
[ty, y] = ode45(@(t, y) ypath(y, uy(t)), tspan, yd0);


figure;
plot(x(:,1), y(:,1), 'g-');
xlabel('X Position [m]'); 
ylabel('Y position [m]');
title('Motion Path of Tool Head forming an Ellipse');

ul1 = @(t) 4*sin(-5*t);
ur1 = @(t) 2*cos(3*t);

ux1 = @(t) ul1(t) + ur1(t);
uy1 = @(t) ul1(t) - ur1(t);

x0 = [0, 0, 0];
y0 = [0, 0, 0];

[tx1, x1] = ode45(@(t, x) xpath(x, ux1(t)), tspan, xd0);
[ty1, y1] = ode45(@(t, y) ypath(y, uy1(t)), tspan, yd0);


figure;
plot(x1(:,1), y1(:,1), 'r-');
xlabel('X Position [m]'); 
ylabel('Y position [m]');
title('Motion Path of Tool Head forming a Lissajou Curve');

function xxd = xpath(xx, ux)

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

xxd = [xx1; xx2; xx3];
end

function yyd = ypath(yy, uy)

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

yyd = [yy1; yy2; yy3];
end

