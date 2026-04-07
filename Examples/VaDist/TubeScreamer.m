%-----------------------------------------------------
% PBMMI Assignment 6 - Virtual Analog Tube Screamer
% 
% This script simulates an analog model of an Ibanez tube screamer. This
% model uses the K method and a Newton Raphson iterative solver to
% determine the roots of the K equation.
% 
% B221438 - 13/04/23
%-----------------------------------------------------

clear all
close all
clc

% Simulation parameters
SR = 88.2e3;                % sample rate [Hz]
dur = .1;                   % duration of sim [s]
Nf = round(SR*dur);         % number of samples in sim

% Settings
plotting = true;            % plotting on/off
audio = true;               % play output sounds on/off
drive = 100;                % drive 0-100
symmetric = true;          % use symmetric transfer function (true) or asymmetric (false)
% Input - create sine wave input
f0 = 100;                   % frequency [Hz]
amp = 1;                    % amplitude of input
tvec = dur*(0:Nf-1)'/Nf;    % time vector [s]
u = amp*sin(2*pi*f0*tvec);  % input vector

% Physical parameters
r1 = 10e3;                  % resistance of R1 [Ohms]
r2 = 51e3;                  % resistance of R2 [Ohms]
r3 = 4.7e3;                 % resistance of R3 [Ohms]
r4 = 10e3;                  % resistance of R4 [Ohms]
rdist = drive*50e3;         % resistance of Rdist [Ohms]
c1 = 1e-6;                  % capacitance of C1 [F]
c2 = 51e-12;                % capacitance of C2 [F]
c3 = 47e-9;                 % capacitance of C3 [F]

Is = 2.52e-9;               % diode saturation current (A)
Vt = 25.83e-3;              % diode thermal voltage (V)
Ni = 1.752;                 % diode ideality factor

% Newton-raphson paratemers
tol = 1e-9;                 % convergence tolerance
max_iter = 50;              % maximum allowed iterations per time-step

% K-method params

% system matricies
A = -[(1/(r1*c1)), 0, 0 ; 1/(r3*c2), (1/((r2+rdist)*c2)), (1/(r3*c2)) ; (1/(r3*c3)), 0, (1/(r3*c3))];
B = [(1/(r1*c1)); (1/(r3*c2)); (1/(r3*c3)) ];
C = [0;(-1/c2);0];
D = [0,1,0];
L = [-1,1,0];
M = 1;

k = 1/SR;                   % sample period
H_min = (2/k)*eye(3) - A;   % scheme matrix H-
H_plus = (2/k)*eye(3) + A;  % scheme matrix H+
K = D*(H_min^(-1))* C;      % K coefficient


x1 = [0 0 0]';              % x_n
x0 = [0 0 0]';              % x_n+1

f0 = 0;                     % f(x_n+1)
f1 = 0;                     % f(x_n)

v0 = [0 0 0];               % initialize voltage at n+1
v1 = [0 0 0];               % initialize voltage at n+1

u0 = 0;                     % initialize input at n
u1 = 0;                     % initialize input at n+1

y = zeros(Nf,1);            % output vector

tic
for n = 1:Nf

    % combined quantity
    P_n = D * (H_min^(-1)) * ( H_plus*x1 + (B)*(u1+u0) + C*(f1));

    % newton raphson
    v0 = v1;                % initial guess
    iter = 0;               % iteration counter
    step = 1;               % initial step

    while (iter < max_iter) && (all(abs(step) > tol)) 
        if symmetric == true
            f0 = 2*Is * sinh(v0/(Ni*Vt));                       % f(v^n+1)
            J = ((2*K*Is)/(Ni*Vt)) * cosh(v0/(Ni*Vt)) -1;       % derivative of k equation
        else
           f0 = Is * ( exp(x0/(2*Ni*Vt)) - exp(-x0/(Ni*Vt)));                  % f(v^n+1)
            J = K * Is/(Ni*Vt) * ( exp(x0/(2*Ni*Vt))*.5 + exp(-x0/(Ni*Vt))) -1; % derivative of k equation
        end 
        g = P_n + K * f0 - v0;                              % K equation 
        step = g/J;                                         % step at current iteration
        v0 = v0 - step;                                     % guess at current iteration
        iter = iter + 1;                                    % iteration counter
    end

    x0 = H_min^(-1)*( H_plus*x1 + (B)*(u1+u0) + C*(f1+f0)); % equation for x^n+1

    % output
    x0 = x0(:,1);
    y(n) = L*x0 + M*u0;                                     % output equation

    % state update
    x1 = x0;
    u1 = u0;
    v1 = v0;
    u0 = u(n);
    f0 = f1;
    
end
toc

if plotting == true
yfft = abs(fft(y));
figure
subplot(2,1,1)
plot(tvec,u,tvec,y)
title('Time domain plot of input and output signals')
xlabel('time [s]')
ylabel('amplitude')
subplot(2,1,2)
loglog((0:(Nf-1)/2)'*floor(SR/Nf), yfft(1:Nf/2))
title('Frequency domain plot of output signal')
xlabel('frequency [Hz]')
ylabel('amplitude')
xlim([20,20000])
end

if audio == true
soundsc(y,SR)
end
