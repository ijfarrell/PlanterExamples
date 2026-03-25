%----------------------------------
% PBMMI Assignment 5 - BB2: Plate Reverb with Mode Pruning
%
% This script improves upon the initial script by implementing a
% methodology of 'pruning' or removing modes based on selected 
% characteristics. These characteristics are primarily their frequency and
% damping proximity to other calculated modes, and their effect at the
% input and output locations, determined by the modal shape functions.
% These can be user defined, with frequency threshold in cents. The IR
% times are calculated and resultant signals are played and plotted to
% analyze. 
%
% B221438 - 03/04/23
%----------------------------------

clc
clear
close all

% sample rate
SR = 44.1e3;            % sample rate [Hz]

% physical parameters
Lx = 1;                 % dimensions [m]
Ly = 2;                 % "
H = 50e-4;               % thickness [m]
T = 100;                % tension [N/m]
rho = 7.87e3;           % density [kg/m^3]
E = 200e9;              % youngs modulus [N/m^2]
v = 0.29;               % poisons ratio
T60_low = 1;            % T60 for first mode
T60_high = 1;           % T60 for highest mode

% i/o parameters
xi = 0.5;                % input x position [m]   
yi = 0.5;                % input y position [m]
xo = 0.75;               % output x position [m]
yo = 0.75;               % output y position [m]

% Mode pruning parameters
cent_range = 5;         % frequency pruning range [¢]
damp_thresh = .5;       % damping pruning range (fraction)
phi_thresh = 1e-5;      % phi i/o threshold (should be near 0)

% derived paramaters
k = 1/SR;                                                                   % sample period
K = sqrt( E*(H^2) / ( 12*rho*(1-v^2) ) );                                   % kappa
c = sqrt( T/(rho*H) );                                                      % wavespeed
Nf =  floor(SR*(T60_low));                                                  % number of samples

omega_max = 2/k;                                                            % maximum modal frequency
beta_max = sqrt( (-c^2 + sqrt(c^4 + 4 * omega_max^2 * K^2)) / (2*K^2) );    % maximum modal wavenumber

Mx = floor(sqrt( ( (Lx*Ly*beta_max)^2 - (pi*Lx)^2 ) / (pi*Ly)^2 ));         % max mode in x direction
My = floor(sqrt( ( (Lx*Ly*beta_max)^2 - (pi*Ly)^2 ) / (pi*Lx)^2 ));         % max mode in y direction

[MX,MY] = meshgrid(1:Mx,1:My);                                              % generate indexing matricies for mx and my
mx = reshape(MX,[Mx*My,1]);                                                 % reshape MX matrix to mx vector
my = reshape(MY,[Mx*My,1]);                                                 % reshape MY matrix to my vector

beta = sqrt( (mx*pi/Lx).^2 + (my*pi/Ly).^2 );                               % generate wavenumber vector

stab_index = beta<beta_max;                                                 % generate stability logical index
beta_stable = nonzeros( beta .* stab_index );                               % remove unstable values from wavenumber vector

omega = sqrt( c^2 * beta_stable.^2 + K^2 * beta_stable.^4 );                % generate frequency vector

sig_0 = ((6*log(10))/ ((max(beta_stable)^2)-(min(beta_stable)^2))) * ( ((max(beta_stable)^2)/T60_low) - ((min(beta_stable)^2))/T60_high);   % calculate loss parameter at beta_m^2 = 0
sig_1 = ((6*log(10))/ ((max(beta_stable)^2)-(min(beta_stable)^2))) * ( 1/T60_high - 1/T60_low );                                            % calculate tan(slope_angle) of loss parameter

sigma = sig_0 + sig_1 * (beta_stable).^2;                                               % calculate loss parameter vector                                             

phi_i = nonzeros((2/sqrt(Lx*Ly)) * sin(mx*xi*pi/Lx) .* sin(my*yi*pi/Ly) .* stab_index); % calculate input modal shape function
phi_o = nonzeros((2/sqrt(Lx*Ly)) * sin(mx*xo*pi/Lx) .* sin(my*yo*pi/Ly) .* stab_index); % calculate output modal shape function

%% NEW PRUNING METHOD

tic
cent_thresh = cent_range * (2^(1/1200)-1);                          % Calculate fractional threshold from user defined cent_range

omega_new = omega;                                                  % Copy omega into new vector to be pruned
for m = 1:length(omega_new)
    if omega_new(m) == 0                                            % if value is 0, skip to next iteration
    else
        if abs(phi_o(m)) < phi_thresh || abs(phi_i(m)) < phi_thresh % if input and output location on mode shape vector is under threshold set sample on omega new to 0     
            omega_new(m) = 0;
        else
            [row] = find(omega>omega(m) - (cent_thresh/2)*omega(m) & omega<omega(m)+ (cent_thresh/2)*omega(m) & omega~=omega(m) & sigma>sigma(m) - (damp_thresh/2)*sigma(m) & sigma<sigma(m)+ (damp_thresh/2)*sigma(m) & sigma~=sigma(m)); % if any values fall between the damping and frequency thresholds of the current sample, set them to 0 
            omega_new(row) = 0;
        end
    end   
end

prunevec = omega_new~=0;                            % create logical vector from omega_new

omega_new = nonzeros(omega_new);                    % remove zeros (pruned modes)

sigma_new = nonzeros(sigma.*prunevec);              % use logical vector to apply pruning to sigma

phi_i_new = nonzeros(phi_i.*prunevec);              % use logical vector to apply pruing to phi_i
phi_o_new = nonzeros(phi_o.*prunevec);              % use logical vector to apply pruing to phi_o

% Initialise p1, p2, y
sp_len = length(omega_new);                         % determine modal vector length
p1 = zeros(sp_len,1);                               % preallocate P^n vector
p2 = zeros(sp_len,1);                               % preallocate P^n-1 vector
in1 = zeros(Nf,1); in1(1) = 1;                      % generate impulse input vector
out1 = zeros(Nf,1);                                 % prealloacate output vector

B = (2 - k^2*omega_new.^2)./(1+sigma_new*k);        % compute B state update vector
C = (sigma_new*k - 1)./(1+sigma_new*k);             % compute C state update vector
D = phi_i_new ./ (1+sigma_new*k);                   % compute D state update vector

for n = 1:Nf
% compute P^n+1 state
p0 = B.*p1 + C.*p2 + D*in1(n);
% update state
p2 = p1;
p1 = p0;
% sum and add to output vector
out1(n)= sum(phi_o_new.*p0);
end