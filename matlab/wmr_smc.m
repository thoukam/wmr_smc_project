function wmr_smc
%% ================== PARAMÈTRES GÉNÉRAUX ==================
Ts      = 0.01;          % pas d'échantillonnage [s]
Tfinal  = 45.0;          % durée simu [s]
ctrl    = 'BSMC';        % 'NSMC' ou 'BSMC'
traj    = 'circle';      % 'circle' ou 'figure8'

% Trajectoire
R     = 1.5;             % rayon (cercle) ou échelle (8)
Omega = 0.25;            % pulsation rad/s
xc    = 0.0; yc = 0.0;   % centre

% Saturations actionneurs
Vmax  = 1.5;             % vitesse linéaire max [m/s]
Wmax  = 1.5;             % vitesse angulaire max [rad/s]

% Gains "table de départ" (article)
a11=15; a12=15; a21=15; a22=15;
p11=5;  p12=9;  p21=7;  p22=13;
q11=9;  q12=5;  q21=9;  q22=5;
k1=10;  k2=15;  epsSig=50;   % épaisseur couche limite (sigmoïde)

% Perturbations (robustesse)
useDist = true;

%% ================== ALLOCATIONS ===========================
N   = round(Tfinal/Ts)+1;
H   = zeros(N, 12);   % [t x y th xr yr thr nu om ex ey et]

%% =========== CONDITIONS INITIALES ALIGNÉES SUR LA REF =====
t0 = 0;
[xr0, yr0, thr0, vr0, omegar0] = ref(traj, t0, xc, yc, R, Omega);

x  = xr0;
y  = yr0;
th = thr0;

prev_thr = thr0;
omega_r  = omegar0;   % pour circle c'est Omega, pour figure8 on recalcule après

%% ================== BOUCLE SIMULATION ====================
for k=1:N
    t = (k-1)*Ts;

    % ----- Référence -----
    [xr,yr,thr,vr,omegar_ref] = ref(traj,t,xc,yc,R,Omega);

    % ----- omega_r -----
    if strcmpi(traj,'circle')
        % pour le cercle on a l'expression exacte
        omega_r = omegar_ref;    % = Omega
    else
        % pour la figure8 on fait une dérivée numérique de theta_r
        if k == 1
            omega_r = 0;
        else
            dth     = angleWrap(thr - prev_thr);
            omega_r = dth / Ts;
        end
    end
    prev_thr = thr;

    % ----- Erreurs locales -----
    ct = cos(th); st = sin(th);
    ex =  (xr - x)*ct + (yr - y)*st;
    ey = -(xr - x)*st + (yr - y)*ct;
    et = angleWrap(thr - th);

    % ----- Contrôleur -----
    switch upper(ctrl)
        case 'NSMC'
            [nu, omega] = ctrlNSMC(ex,ey,et,vr,omega_r, ...
                                   a11,a12,a21,a22, ...
                                   p11,p12,p21,p22, ...
                                   q11,q12,q21,q22, ...
                                   k1,k2,epsSig);
        case 'BSMC'
            [nu, omega] = ctrlBSMC(ex,ey,et,vr,omega_r, ...
                                   a11,a12,a21,a22, ...
                                   p11,p12,p21,p22, ...
                                   q11,q12,q21,q22, ...
                                   epsSig);
        otherwise
            error('ctrl doit être NSMC ou BSMC');
    end

    % ----- Perturbations -----
    if useDist
        D1 = 2.0*sin(t);  % sur nu
        D2 = 1*sin(t);  % sur omega
    else
        D1 = 0; D2 = 0;
    end

    % ----- Saturations -----
    omega = clamp(omega + D2, -Wmax, Wmax);
    nu    = clamp(nu    + D1, -Vmax, Vmax);

    % ----- Intégration cinématique -----
    x  = x  + Ts*nu*cos(th);
    y  = y  + Ts*nu*sin(th);
    th = angleWrap(th + Ts*omega);

    % ----- Log -----
    H(k,:) = [t x y th xr yr thr nu omega ex ey et];
end

%% ================== TRACÉS ================================
t = H(:,1);
x = H(:,2);  y = H(:,3);
xr= H(:,5);  yr= H(:,6);
nu= H(:,8);  om= H(:,9);
ex= H(:,10); ey= H(:,11); et= H(:,12);

% Créer le dossier figures s'il n'existe pas
if ~exist('figures', 'dir')
    mkdir('figures');
end

figure; hold on; axis equal; grid on;
plot(xr,yr,'k--','LineWidth',1.5); 
plot(x ,y ,'LineWidth',1.8);
legend('référence','robot','Location','best');
title(sprintf('Trajectoire (%s) avec %s', traj, ctrl));
xlabel('x [m]'); ylabel('y [m]');
saveas(gcf, sprintf('figures/trajectoire_%s_%s.png', traj, ctrl));

figure; grid on; 
plot(t, ex, t, ey, t, et,'LineWidth',1.5);
legend('e_x','e_y','e_\theta','Location','best'); 
xlabel('t [s]'); ylabel('erreurs');
title('Erreurs locales');
saveas(gcf, sprintf('figures/erreurs_%s_%s.png', traj, ctrl));

figure; grid on; 
plot(t, nu, t, om,'LineWidth',1.5);
legend('\nu','\omega','Location','best');
xlabel('t [s]'); ylabel('commande');
title('Entrées de commande');
saveas(gcf, sprintf('figures/commandes_%s_%s.png', traj, ctrl));

% Petit bilan
rmse_ex = sqrt(mean(ex.^2)); rmse_ey = sqrt(mean(ey.^2));
fprintf('RMSE ex = %.3f m, ey = %.3f m sur %gs (%s, %s)\n', ...
         rmse_ex, rmse_ey, Tfinal, ctrl, traj);
fprintf('Figures sauvegardées dans le dossier figures/\n');
end

%% ================== CONTRÔLEURS ==========================
function [nu, omega] = ctrlNSMC(ex,ey,et,vr,omegar, ...
                                a11,a12,a21,a22, ...
                                p11,p12,p21,p22, ...
                                q11,q12,q21,q22, ...
                                k1,k2,epsSig)
    % Surfaces NSMC
    s1 = et;
    s2 = k1*ex - k2*omegar*ey;

    sig1 = sigmoid(s1, epsSig);
    sig2 = sigmoid(s2, epsSig);

    omega = omegar + a11*abs(s1)^(p11/q11)*sig1 + a12*abs(s1)^(p12/q12)*sig1;
    nu    = ey*omegar + vr + (1/k1)*( k2*(omegar^2)*ex ...
             + a21*abs(s2)^(p21/q21)*sig2 + a22*abs(s2)^(p22/q22)*sig2 );
end

function [nu, omega] = ctrlBSMC(ex,ey,et,vr,omegar, ...
                                a11,a12,a21,a22, ...
                                p11,p12,p21,p22, ...
                                q11,q12,q21,q22, ...
                                epsSig)
    % Surfaces BSMC
    s1 = ex;
    psi = atan(vr * ey);  % psi = atan( vr * y_e )
    s2 = et + psi;

    % dérivées de psi
    dpsi_dye = vr/(1 + (vr*ey)^2);
    dpsi_dvr = ey/(1 + (vr*ey)^2);

    dvr = 0;              % approx d(vr)/dt

    sig1 = sigmoid(s1, epsSig);
    sig2 = sigmoid(s2, epsSig);

    nu    = ey*(omegar) + vr*cos(et) ...
          + a11*abs(s1)^(p11/q11)*sig1 + a12*abs(s1)^(p12/q12)*sig1;

    omega = omegar + dpsi_dye*vr*sin(et) + dpsi_dvr*dvr ...
          + a21*abs(s2)^(p21/q21)*sig2 + a22*abs(s2)^(p22/q22)*sig2;
end

%% ================== TRAJECTOIRES & OUTILS ================
function [xr,yr,thr,vr,omegar] = ref(traj,t,xc,yc,R,Omega)
    switch lower(traj)
        case 'circle'
            xr = xc + R*cos(Omega*t);
            yr = yc + R*sin(Omega*t);
            dx = -R*Omega*sin(Omega*t);
            dy =  R*Omega*cos(Omega*t);
            thr = atan2(dy,dx);
            vr  = hypot(dx,dy);     % = R*Omega
            omegar = Omega;         % exact
        case 'figure8'   % lemniscate de Gerono
            xr = xc + R*sin(Omega*t);
            yr = yc + R*sin(Omega*t).*cos(Omega*t);
            dx =  R*Omega*cos(Omega*t);
            dy =  R*Omega*(cos(2*Omega*t));
            thr = atan2(dy,dx);
            vr  = hypot(dx,dy);
            omegar = NaN;           % on ne l'utilise pas directement
        otherwise
            error('traj doit être circle ou figure8');
    end
end

function y = sigmoid(x, epsSig)
    y = (1 - exp(-epsSig*x)) ./ (1 + exp(-epsSig*x));
end

function z = angleWrap(a)
    z = mod(a+pi, 2*pi) - pi;
end

function y = clamp(x, xmin, xmax)
    y = min(max(x, xmin), xmax);
end