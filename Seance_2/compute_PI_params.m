function [Kpi, Tii] = compute_PI_params(Gai, TpE, N, mphi)

Tii      = N*TpE;                               % Constante de temps régulateur PI
Gri      = tf(1*[Tii 1], [Tii 0]);              % Régulateur PI non ajusté
Goi      = Gri*Gai;                             % Système en boucle ouverte (non ajusté)
[A, phi] = bode(Goi);                           % Bode du système en b.o (w enlevé)
A        = squeeze(A);
phi      = squeeze(phi);
indice   = find(phi>(-180+mphi),1,'last');          
Kpi      = 1/A(indice);                         % Gain calculé
end

