function [N, wc, Gain, phase, Kpi, Tii, Es] = regul_pi(Gai, Kai, TpE, mphi, w, Nmin, Nmax, Ncourbes)
% Régulateur PI en fonction de N

for i = 1:1:Ncourbes
    N(i)     = Nmin + (i-1)/(Ncourbes-1) * (Nmax-Nmin); % N de Nmin à Nmax
    Tii(i)   = N(i)*TpE;                                % Constante de temps régulateur PI
    Gri      = tf(1*[Tii(i) 1], [Tii(i) 0]);            % Régulateur PI non ajusté
    Goi      = Gri*Gai;                                 % Système en boucle ouverte (non ajusté)
    [A, phi] = bode(Goi, w);                            % Bode du système en b.o
    A        = squeeze(A);
    phi      = squeeze(phi);
    indice   = find(phi>(-180+mphi),1,'last');          
    Kpi(i)   = 1/A(indice);                             % Gain du régulateur PI pour avoir la marge de phase spécifiée
    
    
    Gri         = tf(Kpi(i)*[Tii(i) 1], [Tii(i) 0]);    % Régulateur PI ajusté
    Goi         = Gri*Gai;                              % Système en boucle ouverte (ajusté)
    [A, phi]    = bode(Goi, w);                         % Bode du système en b.o
    Gain(i, :)  = 20*log10(squeeze(A));                 % Gain [db]
    phase(i, :) = squeeze(phi);                         % phase [°]
    
    GBO         = Kai*Kpi(i)/Tii(i);                    % Gain boucle ouverte
    Es(i)       = 1/(1+GBO);                            % Erreur statique
    wc(i)       = w(indice);                            % Vecteur de pulsation
end

end

