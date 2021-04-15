%/*
%****************************************************************************
%		import_figure.m
%****************************************************************************
% Importation des données d'une figure du programme d'identification Sysquake
% dans MATLAB.
% Tracage et sauvegarde sur disque de l'image
% Le fichier d'entrée est un fichier .dat ASCII exporté par le programme
% d'identification Sysquake
%
%****************************************************************************
%****************************************************************************
% Version Date     Auteur Motif
% 1.0     28/10/03 SCD    Creation
% 2.0	  05/09/05 SCD    adaptation pour passer du format d'entrée .mat
%                         à .dat (ascii). format .mat de matlab plus
%                         utilisé.
%                         identification.sq n'utilise plus les extensions
%                         matlab (=> exportation en ascii) dès la version
%                         1.03
%****************************************************************************
%*/

clear all; close all; format compact;

LABEL_REP_TEMP    = 'reponse temporelle';
LABEL_REP_FREQ    = 'reponse frequentielle';
LABEL_FFT_SIGNAUX = 'fft signaux';

%dialogue d'ouverture du fichier
[filename, pathname, filterindex] = uigetfile({'*.dat','fichier dat (*.dat)'},'Ouvrir un fichier .dat');

if filename ~= 0 %un fichier valide a été choisi...
    %chargement du fichier des traces temporelles
    fichierAOuvrir = [pathname,filename];

    %Ouvre une 1ère fois le fichier pour en extraire les commentaires de
    %début (contiennent le type de données exportées).
    fid = fopen(fichierAOuvrir, 'r');
    for i = 1:4     %c'est la 4ème ligne qui contient le commentaire intéressant
        dataDescr = fgetl(fid);
    end;
    fclose(fid);

    dataDescr = dataDescr(2:end);       % enlève le '%' de début de ligne
    dataDescr = lower(dataDescr);       % met tout en minuscule
    %dataDescr = strtrim(dataDescr);     % enlève espaces de début et fin éventuels
    %MATLAB 6.5 n'a pas la fonction strtrim...    
    while isspace(dataDescr(1))
        dataDescr = dataDescr(2:end);   % enlève l'espace de début
    end;
    deblank(dataDescr);    
    
    %format reconnu?
    if (~strcmpi(dataDescr, LABEL_REP_TEMP) &&...
        ~strcmpi(dataDescr, LABEL_REP_FREQ) &&...
        ~strcmpi(dataDescr, LABEL_FFT_SIGNAUX))
        dataDescr = 'non reconnu';
    end;

    disp(sprintf('\n *** fichier ouvert %s : %s ***', fichierAOuvrir, dataDescr));
    if strcmp(dataDescr,'non reconnu')
        error('Format de fichier non reconnu ! Provient-il de identification.sq ?'); %quitte la fonction si fichier ouvert non reconnu
    end


    %Ouvre une 2ème fois le fichier et charge données
    data = load(fichierAOuvrir);

    %plot
    figure(1);

    switch dataDescr %trace la figure appropriée aux données chargées
        case LABEL_REP_TEMP,

            %extrait données
            tMes = data(:,1)';
            uMes = data(:,2)';
            yMes = data(:,3)';
            tMod = data(:,4)';
            yMod = data(:,5)';

            % toutes les courbes sont vides?
            emptyTrace =    length(find(isnan(uMes))) == length(uMes) && ...
                            length(find(isnan(yMes))) == length(yMes) && ...
                            length(find(isnan(yMod))) == length(yMod);

            if ~emptyTrace  %au moins une courbe à tracer ?
                %appond données pour tracage
                x = [tMes', tMes', tMod'];
                y = [uMes', yMes', yMod'];
                %tracage
                plot(x,y);
                title ('Réponse temporelle');
                xlabel('t [s]');
                setwidth(1);
                grid on;
            end;

        case LABEL_REP_FREQ,

            %extrait données
            fMes = data(:,1)';
            absGaMes = data(:,2)';
            angGaMes = data(:,3)';
            absGaMod = data(:,4)';
            angGaMod = data(:,5)';
            absGaModAsymp = data(:,6)';
            angGaModAsymp = data(:,7)';

            wMes = 2*pi*fMes;

            % toutes les courbes sont vides?
            emptyTrace =    length(find(isnan(absGaMes))) == length(absGaMes) && ...
                            length(find(isnan(absGaMod))) == length(absGaMod) && ...
                            length(find(isnan(absGaModAsymp))) == length(absGaModAsymp);

            if ~emptyTrace  %au moins une courbe à tracer ?
                %appond données pour tracage
                x = [wMes', wMes', wMes'];
                yAmp = [absGaMes', absGaMod', absGaModAsymp'];
                yPha = [angGaMes', angGaMod', angGaModAsymp'];
                %tracage
                bode_aff(yAmp,yPha,x,'Bode d''amplitude');
            end;

        case LABEL_FFT_SIGNAUX,

            %extrait données
            fMes = data(:,1)';
            absFftUMes = data(:,2)';
            angFftUMes = data(:,3)';
            absFftYMes = data(:,4)';
            angFftYMes = data(:,5)';
            absFftYMod = data(:,6)';
            angFftYMod = data(:,7)';

            wMes = 2*pi*fMes;

            % toutes les courbes sont vides?
            emptyTrace =    length(find(isnan(absFftUMes))) == length(absFftUMes) && ...
                            length(find(isnan(absFftYMes))) == length(absFftYMes) && ...
                            length(find(isnan(absFftYMod))) == length(absFftYMod);

            if ~emptyTrace  %au moins une courbe à tracer ?
                %appond données pour tracage
                x = [wMes', wMes', wMes'];
                yAmp = [absFftUMes', absFftYMes', absFftYMod'];
                yPha = [angFftUMes', angFftYMes', angFftYMod'];
                %tracage
                bode_aff(yAmp,yPha,x,'Bode d''amplitude');
            end;

        %OTHERWISE,
    end


    if ~emptyTrace
        %sauve la figure avec le meme nom de fichier (des préfixes et suffixes sont ajoutés par fig_save)
        [PATHSTR,NAME,EXT,VERSN] = FILEPARTS(fichierAOuvrir); %extrait noms de fichier, extension, path
        fig_save(gcf,NAME,[PATHSTR,filesep]); %sauve les figures à la meme place que le MAT-file ouvert
        msg = sprintf('\n *** images sauvegardées dans %s ***',PATHSTR);
    else
        msg = sprintf('\n *** aucune courbe à tracer ! ; seules les courbes affichées dans SysQuake sont exportées ***');
    end;

    disp (msg);

end

