/**
 * \file	MET_etudiant.c
 * \brief	Fonction de r�gulation �crite par les �tudiants dont le .hex va �tre charg�
 * \author	Michel Girardin
 * \version	0
 * \date	12.03.2021
 */ 
 // test om
//** D�finitions
#define     RTPWATCH_MODE_NONE                   	1		/*!< Aucun mode n'as �t� s�lectionn� dans RTPWatch */
#define     RTPWATCH_MODE_IDENT_BCL_INTERNE      	2		/*!< Identification de la boucle de courant */
#define     RTPWATCH_MODE_REG_BCL_INTERNE        	3		/*!< Regulation de la bouclde de courant */
#define     RTPWATCH_MODE_IDENT_BCL_EXT          	4		/*!< Identification de la boucle position/vitesse */
#define     RTPWATCH_MODE_REG_BCL_EXT        		5		/*!< Regulation de la boucle position/vitesse */
#define     RTPWATCH_MODE_REG_BCL_EXT_PID          	6		/*!< Regulation de la boucle de position/vitesse par un PID */

//** D�finitions de types
#define UINT32_WATCH(v)     (UpdateWatch(#v,(Uint32*)(&(v)),VAR_INT))
#define FLOAT_WATCH(v)      (UpdateWatch(#v,(Uint32*)(&(v)),VAR_FLOAT))

#define 	Fe										12000.0	// Fr�quence d'�chantillonnage
#define 	Fe2										6000.0	// Fr�quence d'�chantillonnage pour le resolver
#define 	h										83.3e-6	// P�riode d'�chantillonnage

#define		Umax									120.0	// Tension maximale
#define		Imax									5.0		// Courant maximal
#define		Wmax									300.0   // Vitesse maximale
#define 	NbrePairePole							3.0		// Nombre de paires de p�les

#define		PI										3.1416
#define 	PI_SUR_2								1.5708
#define		TWO_PI									6.2832
#define		Sqrt3									1.73205
#define		Div_sqrt3								0.57735

#define		INCREMENTAL								0		// Capteur relatif du moteur DC -> 12kHz
#define		RESOLVER								1		// Capteur absolu du moteur synchrone -> 6kHz


//** Macros

//** Variables Globales		/!\ toute initialisation est perdue! utiliser InitialisationEtudiant()
//								(idem pour les variables statiques)

long int nombreTours;

float flagAntiWindup;

// Courant
float Kp_i, Gi_i;
// vitesse
float Kp_w, Gi_w;
// position
float Kp_theta;

float thetaMoteur_old, thetaElectrique, thetaMecanique;
float sinThetaElectrique, cosThetaElectrique;

float udc, uqc;
float idm, iqm;
float uqcp, uqc;
float udcp, udp;
float int_iq, int_id;
float iqc, idc;
float Kpeiq,Kpeid;


/**
 *	\fn 	void InitialisationEtudiant()
 * 	\brief	Cette fonction est appel�e apr�s le chargement du .hex
 */
void InitialisationEtudiant()
{
	FLOAT_WATCH(Kp_i);
	FLOAT_WATCH(Gi_i);
	FLOAT_WATCH(Kp_w);
	FLOAT_WATCH(Gi_w);
	FLOAT_WATCH(Kp_theta);
	FLOAT_WATCH(thetaMecanique);
	FLOAT_WATCH(thetaElectrique);
	FLOAT_WATCH(idm);
	FLOAT_WATCH(iqm);
	FLOAT_WATCH(flagAntiWindup);
	FLOAT_WATCH(uqc);
	FLOAT_WATCH(uqcp);
	FLOAT_WATCH(udc);
	FLOAT_WATCH(udcp);
	uqc=0;
	uqcp=0;
	udc=0;
	udcp=0;
	int_iq=0;
	int_id=0;

	nombreTours = 0;
	thetaMoteur_old = 0.0;
	// courant
	Kp_i = 19.26;//11.41;//19.26;
	Gi_i = 1/0.001896;//1/0.00253;//1/0.001896;
	// vitesse
	Kp_w = .0;
	Gi_w = .0;
	// position
	Kp_theta = .0;
	// Calcul de la vitesse avec d�riv�e + filtre
	thetaMecanique = 0.0;
	thetaElectrique = 0.0;
	sinThetaElectrique = 0.0;
	cosThetaElectrique = 0.0;

	flagAntiWindup=1;
}


/**
 *	\fn 	
 * 	\brief	Cette fonction est appel�e pour r�gler le moteur
 *
 *	\param	flags 			: regroupe diff�rentes informations:
 *								- flags.bit.Mode 			mode de travail de RTPWatch
 *								- flags.bit.FirstMesure		indique la premi�re mesure apr�s un reset
 *	\param	consigne		: consigne envoy�e par RTPWatch
 *	\param	courantBranche1	: courant du moteur (branche 1)
 *	\param	courantBranche2	: courant du moteur (branche 2)
 *	\param	thetaMoteur		: position du moteur
 *	\param	busDC			: tension du bus DC
 * 
 *  \return	tension � appliquer
 */
RapportsCycliques Regulation(FlagsRegulation* flags, float consigne,float courantBranche1, float courantBranche2, float thetaMoteur, float busDC)
{
	RapportsCycliques rapportsCycliquesVariateur;
	float divBusDC;
	float delta_theta;
	float h1, h2, Dx, Dy, CMPR1, CMPR2, CMPR3;
	float usa, usb,isa,isb;
	
	/*------------------------------------------------------------------------------------------**
	**					Traitement de la mesure de position										**
	**------------------------------------------------------------------------------------------*/
	// Une nouvelle mesure de position est disponible une fois sur deux dans le cas du resolver
	if(flags->bit.FirstMesure)									// set la premi�re mesure comme condition initiale
		thetaMoteur_old = thetaMoteur;
	if(flags->bit.NewPosition && flags->bit.ChoixCapteur == RESOLVER)
	{
		delta_theta = thetaMoteur - thetaMoteur_old;
		if(delta_theta < -PI)
		{
			nombreTours++;
		}
		if(delta_theta > PI)
		{
			nombreTours--;
		}
		thetaMecanique = nombreTours * TWO_PI + thetaMoteur;
		thetaElectrique = thetaMoteur * NbrePairePole;
		thetaMoteur_old = thetaMoteur;
		
		// calul le ssinus et cosinus de th�taElectrique
		sincos(thetaElectrique, &sinThetaElectrique, &cosThetaElectrique);
	}
	
	
	/*------------------------------------------------------------------------------------------**
	**							Transformation de Park											**
	**							-> idm, iqm														**
	**------------------------------------------------------------------------------------------*/


	isa=courantBranche1;
	isb=Div_sqrt3*(2*courantBranche2+courantBranche1);
	
	idm=cosThetaElectrique*isa + sinThetaElectrique*isb;
	
	iqm=-sinThetaElectrique*isa + cosThetaElectrique*isb;
	
	/*------------------------------------------------------------------------------------------**
	**							R�gulation (Position, Vitesse, Courant)							**
	**							-> udc, uqc														**
	**------------------------------------------------------------------------------------------*/

	switch(flags->bit.Mode)
	{
		case RTPWATCH_MODE_IDENT_BCL_INTERNE:
		{
			udc = 0.0;
			uqc = consigne;
		}
		break;

		case RTPWATCH_MODE_REG_BCL_INTERNE:
		{
		    idc=0.0;
		    iqc=consigne;

		    //Axe Q: Calcul de UQC
		    Kpeiq=(iqc-iqm)*Kp_i;
			int_iq=int_iq+(Gi_i*h)*(Kpeiq-((uqcp-uqc)*flagAntiWindup));
			uqcp=Kpeiq+int_iq;

			if (uqcp>120)
			{
			    uqc=120;
			}
			else if (uqcp<-120)
			{
			    uqc=-120;
			}
			else
			{
				uqc=uqcp;
			}
			//Axe D: Calcul de UDC
			Kpeid=(idc-idm)*Kp_i;
			int_id=int_id+(Gi_i*h)*(Kpeid-((udcp-udc)*flagAntiWindup));
			udcp=Kpeid+int_id;

			if (udcp>120)
			{
			    udc=120;
			}
			else if (udcp<-120)
			{
			    udc=-120;
			}
			else
			{
			    udc=udcp;
			}


		}
		break;

		case RTPWATCH_MODE_IDENT_BCL_EXT:
		{
		}
		break;

		case RTPWATCH_MODE_REG_BCL_EXT:
		{
		}
		break;

		default:
		{
		udc = 0.0;
		uqc = 0.0;
		}
	}

	
	/*------------------------------------------------------------------------------------------**
	**							Transformation de Park inverse									**
	**							-> usa, usb														**
	**------------------------------------------------------------------------------------------*/

	usa = cosThetaElectrique*udc - sinThetaElectrique*uqc ;
	usb = sinThetaElectrique*udc + cosThetaElectrique*uqc ;
	
	/*------------------------------------------------------------------------------------------**
	**							Calculs des rapports cycliques									**
	**							-> raportsCycliquesVariateur.branche1							**
	**							-> raportsCycliquesVariateur.branche2							**
	**							-> raportsCycliquesVariateur.branche3							**
	**------------------------------------------------------------------------------------------*/
	divBusDC = 1.0/busDC;
	h1 = 1.5*usa*divBusDC;
	h2 = 0.5*Sqrt3*usb*divBusDC;
	if (h2 >= 0.0)
	{
		if (h2 < h1)				// SECTEUR 1
		{
			Dx = h1-h2;
			Dy = 2.0*h2;
			CMPR1 = 0.5*(1.0-Dx-Dy);
			CMPR2 = CMPR1+Dx;
			CMPR3 = 1.0-CMPR1;
			rapportsCycliquesVariateur.branche1 = 1.0-CMPR1;
			rapportsCycliquesVariateur.branche2 = 1.0-CMPR2;
			rapportsCycliquesVariateur.branche3 = 1.0-CMPR3;
		}
		else
		{
			if (h2 > -h1)			// SECTEUR 2
			{
				Dx = h1+h2;
				Dy = -h1+h2;
				CMPR1 = 0.5*(1.0-Dx-Dy);
				CMPR2 = CMPR1+Dy;
				CMPR3 = 1.0-CMPR1;
				rapportsCycliquesVariateur.branche1 = 1.0-CMPR2;
				rapportsCycliquesVariateur.branche2 = 1.0-CMPR1;
				rapportsCycliquesVariateur.branche3 = 1.0-CMPR3;
			}
			else					// SECTEUR 3
			{
				Dx = 2.0*h2;
				Dy = -h1-h2;
				CMPR1 = 0.5*(1.0-Dx-Dy);
				CMPR2 = CMPR1+Dx;
				CMPR3 = 1.0-CMPR1;
				rapportsCycliquesVariateur.branche1 = 1.0-CMPR3;
				rapportsCycliquesVariateur.branche2 = 1.0-CMPR1;
				rapportsCycliquesVariateur.branche3 = 1.0-CMPR2;
			}
		}
	}
	else
	{
		if (h2 < -h1)
		{
			if (h2 > h1)			// SECTEUR 4
			{
				Dx = -h1+h2;
				Dy = -2.0*h2;
				CMPR1 = 0.5*(1.0-Dx-Dy);
				CMPR2 = CMPR1+Dy;
				CMPR3 = 1.0-CMPR1;
				rapportsCycliquesVariateur.branche1 = 1.0-CMPR3;
				rapportsCycliquesVariateur.branche2 = 1.0-CMPR2;
				rapportsCycliquesVariateur.branche3 = 1.0-CMPR1;
			}
			else					// SECTEUR 5
			{
				Dx = -h1-h2;
				Dy = h1-h2;
				CMPR1 = 0.5*(1.0-Dx-Dy);
				CMPR2 = CMPR1+Dx;
				CMPR3 = 1.0-CMPR1;
				rapportsCycliquesVariateur.branche1 = 1.0-CMPR2;
				rapportsCycliquesVariateur.branche2 = 1.0-CMPR3;
				rapportsCycliquesVariateur.branche3 = 1.0-CMPR1;
			}
		}
		else						// SECTEUR 6
		{
			Dx = -2.0*h2;
			Dy = h1+h2;
			CMPR1 = 0.5*(1.0-Dx-Dy);
			CMPR2 = CMPR1+Dy;
			CMPR3 = 1.0-CMPR1;
			rapportsCycliquesVariateur.branche1 = 1.0-CMPR1;
			rapportsCycliquesVariateur.branche2 = 1.0-CMPR3;
			rapportsCycliquesVariateur.branche3 = 1.0-CMPR2;
		}
	}
	// Actualisation depuis RTPWatch
	flags->bit.ChoixCapteur = RESOLVER;

	return rapportsCycliquesVariateur;
}
 
//** end of file **//
