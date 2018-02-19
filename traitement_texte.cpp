#include <akaze_detect.hpp>
#include <tcp.hpp>
#include <traitement_texte.hpp>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include <string>
#include <cstdint>
#include <time.h>
#include <curses.h> // entrée clavier

using namespace std;
using namespace cv;

extern bool arret_demande;
extern double vitesse,acceleration,deceleration;
extern bool fin_prog;
extern double x,y; // coordonnée de la pièce
extern bool detection;
extern int cobot_x, cobot_y, cobot_z;

int emplacement_piece_x;
int emplacement_piece_y;
int emplacement_piece_z;

/************************************************************************************/
/* fonction permettant d'utiliser le switch case avec des comparaison de string     */
/************************************************************************************/
namespace fnv
{
	constexpr uint64_t _(uint64_t h, const char* s)
	{
    	return (*s == 0) ? h :
     	_((h * 1099511628211ull) ^ static_cast<uint64_t>(*s), s+1);
  	}
}

constexpr uint64_t _(const char* s)
{
	return fnv::_(14695981039346656037ull, s);
}

uint64_t _(const std::string& s)
{
	return fnv::_(14695981039346656037ull, s.data());
}

/***********************************************************************************/

int affichage_contenu_programme()
{
    ifstream fichier("contenu_programme.txt", ios::in);  // on ouvre le fichier en lecture
 	
 	printf("\ncontenu du programme choisi : \n");
    if(fichier)  // si l'ouverture a fonctionné
    {
		string ligne;
		while(getline(fichier, ligne))  // tant qu'il y a des lignes
		{
			cout << ligne << endl;  // on l'affiche
		}
        fichier.close(); // on referme le fichier
    }
    else
    {
        cerr << "Impossible d'ouvrir le fichier !" << endl;	
	}
    return 0;
}

int execution_programme()
{
	system("clear");
	ifstream fichier("contenu_programme.txt", ios::in);  // on ouvre en lecture
	std::string mot_clef,ligne;
	double valeur[4];
	int num_ligne,ligne_octet,ligne_recal;
	int stock_ligne[1000]; // permet de stocker l'emplacement de chaque ligne
	int tolerance=10; // tolerance pour l'emplacement originel de la pièce
	bool first_mvtPAP = true; // permet de conaitre si nous lisons pour la première fois un mvt PAP -> coordonnée de la pièce
	bool fin_cycle = false; // permet de savoir si le cycle est terminé
	int NbCycles = 0; // variable pour le comptage de nb ce cycle effectué
	time_t start_time = 0;
    time_t current_time = 0;
	std::istringstream isstream;
	
	if(fichier)  // si l'ouverture a fonctionné
    {
        fichier.seekg(0, ios::beg); // on se place au debut du texte
		while(fichier.eof()==false && fin_cycle == false && fin_prog == false)
		{
        	getline(fichier, ligne);
        	isstream.str(ligne);
        	ligne_octet=fichier.tellg();
			getline(isstream, mot_clef,' ');
			
			if(_(mot_clef)==_("N")) // permet de mémoriser le numero des lignes dans le cas d'un Go To
			{
				getline(isstream, mot_clef,' ');
				num_ligne = atoi(mot_clef.c_str());
				stock_ligne[num_ligne]=ligne_octet;
				getline(isstream, mot_clef,' ');
			}
		
			switch(_(mot_clef))
			{
				case _("Vitesse"):
					getline(isstream, mot_clef, ' ');
					valeur[0]=atof(mot_clef.c_str());
					vitesse = 15 * valeur[0];
					if (vitesse > 2000){vitesse = 2000;} // on veille à ne pas depasser le max constructeur
					printf("\nvitesse %f \n",vitesse);
					break;
				
				case _("Mvt_point"):
					printf("\nMvt_point\n");
					for(int i=0;i<4;i++)
					{
						getline(isstream, mot_clef, ' ');
						valeur[i]=atof(mot_clef.c_str());
					}
					
					/* si c'est la première fois que l'on a un mouvement PAP alors on stock les coordonnées x,y,z de la pièce afin d'effectuer le recalage lorsque l'on aura ces cooordonnées lors d'une demande d'un prochain mouvement */
					if(first_mvtPAP == true) 
					{
						emplacement_piece_x=valeur[0];
						emplacement_piece_y=valeur[1];
						emplacement_piece_z=valeur[2];	
					}
					
					first_mvtPAP=false;
					printf(" %d / %d / %d \n",emplacement_piece_x,emplacement_piece_y,emplacement_piece_z);
					
					if(abs(emplacement_piece_x-valeur[0])<=abs(tolerance) && abs(emplacement_piece_y-valeur[1])<=abs(tolerance) && abs(emplacement_piece_z-valeur[2])<=abs(tolerance))
					{
						while(detection==false) // tant qu'il n'y a pas de pièce detectée on relance la detection
						{
							for(int i=0;i<10;i++) // on analyse l'environement pendant quelque cycle
							{
								detection_akaze();
								demande_position();
								waitKey(50);
								cout<<i<<endl;
							}
						}
						close_windows();
						envoie_tram_deplacement(acceleration,vitesse,deceleration,x,y,cobot_z,valeur[3]); // on vient au dessus de la pièce
						envoie_tram_deplacement(acceleration,vitesse,deceleration,x,y,valeur[2],valeur[3]); // on vient en contact sur la pièce
						printf("**************** recalage done pour se placer au dessus de la pièce! *****************\n");
					}
					
					else
					{
						envoie_tram_deplacement(acceleration,vitesse,deceleration,valeur[0],valeur[1],valeur[2],valeur[3]);	
					}
					detection=false;
					break;
				
				case _("Index1"):
					printf("\nindex1\n");
				/*	
					firstLine_index_1 = ligne;
					getline(fichier, ligne);
					secondLine_index_1 = ligne;
					getline(fichier, ligne);
				      	thirdLine_index_1 = ligne;
				      	getline(fichier, ligne);
				      	forthLine_index_1= ligne;
				      	getline(fichier, ligne);
					fifthLine_index_1 = ligne;
				*/	
					break;
				
				case _("Index2"):
					printf("\nindex2\n");
				/*					
					firstLine_index_2 = ligne;
					getline(fichier, ligne);
					secondLine_index_2 = ligne;
					getline(fichier, ligne);
				      	thirdLine_index_2 = ligne;
				      	getline(fichier, ligne);
				      	forthLine_index_2= ligne;
				      	getline(fichier, ligne);
					fifthLine_index_2 = ligne;
				*/	
					break;
	
				case _("Mvt_point_index"):
					break;
				
				case _("Attente_temps"):
					getline(isstream, mot_clef, ' ');
					valeur[0]=atof(mot_clef.c_str());
					printf("\nattente_temps de %f secondes\n",valeur[0]);
					start_time = time(NULL);
					while(current_time-start_time+1<=valeur[0])// attente en seconde
					{
						current_time = time(NULL);
						demande_position(); // permet de garder la connexion active même pendant le temps d'attente
						cout<<"demande pos"<<endl;
						waitKey(500); // pause de 0.5s
					}
					break;
					
				case _("Mvt_relatif"):
					printf("\nMvt_relatif\n");
					/* /!\ le cobot n'est pas encore opérationnel pour recevoir cet ordre */
					//envoie_tram_mouvement_relatif(direction,vitesse,distance);
					break;
					
				case _("End"):
					printf("\nEnd\n");
					break;
					
				case _("Retour_depart"):
					printf("\nRetour_depart\n");
					fichier.clear();
					fichier.seekg(0, ios::beg);
					break;
					
				case _("Retour_Cycle"):
					getline(isstream, mot_clef, ' ');
					valeur[0]=atoi(mot_clef.c_str());
					NbCycles++;
					if(NbCycles > valeur[0])
					{
						fin_cycle = true;
					}
					printf("\nRetour_debut_cycles\n");
					fichier.clear();
					fichier.seekg(0, ios::beg);
					break;
				
				case _("Go_To"):
					getline(isstream, mot_clef, ' ');
					valeur[0]=atoi(mot_clef.c_str());
					fichier.clear();
					fichier.seekg(stock_ligne[(int)(valeur[0]-1)], ios::beg);
					printf("\nGo to ligne %d\n",(int)(valeur[0]));
					break;
					
				case _("Fermer_pince"):
					printf("\nfermer_pince\n");
					//on ferme la pince
					break;
					
				case _("Ouvrir_pince"):
					printf("\nouvrir_pince\n");
					//on ouvre la pince
					break;
				case _("Descente"):
					printf("\ndescente\n");
					//descente jusqu'a l'objet
					break;
					
				case _("Attente_entree"):
					printf("\nattente_entree\n");
					//attendre le passage de l'entre x à 0 ou 1
					break;
				
					
				case _("Action_sortie"):
					printf("\nAction_sortie\n");
					//on met la sortie x à 0 ou 1
					break;
				
				case _("\n"):
					printf("\nfini\n");
					break;
						
				default:
				printf("\ndefault\n");
				
				break;
			}
			
			isstream.clear();
			waitKey(100); // pause de 100ms
		}	
		fichier.close();
	}
	else
	{
		printf("error de lecture du fichier");
	}
	cout<<"fini"<<endl;
	return 0;
}
