#ifndef _TCP_H
#define _TCP_H

#include <iostream>

int envoie_tram_deplacement(int tVarAcc,int tVarVit,int tVarDec,int tVarPosX,int tVarPosY,int tVarPosZ,int tVarPosU);

int envoie_tram_mouvement_relatif(int direction,int vitesse,int distance);

int balayage_zone(void); 

void demande_position(void);

int tcp_connect(void);

int tcp_close(void);

int demande_liste_prog(void);

int demande_contenu_prog(std::string nom_programme);

int demande_arret(void);

int demande_marche(void);

int moteur_ready(void);

#endif
