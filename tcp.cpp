#include <stdio.h>
#include <string.h>    //strlen

#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <netinet/tcp.h>

#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <netdb.h>
#include <math.h>

#include <gtkmm/comboboxtext.h>

#include <opencv2/opencv.hpp>

#include <akaze_detect.hpp>
#include <tcp.hpp>
#include <traitement_texte.hpp>

using namespace std;
using namespace cv;

typedef unsigned char byte;

/**************************************/
/* declaration des variables          */
/**************************************/

extern double vitesse,acceleration,deceleration;
extern Gtk::ComboBoxText listeProgramme; 

int cobot_x, cobot_y, cobot_z; // variables contenant les coordonnées réel du cobot

const int taille_tram_reponse_longue=10000; 
const int taille_tram_reponse_courte=64;
const int taille_tram_envoie=64;

int socket_desc; //socket pour l'envoie

string programme[100]; // tableau contenant le nom des programmes disponible
byte OutBuffer[taille_tram_envoie]; //tableau de trame a envoyer pour le pilotage moteur
byte server_reply[3000];  //tableau pour stocker la réponse de la rasp

/**************************************/
/*fin  declaration des variables      */
/**************************************/


int envoie_tram_deplacement(int tVarAcc,int tVarVit,int tVarDec,int tVarPosX,int tVarPosY,int tVarPosZ,int tVarPosU)
{
	printf("Accel = %d ,vitesse = %d ,decel = %d ,posX =  %d ,posY = %d ,posZ = %d ,posU =  %d\n",tVarAcc, tVarVit,tVarDec,tVarPosX,tVarPosY,tVarPosZ, tVarPosU);

	int consigne_x=tVarPosX;
	int consigne_y=tVarPosY;
	int consigne_z=tVarPosZ;
	int consigne_u=tVarPosU;

	tVarPosX*=1000;
	tVarPosY*=1000;
	tVarPosZ*=1000;
	tVarPosU*=1000;
	OutBuffer[0] = 0xAA;
	OutBuffer[1] = 0xAA;
	OutBuffer[2] = 0x79; //Adresse Rpi
	OutBuffer[3] = 0x10; //aller en X Y Z U
	//octets acceleration
	OutBuffer[4] = (byte)(tVarAcc & 0xFF);
	tVarAcc >>= 8;
	OutBuffer[5] = (byte)(tVarAcc);
	//octets Vitesse
	OutBuffer[6] = (byte)(tVarVit & 0xFF);
	tVarVit >>= 8;
	OutBuffer[7] = (byte)(tVarVit);
	//octets Deceleration
	OutBuffer[8] = (byte)(tVarDec & 0xFF);
	tVarDec >>= 8;
	OutBuffer[9] = (byte)(tVarDec);
	//octets positionX
	OutBuffer[10] = (byte)(tVarPosX & 0xFF);
	tVarPosX >>= 8;
	OutBuffer[11] = (byte)(tVarPosX & 0xFF);
	tVarPosX >>= 8;
	OutBuffer[12] = (byte)(tVarPosX & 0xFF);
	tVarPosX >>= 8;
	OutBuffer[13] = (byte)(tVarPosX);
	//octets positionY
	OutBuffer[14] = (byte)(tVarPosY & 0xFF);
	tVarPosY >>= 8;
	OutBuffer[15] = (byte)(tVarPosY & 0xFF);
	tVarPosY >>= 8;
	OutBuffer[16] = (byte)(tVarPosY & 0xFF);
	tVarPosY >>= 8;
	OutBuffer[17] = (byte)(tVarPosY);
	//octets positionZ
	OutBuffer[18] = (byte)(tVarPosZ & 0xFF);
	tVarPosZ >>= 8;
	OutBuffer[19] = (byte)(tVarPosZ & 0xFF);
	tVarPosZ >>= 8;
	OutBuffer[20] = (byte)(tVarPosZ & 0xFF);
	tVarPosZ >>= 8;
	OutBuffer[21] = (byte)(tVarPosZ);
	//octets positionU
	OutBuffer[22] = (byte)(tVarPosU & 0xFF);
	tVarPosU >>= 8;
	OutBuffer[23] = (byte)(tVarPosU & 0xFF);
	tVarPosU >>= 8;
	OutBuffer[24] = (byte)(tVarPosU & 0xFF);
	tVarPosU >>= 8;
	OutBuffer[25] = (byte)(tVarPosU);
	OutBuffer[26] = 0x00;
	for(int i=27;i<taille_tram_envoie+1;i++){OutBuffer[i]=0x00;} // bourage de la trame avec des 0
	if( send(socket_desc , OutBuffer, taille_tram_envoie , 0) < 0){puts("echec envoie demande de deplacement");return 1;} // test de l'envoie
	puts("demande de deplacement faite\n");
	
	/* on va s'assurer de bien atteindre la position ! */
	int tolerance = 2;
	while(abs(cobot_x-consigne_x)>abs(tolerance) || abs(cobot_y-consigne_y)>abs(tolerance)  || abs(cobot_z-consigne_z)>abs(tolerance) )
	{
		waitKey(500); // rafraichissement de la position toute les 0.5 sec		
		demande_position();
		printf("reel x : %d , consigne x : %d, dif : %d\n",cobot_x,consigne_x,cobot_x-consigne_x);
		printf("reel y : %d , consigne y : %d, dif : %d\n",cobot_y,consigne_y,cobot_y-consigne_y);
		printf("reel z : %d , consigne z : %d, dif : %d\n\n",cobot_z,consigne_z,cobot_z-consigne_z);
	}
	waitKey(500);
	return 0;
}

/*
int envoie_tram_mouvement_relatif(int direction,int vitesse,int distance)
{
	OutBuffer[0] = 0xAA;
	OutBuffer[1] = 0xAA;
	OutBuffer[2] = 0x79; //Adresse Rpi
	OutBuffer[3] = 0x11; //mouvement relatif
	//octets direction 1 = X+, 2 = X-, 3 = Y+, 4 = Y-, 5 = Z+, 6 = Z-, 7 = U+, 8 = U-
	OutBuffer[4] = (byte)(direction);
	//octets Vitesse
	OutBuffer[5] = (byte)(vitesse & 0xFF);
	vitesse >>= 8;
	OutBuffer[6] = (byte)(vitesse);
	//octets distance en mm toujours positif
	OutBuffer[7] = (byte)(distance & 0xFF);
	distance >>= 8;
	OutBuffer[8] = (byte)(distance & 0xFF);
	distance >>= 8;
	OutBuffer[9] = (byte)(tVarPosX & 0xFF);
	distance >>= 8;
	OutBuffer[10] = (byte)(distance);
	for(int i=11;i<taille_tram_envoie+1;i++){OutBuffer[i]=0x00;} // bourage de la trame avec des 0

	if( send(socket_desc , OutBuffer, taille_tram_envoie , 0) < 0){puts("echec envoie demande de deplacement");return 1;} // test de l'envoie
	puts("demande de deplacement faite\n");
	return 0;
}
*/

int balayage_zone() 
{
	int rayon;
	float angle;
	for(int i=1;i<3;i++)
	{
		rayon=-200*i+700;	
		for(int j=0;j<14;j++)
		{
		angle=0.25*j;
		envoie_tram_deplacement(acceleration,vitesse,deceleration,(rayon*cos(angle)),(rayon*sin(angle)),170,0);
		sleep(2); //temps entre chaque déplacement
		}
	}	
	return 0;
}


void demande_position()
{
	OutBuffer[0] = 0xAA;
	OutBuffer[1] = 0xAA;
	OutBuffer[2] = 0x79; //Adresse Rpi
	OutBuffer[3] = 0x12; //interogation sur la position
	for(int i=4;i<taille_tram_envoie+1;i++) // bourage de la trame avec des 0
	{
		OutBuffer[i]=0x00;
	} 
	if( send(socket_desc , OutBuffer, taille_tram_envoie , 0) < 0){puts("Send failed");} // test de l'envoie
	
	read(socket_desc, server_reply, taille_tram_reponse_longue);// reception de la reponse

	cobot_x=server_reply[3]+server_reply[4]*0x100+server_reply[5]*0x10000+server_reply[6]*0x1000000;
	cobot_x/=1000;
	cobot_y=server_reply[7]+server_reply[8]*0x100+server_reply[9]*0x10000+server_reply[10]*0x1000000;
	cobot_y/=1000;
	cobot_z=server_reply[11]+server_reply[12]*0x100+server_reply[13]*0x10000+server_reply[14]*0x1000000;
	cobot_z/=1000;
}

int demande_liste_prog()
{
	OutBuffer[0] = 0xAA;
	OutBuffer[1] = 0xAA;
	OutBuffer[2] = 0x79; //Adresse Rpi
	OutBuffer[3] = 0x15; //demande liste programme
	for(int i=4; i < taille_tram_envoie+1; i++)
	{
		OutBuffer[i] = 0x00;
	}
	if(send(socket_desc , OutBuffer, taille_tram_envoie , 0) < 0){puts("Send failed");return 1;}
	
	read(socket_desc, server_reply, taille_tram_reponse_longue);// reception de la reponse
	//tcp_close();
    	int j=0;
    	int num_prog=0;
    	char message;
    	
    	while(server_reply[j]!=NULL)
    	{
        	if(server_reply[j]==0x0d)
        	{
        		num_prog++;
        	}	
        	else if (server_reply[j]==0x79&&server_reply[j+1]==0x01)
        	{
        		j++;
        	}
        	else if (server_reply[j]==0x01||server_reply[j]==0x02||server_reply[j]==0x03||server_reply[j]==0x3f)
            	{
            		// on fait rien et j va prendre +1
            	}
        	else if (server_reply[j]==0x20)
        	{
        		programme[num_prog]+=" ";
        	}
        	else
        	{
            		programme[num_prog]+=server_reply[j];
        	}
        j++;
    	}
    	for(int i=0;i<num_prog;i++)
    	{
    		listeProgramme.append(programme[i]);
    	}
    	return 0;
}

int demande_contenu_prog(string nom_programme)
{
	
	char nom_prog[64]={0x00}; // permet de contenir le nom du programme
    	strcpy(nom_prog, nom_programme.c_str());
	OutBuffer[0] = 0xAA;
	OutBuffer[1] = 0xAA;
	OutBuffer[2] = 0x79; //Adresse Rpi
	OutBuffer[3] = 0x17; //aller � X Y Z U
	for(int i=4;i<taille_tram_envoie+1;i++){OutBuffer[i] = nom_prog[i-4];}
	
	tcp_connect();
	if(send(socket_desc , OutBuffer, taille_tram_envoie , 0) < 0){puts("Send failed");return 1;}
	
	read(socket_desc, server_reply, taille_tram_reponse_longue);// reception de la reponse
	
    	int j=0;
    	ofstream fichier("contenu_programme.txt", ios::out | ios::trunc);  // ouverture en écriture avec effacement du fichier ouvert
    	if(fichier)
    	{
        	while(server_reply[j]!=NULL)
        	{
            		if(server_reply[j]==0x0a)
            		{
                		fichier<<endl;
            		}
            		else if (server_reply[j]==0x79&&server_reply[j+1]==0x01)
        		{
        			j++; // le decalage permet de suprimer le " y"
        		}
            		else if (server_reply[j]==0x01||server_reply[j]==0x02||server_reply[j]==0x03||server_reply[j]==0x3f)
            		{
            		// on fait rien et j va prendre +1
            		}
            		else
            		{
                		fichier<<server_reply[j];
            		}
            		j++;
        	}
   	}
    	else {cerr << "Impossible d'ouvrir le fichier !" << endl;}
}

int demande_arret()
{
	OutBuffer[0] = 0xAA;
	OutBuffer[1] = 0xAA;
	OutBuffer[2] = 0x79; //Adresse Rpi
	OutBuffer[3] = 0x07; //demande arret moteur
	for(int i=4;i<taille_tram_envoie+1;i++){OutBuffer[i] = 0;}
	if(send(socket_desc , OutBuffer, taille_tram_envoie , 0) < 0){puts("Send failed");return 1;}
	printf("demande arret faite\n");
	return 0;
}

int demande_marche()
{
	OutBuffer[0] = 0xAA;
	OutBuffer[1] = 0xAA;
	OutBuffer[2] = 0x79; //Adresse Rpi
	OutBuffer[3] = 0x08; //Commande de mise en marche moteur
	for(int i=4;i<taille_tram_envoie+1;i++){OutBuffer[i] = 0;}
	if(send(socket_desc , OutBuffer, taille_tram_envoie , 0) < 0){puts("Send failed");return 1;}
	printf("demande marche faite\n");
	sleep(1);
	return 0;
}

int moteur_ready()
{	
	int etat_moteur_1=255,etat_moteur_2=255,etat_moteur_3=255,etat_moteur_4=255;
	OutBuffer[0] = 0xAA;
	OutBuffer[1] = 0xAA;
	OutBuffer[2] = 0x79; //Adresse Rpi
	OutBuffer[3] = 0x24; //demande de lecture d'etat des octets
	for(int i=4;i<taille_tram_envoie+1;i++){OutBuffer[i] = 0;}
	
	while(etat_moteur_1!=4 || etat_moteur_2!=4 || etat_moteur_3!=4 || etat_moteur_4!=4)
	{
		sleep(1); // on temporise les demandes pour ne pas surcharger la trame
		if(send(socket_desc , OutBuffer, taille_tram_envoie , 0) < 0){puts("Send failed");return 1;}
		printf("demande etat octet faite\n");
		read(socket_desc, server_reply, taille_tram_reponse_longue);// reception de la reponse
		etat_moteur_1=server_reply[3];
		etat_moteur_2=server_reply[4];
		etat_moteur_3=server_reply[5];
		etat_moteur_4=server_reply[6];
		printf("etat moteur 1 : %d\n",etat_moteur_1);
		printf("etat moteur 2 : %d\n",etat_moteur_2);
		printf("etat moteur 3 : %d\n",etat_moteur_3);
		printf("etat moteur 4 : %d\n",etat_moteur_4);
		printf("etat pince    : %d\n",server_reply[7]);
	}
	return 0;

}

int tcp_connect()
{
    	struct sockaddr_in server;
    	//Create socket
    	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    	if (socket_desc == -1){printf("Could not create socket");}// test de création de la socket
    	server.sin_addr.s_addr = inet_addr("192.168.0.87"); // IP de la RPI (192.168.0.87)
    	server.sin_family = AF_INET;
    	server.sin_port = htons( 87 ); // Port de la RPI (87)
    	// option No delay
    	int i = 1;
	setsockopt( socket_desc, IPPROTO_TCP, TCP_NODELAY, (void *)&i, sizeof(i));
    	
    	/* option keep active */
	/*int optval = 1;
   	if(setsockopt(socket_desc, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval)) < 0){perror("setsockopt()");close(socket_desc);exit(EXIT_FAILURE);}
   	printf("SO_KEEPALIVE is %s\n", (optval ? "ON" : "OFF")); // verification de l'activation*/
   	
 	//Connect to remote server
 	if (connect(socket_desc , (struct sockaddr *)&server , sizeof(server)) < 0){puts("connect error");return 1;} // test de la connection
    	puts("Connected\n");
    
	return 0;
}

int tcp_close()
{
	close(socket_desc);
	puts("closed\n");
}

