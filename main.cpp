#include "opencv2/opencv.hpp"

#include <gtkmm/button.h>
#include <gtkmm/buttonbox.h>
#include <gtkmm/comboboxtext.h>
#include <gtkmm/label.h>
#include <gtkmm/main.h>
#include <gtkmm/window.h>
#include <gtkmm/stock.h>
#include <gtkmm/messagedialog.h>
#include <gtkmm/dialog.h>

#include <iostream>
#include <string>
#include <fstream>
#include <map>
#include <string>
#include <cstdint>
#include <time.h>

#include <akaze_detect.hpp>
#include <tcp.hpp>
#include <traitement_texte.hpp>

using namespace cv;
using namespace std;

/**************************************/
/* declaration des variables          */
/**************************************/

int FRAME_WIDTH = 640; //largeur de l’image, s’adapte automatiquement en fonction de la caméra
int FRAME_HEIGHT = 480; //hauteur de l’image, s’adapte automatiquement en fonction de la camera

int hauteur_init=170; // hauteur à laquelle on observe l'objet à trouver, changer cette valeur pour changer la zone du plan de travail observé

bool etat_init=false; // permet de savoir si le programme a été initialisé ou pas 
bool fin_prog=true; // permet de savoir si un ordre de fin a été donné ou pas
bool detection=false; // permet de savoir si un objet est detecté ou pas
double x,y; // coordonnée de la pièce dans le plan du cobot

double acceleration=80,deceleration=60; //acceleration et deceleration par defaut
double vitesse=1500; // vitesse par defaut,(1500 = 100% de la valeur max constructeur)
double echelle; // rapport entre les pixels et les cm sur le plan de travail

string nom_programme;// nom du programme à executer, défini par l'utilisateur lors de l'initialisation
string nom_objet; // nom de l'objet à detecter, défini par l'utilisateur lors de l'initialisation

Gtk::ComboBoxText listeProgramme; // liste deroulante contenant tous les programmes du cobot
Gtk::ComboBoxText listeObjet; // liste deroulante contenant toutes les references enregistrées ultérieurement


/**************************************/
/* fin  declaration des variables     */
/**************************************/


void start(Gtk::Label* etiquette)
{
	if(etat_init==true && fin_prog==true)
	{
		fin_prog=false;
		etiquette->set_text(" Programme en marche ");
		tcp_connect();
		demande_marche();
		//moteur_ready();
		execution_programme();
		fin_prog=true;
	}
	else if(etat_init==true && fin_prog==false)
	{
		cout<<"debug"<<endl;
	}
	else
	{
		Gtk::MessageDialog dialogue("Vous devez initialiser le programme avant de le lancer", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_CLOSE);
		dialogue.set_title("Information");
		dialogue.run();
	}
}

void initialisation(Gtk::Label* etiquette)
{
    	if(listeObjet.get_active_text()==""||listeObjet.get_active_text()=="."||listeObjet.get_active_text()=="..")
    	{
    		Gtk::MessageDialog dialogue("Vous devez choisir une reference valide", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_CLOSE);
		dialogue.set_title("Information");
		dialogue.run();
    	}
    	else if(listeProgramme.get_active_text()=="")
    	{
    		Gtk::MessageDialog dialogue("Vous devez choisir un programme valide", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_CLOSE);
		dialogue.set_title("Information");
		dialogue.run();
    	}
    	else
    	{
    		nom_objet=listeObjet.get_active_text();
    		nom_programme=listeProgramme.get_active_text();
		demande_contenu_prog(nom_programme);// recuperation du contenu du programme choisi
		affichage_contenu_programme();
		init_espace_travaille();
		etiquette->set_text("Initialisé");
		etat_init=true;
	}
}

void arret(Gtk::Label* etiquette)
{
	if(fin_prog==true)
	{
    		Gtk::MessageDialog dialogue("Aucun programme n'a été lancé", false, Gtk::MESSAGE_ERROR, Gtk::BUTTONS_CLOSE);
		dialogue.set_title("Information");
		dialogue.run();  
	}
	else
	{
		fin_prog=true;
		detection=false;
		demande_arret();
		destroyAllWindows();
		etiquette->set_text(" Programme arreté ");
	}
}

void ajout_ref(Gtk::Entry* saisie)
{
	if(saisie->get_text()!="")
	{
		reference (saisie->get_text());
		init_detection();
	}
	else
	{
		Gtk::MessageDialog dialogue("Entrer un nom de reference", false, Gtk::MESSAGE_ERROR,Gtk::BUTTONS_CLOSE);
		dialogue.set_title("Information");
		dialogue.run();
	}
}

int main(int argc, char* argv[]) 
{
	cout<<"debut main"<<endl;
	
	system("sudo ifconfig enp2s0 192.168.0.1 netmask 255.255.255.0 up"); // on donne une adresse IP fixe au port ethernet "enp2s0" afin de pouvoir communiquer avec le cobot
	
    	Gtk::Main app(argc, argv);
    	Gtk::Window fenetre;
    
    	Gtk::VBox boiteV(false, 10); //Création d'une boîte verticale. Il y aura un minimum de 10px entre chaque widget.
    	Gtk::HBox boiteH0(true); //Création d'une boîte horizontale. Les widgets prendront le même espace.
	Gtk::HBox boiteH1(true); //Création d'une boîte horizontale. Les widgets prendront le même espace.
	Gtk::HBox boiteH2(true); //Création d'une boîte horizontale. Les widgets prendront le même espace.
	Gtk::HBox boiteH3(true); //Création d'une boîte horizontale. Les widgets prendront le même espace.
	Gtk::HBox boiteH4(true); //Création d'une boîte horizontale. Les widgets prendront le même espace.
	Gtk::HBox boiteH5(true); //Création d'une boîte horizontale. Les widgets prendront le même espace.
	
	Gtk::Button bouton_add(Gtk::Stock::ADD);
    	bouton_add.set_can_focus(false);
    	Gtk::Button bouton_init(Gtk::Stock::APPLY);
    	bouton_init.set_can_focus(false);
    	Gtk::Button bouton_start("Start");
    	bouton_start.set_can_focus(false);
    	Gtk::Button bouton_fermer(Gtk::Stock::STOP);
    	bouton_fermer.set_can_focus(false);
    	
    	Gtk::Label etiquette0(" Ajouter une reference à detecter : ");
    	Gtk::Label etiquette1(" Selectionner la reférence à rechercher : ");
    	Gtk::Label etiquette2(" Selectionner le programme à excecuter : ");
    	Gtk::Label etiquette3(" Initialisation du système : ");
    	Gtk::Label etiquette4(" Mise en marche système : ");
    	Gtk::Label etiquette5(" Arret du programme : ");
    	Gtk::Label etiquette6(" Non initialisé ");
    	Gtk::Label etiquette7("  ");
    	Gtk::Label etiquette8(" Programme arreté ");
    	
    	Gtk::Entry etiquette9; // zone de saisie pour donner un nom à la nouvelle reference
    
    	//Ajout de boutons à la boîte.
    	boiteH0.pack_start(etiquette0);
    	boiteH0.pack_start(etiquette9);
    	boiteH0.pack_start(bouton_add);
	boiteH1.pack_start(etiquette1);
    	boiteH1.pack_start(listeObjet);
    	boiteH2.pack_start(etiquette2);
    	boiteH2.pack_start(listeProgramme);
    	boiteH3.pack_start(etiquette3);
    	boiteH3.pack_start(bouton_init);
    	boiteH3.pack_start(etiquette6);
	boiteH4.pack_start(etiquette4);
    	boiteH4.pack_start(bouton_start);
    	boiteH4.pack_start(etiquette7);
    	boiteH5.pack_start(etiquette5);
	boiteH5.pack_start(bouton_fermer);
	boiteH5.pack_start(etiquette8);
   	
    	boiteV.pack_start(boiteH0);
    	boiteV.pack_start(boiteH1);
    	boiteV.pack_start(boiteH2);
    	boiteV.pack_start(boiteH3);
    	boiteV.pack_start(boiteH4);
	boiteV.pack_start(boiteH5);
    
    	tcp_connect(); // connection au cobot
	demande_liste_prog(); // recuperation de la liste des programmes
	init_detection(); // permet d'actualiser la liste des reference d'objet
     
    	fenetre.add(boiteV); //Ajout de la boîte à la fenêtre.

    	fenetre.show_all();
    	
	listeObjet.signal_changed().connect([]() { cout<<"etiquette"<<endl; });
    	listeProgramme.signal_changed().connect([]() {cout<<"etiquette"<<endl;;});
	bouton_fermer.signal_clicked().connect([&etiquette8]() {Gtk::Main::quit();arret(&etiquette8);Gtk::Main::run();});
    	bouton_start.signal_clicked().connect([&etiquette8]() { Gtk::Main::quit();start(&etiquette8);Gtk::Main::run(); });
    	bouton_init.signal_clicked().connect([&etiquette6]() { Gtk::Main::quit();initialisation(&etiquette6);Gtk::Main::run();});
    	bouton_add.signal_clicked().connect([&etiquette9]() { Gtk::Main::quit();ajout_ref(&etiquette9);Gtk::Main::run(); });
    
    	Gtk::Main::run(fenetre);
    
    	return 0;
}



