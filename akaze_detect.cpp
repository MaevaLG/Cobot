#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <dirent.h>
#include <gtkmm.h>

#include <akaze_detect.hpp>
#include <tcp.hpp>
#include <traitement_texte.hpp>


using namespace std;
using namespace cv;

/**************************************/
/* declaration des variables          */
/**************************************/

#define camera_x -440 // coordonnée en x de la camera par rapport à l'origine du robot
#define camera_y 80 // coordonnée en y de la camera par rapport à l'origine du robot
#define num_cam 1 //selection de la camera, 0 est la camera par defaut 
#define nb_pt_commun 5 // nombre de point commun minimum pour dire que c'est bien la pièce, à ajusté en fonction de la résolution de la camera et de l'environement

extern int cobot_x, cobot_y, cobot_z;
extern double echelle;
extern double x,y; 
extern double vitesse,acceleration,deceleration;
extern int FRAME_WIDTH ;
extern int FRAME_HEIGHT;
extern bool detection;
extern string nom_objet; 
extern Gtk::ComboBoxText listeObjet;

const float inlier_threshold = 1000.0; // Distance threshold to identify inliers
const float nn_match_ratio =0.6;   // Nearest neighbor matching ratio
float distance_reel=200; // distance entre les 2 points sur la feuille de callibrage

VideoCapture capture;//video capture object to acquire webcam feed

Mat img1,img1_gray,frame_gray,frame; // stockage des différentes images qui seront traité


/**************************************/
/*fin  declaration des variables      */
/**************************************/

int init_espace_travaille()
{
	capture.open(num_cam);
	if(!capture.isOpened()){fprintf( stderr, "connection impossible \r\n" );getchar();return -1;}
	FRAME_WIDTH=capture.get(CV_CAP_PROP_FRAME_WIDTH);
	FRAME_HEIGHT=capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	printf("largeur de la frame : %d , hauteur de la frame : %d\n",FRAME_WIDTH,FRAME_HEIGHT);
	Mat calibrage,calibrage_gray,calibrage_bw;
	
	char touche='a';
	while(touche!='q'&& touche!='Q')
	{
		cvNamedWindow( "calibrage", CV_WINDOW_AUTOSIZE );
		capture.read(calibrage);
		putText(calibrage, "Placer la feuille de calibration puis appuyer sur 'q' ",cvPoint(30,30),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,255), 1.5, CV_AA);
		imshow( "calibrage", calibrage);
		touche=waitKey(50);
	}
	destroyWindow("calibrage");
	capture.read(calibrage);
	
	/* mise en nuance de gris */
	cvtColor( calibrage , calibrage_gray, COLOR_BGR2GRAY );
	
	/* convertion en binaire */
	GaussianBlur(calibrage_gray, calibrage_gray, Size(7,7), 1.5, 1.5,BORDER_DEFAULT);
	Canny( calibrage_gray, calibrage_bw, 170, 200, 3, true );
	
	/*hough transformé pour detecter les cerlces */
	std::vector<cv::Vec<float, 3> > circles; // Memory for hough circles
    HoughCircles(calibrage_bw, circles, CV_HOUGH_GRADIENT, 2,30, 20, 50, 5, 70);
        
	/* affichage des centre de cercle detecté */        
    for( size_t i = 0; i < circles.size(); i++ )
   	{   
   		Point center(cvRound(circles[i][0]), cvRound(circles[i][1])); // coordonnées du cercle x , y
       	int radius = cvRound(circles[i][2]); // rayon du cercle 
       	circle(calibrage, center, 3, Scalar(0,255,0), -1, 8, 0 ); // draw the circle center
   	}
    	
   	/* ici on va recuperer la distance en norme entre les 2 points et diviser la distance reel en mm par cette norme
   	on obtient ainsi une "echelle" donnant la relation entre les pixels et les mm */
   	echelle=distance_reel/(float)sqrt((circles[0][0]-circles[1][0])*(circles[0][0]-circles[1][0])+(circles[0][1]-circles[1][1])*(circles[0][1]-circles[1][1])); //(x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)
    printf("La mise a l'echelle est de : %f\n",echelle);
    	
    /* on stock les images afin de permettre le debeug */
   	imwrite("calibrage/calibrage.png", calibrage);
   	imwrite("calibrage/calibrage_bw.png", calibrage_bw);
    	
return 0;
}


int reference (string nom1)
{	
		capture.open(num_cam);
		
		if(!capture.isOpened())
		{
			fprintf(stderr, "connection impossible \r\n" );
			getchar();
			return -1;
		}
		Mat reference;
		char touche='a';
		while(touche!='q'&& touche!='Q')
		{
			cvNamedWindow( "reference", CV_WINDOW_AUTOSIZE );
			capture.read(reference);
			putText(reference, "appuyer sur 'q' pour prendre la photo ",cvPoint(30,30),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,255,255), 1.5, CV_AA);
			imshow( "reference", reference);
			touche=waitKey(50);
		}
		capture.read(reference);
		destroyWindow("reference");
		capture.release();
		char nom[30];
		strcpy(nom,"reference/");
		strcat (nom,nom1.c_str());
		strcat (nom,".png");
		
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(3);
		imwrite(nom,reference),compression_params;
}

int init_detection()
{
	DIR * rep = opendir("./reference/");
	listeObjet.clear();
    	if (rep != NULL)
    	{
       	 	struct dirent * ent;
        	while ((ent = readdir(rep)) != NULL)
        	{
        		listeObjet.append(ent->d_name);
        	}
        	closedir(rep);
    	}
	capture.open(num_cam);
	if(!capture.isOpened())
	{
		fprintf(stderr, "connection impossible \r\n" );
		getchar();
		return -1;
	} // test de la connection avec la webcam 
	puts("Camera connected\r\n");
	return(0);
}

void detection_akaze()
{
	char adresse_img[30];	
	
	strcpy (adresse_img,"./reference/");
	strcat (adresse_img,nom_objet.c_str());
	img1 = imread(adresse_img);
    cvNamedWindow( "resultat", CV_WINDOW_AUTOSIZE );
    moveWindow("resultat", 0, 250);
    	
    capture.read(frame);
	// mise en nuance de gris de l'image de ref et de la frame 
    cvtColor(img1,img1_gray,CV_BGR2GRAY);
    cvtColor(frame,frame_gray,CV_BGR2GRAY);
    
    // utilisation d'homography 
    Mat homography;
    FileStorage fs("src/detection_akaze/H1to3p.xml", FileStorage::READ);
   	fs.getFirstTopLevelNode() >> homography;
	
	std::vector<KeyPoint> kpts1, kpts2;
   	Mat desc1, desc2;

    Ptr<AKAZE> akaze = AKAZE::create();
    akaze->detectAndCompute(img1, noArray(), kpts1, desc1);
    akaze->detectAndCompute(frame, noArray(), kpts2, desc2);

    BFMatcher matcher(NORM_HAMMING);
    vector< vector<DMatch> > nn_matches;
    matcher.knnMatch(desc1, desc2, nn_matches, 2);

    vector<KeyPoint> matched1, matched2, inliers1, inliers2;
    vector<DMatch> good_matches;
    
	if(kpts2.size()!=0)// évite le plantage si il n'y a pas de point de reférence sur la 2ème image
	{ 
    	for(size_t i = 0; i < nn_matches.size(); i++) 
    	{
        	DMatch first = nn_matches[i][0];
        	float dist1 = nn_matches[i][0].distance;
        	float dist2 = nn_matches[i][1].distance;
        	if(dist1 < nn_match_ratio * dist2) 
        	{
            	matched1.push_back(kpts1[first.queryIdx]);
            	matched2.push_back(kpts2[first.trainIdx]);
        	}
    	}
    	for(unsigned i = 0; i < matched1.size(); i++) 
    	{
        	Mat col = Mat::ones(3, 1, CV_64F);
        	col.at<double>(0) = matched1[i].pt.x;
        	col.at<double>(1) = matched1[i].pt.y;

        	col = homography * col;
        	col /= col.at<double>(2);
        	double dist = sqrt( pow(col.at<double>(0) - matched2[i].pt.x, 2) + pow(col.at<double>(1) - matched2[i].pt.y, 2));
	
        	if(dist < inlier_threshold) 
       		{
        	   	int new_i = static_cast<int>(inliers1.size());
        	   	inliers1.push_back(matched1[i]);
        	   	inliers2.push_back(matched2[i]);
        	   	good_matches.push_back(DMatch(new_i, new_i, 0));
        	}
    	}
	}
	
	if(matched2.size()>=nb_pt_commun) // dans le cas ou il y a plus que nb_pt_commun de point commun alors c'est la pièce que l'on detecte
	{
		detection=true;
		x=0,y=0;
		/* on calcul le baricentre du nuage de points */
		for(unsigned i=0; i < matched2.size();i++) 
		{
			x+=matched2[i].pt.x;
			y+=matched2[i].pt.y;
		}
		x/=matched2.size();
		y/=matched2.size();
	}
	else
	{
		detection=false;
	} // si il n'y a pas de détéction
    
	/* placement d'un point au baricentre du nuage de points */
	Mat res;
	Point center((x), (y));
	circle( frame, center, 10, Scalar(0,255,0), -1, 8, 0 );

	/* on trace les points commun entre la référence et la video continu */
	drawMatches(img1, inliers1, frame, inliers2, good_matches, res);
	imshow( "resultat" , res);
    
	/* mise de l'origine 0,0 au centre de l'image */
	x-=FRAME_WIDTH/2;
	y-=FRAME_HEIGHT/2;

	/* remise à l'echelle du cobot */
	x*=echelle;
	y*=echelle;
	   
	x=camera_x+x;
	y=camera_y-y;
		  
	system("clear");

	double inlier_ratio = inliers1.size() * 1.0 / matched1.size();

    cout << "**************************************" << endl;
    cout << "Keypoints 1:                        \t" << kpts1.size() << endl;
    cout << "Keypoints 2:                        \t" << kpts2.size() << endl;
   	cout << "Matches:                            \t" << matched1.size() << endl;
   	cout << "X =                                 \t" << x <<endl;
   	cout << "Y =                                 \t" << y <<endl;
   	cout << endl;

   	waitKey(5);
}

void close_windows()
{
	destroyWindow("resultat");
}
