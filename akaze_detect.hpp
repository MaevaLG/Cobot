#ifndef _AKAZE_DETECT_H
#define _AKAZE_DETECT_H

#include <iostream>


int init_espace_travaille(void);

int reference (std::string nom);

int init_detection(void);

void detection_akaze(void);

void close_windows(void);


#endif
