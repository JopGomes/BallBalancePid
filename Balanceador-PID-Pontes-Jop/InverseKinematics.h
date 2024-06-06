#ifndef InverseKinematics_H
#define InverseKinematics_H

//constants
#define A 0
#define B 1
#define C 2

class Machine { //machine class
  public:
    //class functions
    Machine(float d, float e, float f, float g);
    double theta(int leg, float hz, float nx, float ny); //returns the value of theta a, b, or c
};

#endif
