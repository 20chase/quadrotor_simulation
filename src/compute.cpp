#include <iostream>
#include <Eigen/Dense>
#include <math.h>

using namespace std;
using namespace Eigen;

extern double Ixx;
extern double Iyy;
extern double Izz;
extern double Ir;

extern double M;

extern double G;

extern double Kl;
extern double Kd;

extern double L;

extern MatrixXd inertia;


MatrixXd computeRotation(MatrixXd angle)
{
	MatrixXd rotation(3, 3);

	double phi = angle(0, 0);
	double theta = angle(1, 0);
	double psi = angle(2, 0);

	rotation << cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
		        cos(theta)*sin(phi) + cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi) , sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),
		                     -cos(phi)*sin(theta)                 ,       sin(phi)    ,              cos(psi)*cos(theta)                  ;

	return rotation;
}

MatrixXd computeThrust(MatrixXd w)
{
	MatrixXd Thrust(3, 1);

	Thrust(0, 0) = 0;
	Thrust(1, 0) = 0;
	
	double buffer = w.sum();

	Thrust(2, 0) = Kl * buffer;

	return Thrust;
}

MatrixXd computeAcceleration(MatrixXd Thrust, MatrixXd Rotation)
{
	MatrixXd gravity(3, 1);
	gravity = MatrixXd::Zero(3, 1);
	gravity(2, 0) = -G;

	Thrust = Rotation * Thrust;

	MatrixXd Accerelation(3, 1);

	Accerelation = gravity + Thrust / M;

	return Accerelation;
}

MatrixXd computeTorque(MatrixXd w)
{
	MatrixXd Torgue(3, 1);

	Torgue(0, 0) = Kl * (w(1, 0) - w(3, 0)) * L;
	Torgue(1, 0) = Kl * (w(2, 0) - w(0, 0)) * L;
	Torgue(2, 0) = Kd * (w(0, 0) + w(2, 0) - w(1, 0) - w(3, 0));

	return Torgue;
}

MatrixXd computeOmegaDot(MatrixXd omega, MatrixXd torque, MatrixXd w)
{
	MatrixXd omegaDot(3, 1);
    double sumW = sqrt(w(0, 0)) - sqrt(w(1, 0)) + sqrt(w(2, 0)) - sqrt(w(3, 0));
    omegaDot(0, 0) = torque(0, 0) / Ixx - (Iyy - Izz) * omega(1, 0) * omega(2, 0) / Ixx + Ir * omega(1, 0) * sumW;
    omegaDot(1, 0) = torque(1, 0) / Iyy - (Izz - Ixx) * omega(0, 0) * omega(2, 0) / Iyy + Ir * omega(0, 0) * sumW;
	omegaDot(2, 0) = torque(2, 0) / Izz - (Ixx - Iyy) * omega(0, 0) * omega(1, 0) / Izz;

	return omegaDot;
}


double limitMax(double input, double max)
{
    if (abs(input) > max)
    {
        if (input > 0)
        {
            return max;
        }
        else
        {
            return -max;
        }
    }
    else
    {
        return input;
    }
}


