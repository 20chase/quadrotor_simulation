#include <iostream>
#include <Eigen/Dense>
#include <fstream>

using namespace std;
using namespace Eigen;

double Ixx = 0;
double Iyy = 0;
double Izz = 0;
double Ir  = 0;

double M = 0;

double G = 9.8;

double Kl  = 0;
double Kd  = 0;

double L   = 0;

double attitudeTime = 0.005;

struct PID
{
	double kp;
	double kd;
};

PID PIDRoll, PIDPitch, PIDYaw, PIDX, PIDY, PIDZ;



MatrixXd w(4, 1), inertia(3, 3), rotation(3, 3);
MatrixXd acceleration(3, 1), velocity(3, 1), position(3, 1);
MatrixXd angle(3, 1), omega(3, 1), omegaDot(3, 1);
MatrixXd P4Attitude(3, 1), D4Attitude(3, 1), P4Position(3, 1), D4Position(3, 1);

int readParam()
{
	ifstream fin;

    fin.open("/home/ftx1994/QuadRotor_Param.txt");

	if (!fin)
	{
		cout << "Warning: Read Parameter Err!" << endl;
		cout << "5 seconds later will exit!" << endl;
		return -1;
	}

	double value[5];

	while (!fin.eof())
	{
		char name;

		fin >> name >> value[0] >> value[1] >> value[2] >> value[3];

		if (name == '#')
		{
			break;
		}

		if (name == 'I')
		{
			Ixx = value[0];
			Iyy = value[1];
			Izz = value[2];
			Ir  = value[3];
			continue;
		}

		if (name == 'M')
		{
			M = value[0];
			continue;
		}

		if (name == 'G')
		{
			G = value[0];
			continue;
		}

		if (name == 'K')
		{
			Kl = value[0];
			Kd = value[1];
			continue;
		}

		if (name == 'L')
		{
			L = value[0];
			continue;
		}

		if (name == 'X')
		{
            PIDRoll.kp = value[0];
            PIDRoll.kd = value[1];
            PIDX.kp = value[2];
            PIDX.kd = value[3];
			continue;
		}

		if (name == 'Y')
		{
            PIDPitch.kp = value[0];
            PIDPitch.kd = value[1];
            PIDY.kp = value[2];
            PIDY.kd = value[3];
			continue;
		}

		if (name == 'Z')
		{
            PIDYaw.kp = value[0];
            PIDYaw.kd = value[1];
            PIDZ.kp = value[2];
            PIDZ.kd = value[3];
			continue;
		}

	}

	fin.close();

	return 1;
}

int initialize()
{
	if (readParam() < 0)
	{
		return -1;
	}

	inertia = MatrixXd::Zero(3, 3);
	inertia(0, 0) = Ixx;
	inertia(1, 1) = Iyy;
	inertia(2, 2) = Izz;

    P4Attitude(0, 0) = PIDRoll.kp;
    P4Attitude(1, 0) = PIDPitch.kp;
    P4Attitude(2, 0) = PIDYaw.kp;

    D4Attitude(0, 0) = PIDRoll.kd;
    D4Attitude(1, 0) = PIDPitch.kd;
    D4Attitude(2, 0) = PIDYaw.kd;

    P4Position(0, 0) = PIDX.kp;
    P4Position(1, 0) = PIDY.kp;
    P4Position(2, 0) = PIDZ.kp;

    D4Position(0, 0) = PIDX.kd;
    D4Position(1, 0) = PIDY.kd;
    D4Position(2, 0) = PIDZ.kd;



    w = MatrixXd::Zero(3, 1);
    angle = MatrixXd::Zero(3, 1);
    omega = MatrixXd::Zero(3, 1);
    omegaDot = MatrixXd::Zero(3, 1);
    position = MatrixXd::Zero(3, 1);
    velocity = MatrixXd::Zero(3, 1);
    acceleration = MatrixXd::Zero(3, 1);

    w = MatrixXd::Zero(4, 1);

}

void writeTrajectory(double (*trajectory)[6], int num)
{
    ofstream fout("/home/ftx1994/Trajectory.txt");

	for (int i = 0; i < num; i++)
	{
		fout << trajectory[i][0] << "    " << trajectory[i][1] << "    " << trajectory[i][2] << "    " << trajectory[i][3] << "    " << trajectory[i][4] << "    " << trajectory[i][5] << endl;
	}

}
