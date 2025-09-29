#pragma once

#include <iostream>
#include <vector>
using namespace std;

class SFSMC
{
public:
	// ���ø������ɶȿ���������
	void setPitchController_Parameters(double SAT_MAX_PITCH, double KP1, double KP2, double DELP
		, double GAMMAP, double KDELP,double BETAP);
	//����ƫ�����ɶȿ���������
	void setYawController_Parameters(double SAT_MAX_YAW, double KY1, double KY2, double DELY
		, double GAMMAY, double KDELY, double BETAY);
	//������
	void ControllerOutput(SFSMC& controller);
	vector<double> getResult();
	//����������̬
	void setDesiredAttitude(double PITCH_DESIRED, double YAW_DESIRED);
	//��ȡ������̬
	vector<double> getDesiredAttitude();
	//��ȡ��ǰAUV��̬
	vector<double> getCurrentAttitude();
	//��ȡ��һʱ�̵���̬���
	vector<double> getErrorLast();
	//����IT2-FLS��Ȩֵ����-dWp+dWy+Basp+Basy
	void setIT2_FLS(vector<double> INPUT_VALUE, double S1, double S2, vector<double> WF,
		vector<double> BAS, double roll, double M1, double M2);
	vector<double> getIT2_FLS(); //-dWp+dWy+Basp+Basy
	//���õ�ǰ��̬
	void setCurrentAttitude(double ROLL_ANGLE, double PITCH_ANGLE, double YAW_ANGLE);
	//������һʱ�̵���̬���
	void setErrorLast(double PITCH_ERROR, double YAW_ERROR);
	//����Ȩֵ��������
	void setWeightVectors(vector<double> WEIGHT);
	vector<double> getWeightVectors();
	//���û�����
	void setBasisVectors(vector<double> BASIS);
	vector<double> getBasisVectors();

	void setSigmaVectors(double sigma_pitch, double sigma_yaw);
	vector<double> getSigmaVectors();

	void setLastAttitude(double roll_value,double pitch_value, double yaw_value);
	vector<double> getLastAttitude();

public:
	vector<double> x1UMF = { 0.0,0.0,0.0 };
	vector<double> x2UMF = { 0.0,0.0,0.0 };
	vector<double> x3UMF = { 0.0,0.0,0.0 };
	vector<double> x4UMF = { 0.0,0.0,0.0 };

	vector<double> x1LMF = { 0.0,0.0,0.0 };
	vector<double> x2LMF = { 0.0,0.0,0.0 };
	vector<double> x3LMF = { 0.0,0.0,0.0 };
	vector<double> x4LMF = { 0.0,0.0,0.0 };

	double temp1 = 0.0;
	double temp2 = 0.0;
	double temp3 = 0.0;

	int num = 0;
	double sum1 = 0.0;
	double sum2 = 0.0;

	vector<double> temp1_vector = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> temp2_vector = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> temp3_vector = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> temp4_vector = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> temp5_vector = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> temp6_vector = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

	vector<double> WFP_vector = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> WFY_vector = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

	vector<double> FLY = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> FUY = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

	vector<double> FLB = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> FUB = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

	double sum_pitch_error = 0.0;
	double sum_yaw_error = 0.0;
private:
	vector<double> SIGMA_F = { 2.5,2.5 };
	vector<double> CURRENT_ATTITUDE = { 0.0,0.0,0.0 };
	vector<double> LAST_ATTITUDE = { 0.0,0.0,0.0 };
	vector<double> WEIGHT_VECTOR = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 ,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> D_WEIGHT_VECTOR = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 ,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> BASIS_VECTOR = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 ,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> LAST_ATTITUDE_ERROR = { 0.0,0.0 };
	vector<double> DESIRED_ATTITUDE = { 0.0,0.0 };
	vector<double> result = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 ,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 ,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 ,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
	vector<double> INPUT_FLS = { 0.0,0.0,0.0,0.0 };
	vector<double> TORQUE = {0.0,0.0};

	double C_SAT_MAX_PITCH;
	double C_KP1;
	double C_KP2;
	double C_DELP;
	double C_GAMMAP;
	double C_KDELP;
	double C_BETAP;

	double C_SAT_MAX_YAW;
	double C_KY1;
	double C_KY2;
	double C_DELY;
	double C_GAMMAY;
	double C_KDELY;
	double C_BETAY;

	double M1 = 8.33;
	double M2 = 8.33;


	double SAMPLE_TIME = 0.1;
};
