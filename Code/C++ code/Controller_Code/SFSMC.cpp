#include "SFSMC.h"
#define PI 3.1415926

//设置俯仰自由度的控制器参数
void SFSMC::setPitchController_Parameters(double SAT_MAX_PITCH, double KP1, double KP2, double DELP
	, double GAMMAP, double KDELP, double BETAP)
{
	C_SAT_MAX_PITCH = SAT_MAX_PITCH;
	C_KP1 = KP1;
	C_KP2 = KP2;
	C_DELP = DELP;
	C_GAMMAP = GAMMAP;
	C_KDELP = KDELP;
	C_BETAP = BETAP;
}

//设置偏航自由度控制器参数
void SFSMC::setYawController_Parameters(double SAT_MAX_YAW, double KY1, double KY2, double DELY
	, double GAMMAY, double KDELY, double BETAY) 
{
	C_SAT_MAX_YAW = SAT_MAX_YAW;
	C_KY1 = KY1;
	C_KY2 = KY2;
	C_DELY = DELY;
	C_GAMMAY = GAMMAY;
	C_KDELY = KDELY;
	C_BETAY = BETAY;
}
//设置期望姿态
void SFSMC::setDesiredAttitude(double PITCH_DESIRED, double YAW_DESIRED)
{
	DESIRED_ATTITUDE[0] = (PITCH_DESIRED);
	DESIRED_ATTITUDE[1] = (YAW_DESIRED);
}

//获取期望姿态
vector<double> SFSMC::getDesiredAttitude()
{
	return DESIRED_ATTITUDE;
}

void SFSMC::setCurrentAttitude(double ROLL_ANGLE, double PITCH_ANGLE, double YAW_ANGLE)
{
	CURRENT_ATTITUDE[0] = (ROLL_ANGLE);
	CURRENT_ATTITUDE[1] = (PITCH_ANGLE);
	CURRENT_ATTITUDE[2] = (YAW_ANGLE);
}

//获取当前AUV姿态
vector<double> SFSMC::getLastAttitude()
{
	return LAST_ATTITUDE;
}

void SFSMC::setLastAttitude(double ROLL_ANGLE, double PITCH_ANGLE, double YAW_ANGLE)
{
	LAST_ATTITUDE[0] = (ROLL_ANGLE);
	LAST_ATTITUDE[1] = (PITCH_ANGLE);
	LAST_ATTITUDE[2] = (YAW_ANGLE);
}

//获取当前AUV姿态
vector<double> SFSMC::getCurrentAttitude()
{
	return CURRENT_ATTITUDE;
}

//获取上一时刻的姿态误差
vector<double> SFSMC::getErrorLast()
{
	return LAST_ATTITUDE_ERROR;
}

//设置上一时刻的姿态误差
void SFSMC::setErrorLast(double PITCH_ERROR, double YAW_ERROR)
{
	LAST_ATTITUDE_ERROR[0] = (PITCH_ERROR);
	LAST_ATTITUDE_ERROR[1] = (YAW_ERROR);
}
//设置权值向量矩阵
void SFSMC::setWeightVectors(vector<double> WEIGHT)
{
	WEIGHT_VECTOR.swap(WEIGHT);
}
vector<double> SFSMC::getWeightVectors()
{
	return WEIGHT_VECTOR;
}
//设置基向量
void SFSMC::setBasisVectors(vector<double> BASIS)
{
	BASIS_VECTOR.swap(BASIS);
}
vector<double> SFSMC::getBasisVectors()
{
	return BASIS_VECTOR;
}

//更新IT2-FLS的权值向量-dWp+dWy+Basp+Basy
void SFSMC :: setIT2_FLS(vector<double> INPUT_VALUE, double S1, double S2, vector<double> WF, vector<double> BAS, double roll, double M1, double M2)
{
	double pitch_value = INPUT_VALUE[0];
	double d_pitch_value = INPUT_VALUE[2];

	double yaw_value = INPUT_VALUE[1];
	double d_yaw_value = INPUT_VALUE[3];



	for (int i = 0;i < 3;i++) {
		temp1 = -pow(((pitch_value - PI + i * PI) / (PI / 2)), 2);
		temp2 = -pow(((pitch_value - PI + i * PI) / ((PI / 4)-0.02)), 2);
		x1UMF[i] = (double)exp(temp1);
		x1LMF[i] = (double)exp(temp2);
	}

	for (int i = 0;i < 3;i++) {
		temp1 = -pow(((d_pitch_value - 63.0 + i * 63.0) / 31.5), 2);
		temp2 = -pow(((d_pitch_value - 63.0 + i * 63.0) / 14.5), 2);
		x2UMF[i] = (double)exp(temp1);
		x2LMF[i] = (double)exp(temp2);
	}

	for (int i = 0;i < 3;i++) {
		temp1 = -pow(((yaw_value - PI + i * PI) / (PI / 2)), 2);
		temp2 = -pow(((yaw_value - PI + i * PI) / ((PI / 4) - 0.02)), 2);
		x3UMF[i] = (double)exp(temp1);
		x3LMF[i] = (double)exp(temp2);
	}

	for (int i = 0;i < 3;i++) {
		temp1 = -pow(((d_yaw_value - 63.0 + i * 63.0) / 31.5), 2);
		temp2 = -pow(((d_yaw_value - 63.0 + i * 63.0) / 14.5), 2);
		x4UMF[i] = (double)exp(temp1);
		x4LMF[i] = (double)exp(temp2);
	}

	for (int i = 0;i < 3;i++) {
		for (int j = 0;j < 3;j++) {
			FLY[num] = x1LMF[i] * x2LMF[j];
			FUY[num] = x1UMF[i] * x2UMF[j];
			num = num + 1;
		}
	}

	num = 0;
	for (int i = 0;i < 9;i++) {
		sum1 = sum1 + FLY[i];
		sum2 = sum2 + FUY[i];
	}

	for (int i = 0;i < 9;i++) {
		FLB[i] = FLY[i] / sum1;
		FUB[i] = FUY[i] / sum2;
		temp1_vector[i] = 0.5 * (FLB[i] + FUB[i]);
	}
	sum1 = 0.0; sum2 = 0.0;

	for (int i = 0;i < 3;i++) {
		for (int j = 0;j < 3;j++) {
			FLY[num] = x3LMF[i] * x4LMF[j];
			FUY[num] = x3UMF[i] * x4UMF[j];
			num = num + 1;
		}
	}

	num = 0;
	for (int i = 0;i < 9;i++) {
		sum1 = sum1 + FLY[i];
		sum2 = sum2 + FUY[i];
	}

	for (int i = 0;i < 9;i++) {
		FLB[i] = FLY[i] / sum1;
		FUB[i] = FUY[i] / sum2;
		temp2_vector[i] = 0.5 * (FLB[i] + FUB[i]);
	}
	sum1 = 0.0; sum2 = 0.0;

	temp1 = S1 * M1 * cos(roll) - S2 * M1 * sin(roll);
	temp2 = ((S1 * M2 * sin(roll)) / (cos(pitch_value))) + ((S2 * M2 * cos(roll)) / (cos(pitch_value)));

	for (int i = 0;i < 9;i++) {
		temp3_vector[i] = C_BETAP * temp1 * temp1_vector[i];
		temp4_vector[i] = C_BETAY * temp2 * temp2_vector[i];
	}
	for (int i = 0;i < 9;i++) {
		WFP_vector[i] = WF[i];
		WFY_vector[i] = WF[i + 9];
		temp5_vector[i] = C_BETAP * -2 *0.1* WFP_vector[i] + temp3_vector[i];
		temp6_vector[i] = C_BETAY * -2 *0.1* WFY_vector[i] + temp4_vector[i];
	}

	for (int i = 0;i < 36;i++) {
		if (i < 9) {
			result[i] = temp5_vector[i];
		}
		else if (i >= 9 && i < 18) {
			result[i] = temp6_vector[i-9];
		}
		else if (i >= 18 && i < 27) {
			result[i] = temp1_vector[i-18];
		}
		else {
			result[i] = temp2_vector[i-27];
		}
	}
}

vector<double> SFSMC::getIT2_FLS()
{
	return result;
}

void SFSMC::setSigmaVectors(double sigma_pitch, double sigma_yaw)
{
	SIGMA_F[0] = sigma_pitch;
	SIGMA_F[1] = sigma_yaw;
}
vector<double> SFSMC::getSigmaVectors()
{
	return SIGMA_F;
}

void SFSMC::ControllerOutput(SFSMC& controller)
{
	vector<double> stateValue(controller.getCurrentAttitude());
	vector<double> desiredValue(controller.getDesiredAttitude());
	vector<double> lastErrorValue(controller.getErrorLast());
	double roll_value = stateValue[0];
	double pitch_value = stateValue[1];
	double yaw_value = stateValue[2];

	double desired_pitch_value = desiredValue[0];
	double desired_yaw_value = desiredValue[1];

	double sigma_pitch_value = (controller.SIGMA_F)[0];
	double sigma_yaw_value = (controller.SIGMA_F)[1];

	double last_pitch_error_value = lastErrorValue[0];
	double last_yaw_error_value = lastErrorValue[1];

	double error_pitch_value = pitch_value - desired_pitch_value;
	double error_yaw_value = yaw_value - desired_yaw_value;

	controller.setErrorLast(error_pitch_value, error_yaw_value);

	double d_error_pitch_value = (error_pitch_value - last_pitch_error_value)/controller.SAMPLE_TIME;
	double d_error_yaw_value = (error_yaw_value - last_yaw_error_value)/controller.SAMPLE_TIME;

	sum_pitch_error = sum_pitch_error + error_pitch_value;
	sum_yaw_error = sum_yaw_error + error_yaw_value;

	double s_pitch = d_error_pitch_value + controller.C_KP1 * pow(sigma_pitch_value, 2) * error_pitch_value +
		controller.C_KP2 * pow(sigma_pitch_value, 2) * controller.sum_pitch_error;
	double s_yaw = d_error_yaw_value + controller.C_KY1 * pow(sigma_yaw_value, 2) * error_yaw_value +
		controller.C_KY2 * pow(sigma_yaw_value, 2) * controller.sum_yaw_error;

	double t1 = -controller.C_SAT_MAX_PITCH * tanh(s_pitch / (C_DELP * pow(sigma_pitch_value, 2)));
	double t2 = -controller.C_SAT_MAX_YAW * tanh(s_yaw / (C_DELY * pow(sigma_yaw_value, 2)));

	double phi1 = controller.C_KP1 *  error_pitch_value + controller.C_KP2 *  controller.sum_pitch_error;
	double phi2 = controller.C_KY1 *  error_yaw_value + controller.C_KY2 *  controller.sum_yaw_error;
	
	double G0 = s_pitch * M1 * cos(roll_value) * phi1 - s_yaw * M1 * sin(roll_value) * phi1 +
		((s_pitch * M2 * sin(roll_value) * phi2) / (cos(pitch_value))) + ((s_yaw * M2 * cos(roll_value) * phi2) / (cos(pitch_value)));

	double para1_pitch = C_GAMMAP / (1 + 2 * C_GAMMAP * G0);
	double para1_yaw = C_GAMMAY / (1 + 2 * C_GAMMAY * G0);

	double G1 = s_pitch * M1 * cos(roll_value) * sigma_pitch_value - s_yaw * M1 * sin(roll_value) * sigma_pitch_value +
		((s_pitch * M2 * sin(roll_value) * sigma_yaw_value) / (cos(pitch_value))) 
		+ ((s_yaw * M2 * cos(roll_value) * sigma_yaw_value) / (cos(pitch_value)));

	double para2_pitch = C_KP1 * G1 * d_error_pitch_value + C_KP2 * G1 * error_pitch_value;
	double para2_yaw = C_KY1 * G1 * d_error_yaw_value + C_KY2 * G1 * error_yaw_value;

	double WFP = 0.0;
	double WFY = 0.0;
	double temp_SP = 0.0;
	double temp_SY = 0.0;

	for (int i = 0;i < 9;i++) {
		temp_SP = WEIGHT_VECTOR[i] * BASIS_VECTOR[i];
		temp_SY = WEIGHT_VECTOR[i + 9] * BASIS_VECTOR[i + 9];
		WFP = WFP + temp_SP;
		WFY = WFY + temp_SY;
	}

	double G2 = s_pitch * M1 * cos(roll_value) * WFP - s_yaw * M1 * sin(roll_value) * WFP +
		((s_pitch * M2 * sin(roll_value) * WFY) / (cos(pitch_value)))
		+ ((s_yaw * M2 * cos(roll_value) * WFY) / (cos(pitch_value)));

	double para3_pitch = G2/sigma_pitch_value;
	double para3_yaw = G2/sigma_yaw_value;

	double para4_pitch = C_SAT_MAX_PITCH * C_DELP * tanh(s_pitch / (C_DELP * pow(sigma_pitch_value, 2)));
	double para4_yaw = C_SAT_MAX_YAW*C_DELY* tanh(s_yaw / (C_DELY * pow(sigma_yaw_value, 2)));

	double para5_pitch = pow(s_pitch, 2) / sigma_pitch_value;
	double para5_yaw = pow(s_yaw, 2) / sigma_yaw_value;

	double d_sigma_pitch = para1_pitch * (para2_pitch + para3_pitch + para4_pitch + para5_pitch);
	double d_sigma_yaw = para1_yaw * (para2_yaw + para3_yaw + para4_yaw + para5_yaw);

	double new_sigma_pitch = sigma_pitch_value + d_sigma_pitch * (SAMPLE_TIME);
	double new_sigma_yaw = sigma_yaw_value + d_sigma_yaw * SAMPLE_TIME;

	if (new_sigma_pitch < .1)
		new_sigma_pitch = .1;
	else
		new_sigma_pitch = new_sigma_pitch + 0;

	if (new_sigma_yaw < .1)
		new_sigma_yaw = .1;
	else
		new_sigma_yaw = new_sigma_yaw + 0;

	controller.setSigmaVectors(new_sigma_pitch, new_sigma_yaw);

	INPUT_FLS[0] = pitch_value;
	INPUT_FLS[1] = (pitch_value - LAST_ATTITUDE[1])/ SAMPLE_TIME;
	INPUT_FLS[2] = yaw_value;
	INPUT_FLS[3] = (yaw_value - LAST_ATTITUDE[2]) / SAMPLE_TIME;

	controller.setIT2_FLS(INPUT_FLS, s_pitch,
		s_yaw, WEIGHT_VECTOR, BASIS_VECTOR, roll_value, M1, M2);

	vector<double> temp_FLS(controller.getIT2_FLS());
	vector<double> WF(18, 0);
	vector<double> BAS(18, 0);

	for (int i = 0;i < 36;i++) {
		if (i < 18) {
			WF[i] = temp_FLS[i]*SAMPLE_TIME + WEIGHT_VECTOR[i];
		}
		else {
			BAS[i - 18] = temp_FLS[i];
		}
	}
	controller.setBasisVectors(BAS);
	controller.setWeightVectors(WF);
	controller.setLastAttitude(roll_value, pitch_value, yaw_value);

	TORQUE[0] = t1;
	TORQUE[1] = t2;
}

vector<double> SFSMC::getResult()
{
	return TORQUE;
}