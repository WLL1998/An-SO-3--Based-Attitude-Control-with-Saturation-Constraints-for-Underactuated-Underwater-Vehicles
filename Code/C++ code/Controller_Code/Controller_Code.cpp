// Controller_Code.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "SFSMC.h"
#define PI 3.1415926
#include<string>
#include<typeinfo>
#include<iostream>
#include<winsock.h>
#include<sstream>
#include<time.h>
#pragma comment(lib,"ws2_32.lib")
using namespace std;

void initialization() {
	//初始化套接字库
	WORD w_req = MAKEWORD(2, 2);//版本号
	WSADATA wsadata;
	int err;
	err = WSAStartup(w_req, &wsadata);
	if (err != 0) {
		cout << "初始化套接字库失败!" << endl;
	}
	else {
		cout << "初始化套接字库成功！" << endl;
	}
	//检测版本号
	if (LOBYTE(wsadata.wVersion) != 2 || HIBYTE(wsadata.wHighVersion) != 2) {
		cout << "套接字库版本号不符！" << endl;
		WSACleanup();
	}
	else {
		cout << "套接字符版本号正确！" << endl;
	}
}

int main()
{

    SFSMC ctrl;
    vector<double> Controller_value = { 0.0,0.0 };
	//定义长度变量
	int send_len = 0;
	int recv_len = 0;
	//定义发送缓冲区和接受缓冲区
	double roll_current_value = 0;
	double pitch_current_value = 0;
	double yaw_current_value = 0;
	double torque_pitch = 0.0;
	double torque_yaw = 0.0;
	double data[5] = { roll_current_value, pitch_current_value, yaw_current_value, torque_pitch, torque_yaw };
	char send_buf[100] = { 'a' };
	char recv_buf[100] = { 'a' };
	stringstream ss;
	stringstream ss1;
	string str = "";
	string temp = "";
	int len = 0;
	int pos = 0;
	int j = 0;
	int num = 0;
	int num1 = 0;
	int p2 = 0;
	double convert = 0;
	char temp1[10] = { '\0' };

    ctrl.setPitchController_Parameters(15.0, 1.4, 0.009, 0.1, 0.1, 0.01, 0.7);
    ctrl.setYawController_Parameters(15.0, 1.4, 0.009, 0.1, 0.1, 0.01, 0.7);
    ctrl.setDesiredAttitude(20*PI/180, 20*PI/180);
	ctrl.setLastAttitude(0.0, 0.0,0.0);
	ctrl.setErrorLast(0.0, 0.0);

	//定义服务端套接字，接受请求套接字
	SOCKET s_server;
	//服务端地址客户端地址
	SOCKADDR_IN server_addr;
	initialization();
	//填充服务端信息
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	server_addr.sin_port = htons(777);
	//创建套接字
	s_server = socket(AF_INET, SOCK_STREAM, 0);
	if (connect(s_server, (SOCKADDR*)&server_addr, sizeof(SOCKADDR)) == SOCKET_ERROR) {
		cout << "服务器连接失败！" << endl;
		WSACleanup();
	}
	else {
		cout << "服务器连接成功！" << endl;
	}



	//发送数据
	while (1) {
		ctrl.setCurrentAttitude(data[0], data[1], data[2]);
		len = 0; //用于计算数据的长度
		pos = 0; //将每个数据存入send_buf的对应下标
		j = 0; // 下标的指针位置
		num = 5; //状态的数量
		p2 = 0; //读取数据时，用于指示double数据的位置
		num1 = 0; //用于计算数据的数量
		convert = 0; //用于暂存数据的变量
		ctrl.ControllerOutput(ctrl);

		Controller_value = ctrl.getResult();
		data[3] = Controller_value[0];
		data[4] = Controller_value[1];
		//将double变为send_buf[100]
		for (int i = 0;i < num;i++) {
			ss << data[i];
			temp = ss.str(); //当前变量暂存在temp中
			len = temp.length(); //计算当前变量的位数
			len = len + pos; //计算当前send_buf的长度，pos为当前指针的位置
			for (j = pos;j < len;j++) {
				send_buf[j] = temp[j - pos];
			}
			if (i < num - 1) {
				send_buf[j] = ','; //除最后一位外，两个变量间加一个","
			}
			pos = j + 1; //后移一位
			temp = ""; //清空temp
			ss.str("");
			ss.clear();
		}

		send_len = send(s_server, send_buf, sizeof(send_buf), 0);


		if (send_len < 0) {
			cout << "发送失败！" << endl;
			break;
		}
		else {
			cout << "发送成功:" << send_buf << endl;
		}

		for (int b = 0;b < 100;b++) {
			send_buf[b] = '\0';
		}
		Sleep(100);
		recv_len = recv(s_server, recv_buf, sizeof(recv_buf), 0);
		if (recv_len < 0) {
			cout << "接受失败！" << endl;
			break;
		}
		else {
			cout << "服务端信息：" << recv_buf << endl;
		}
		Sleep(400);
		// 将recv_buf转换为double 数组
		for (int p1 = 0;p1 < 100;p1++) {
			if (recv_buf[p1] == '\0' || recv_buf[p1] == 's') {
				break; //当读到空位时，不再读取数据
			}
			else {
				if (recv_buf[p1] != ',') {
					temp1[p2] = recv_buf[p1];
					p2 = p2 + 1;
					//当指针变量非","数据，将对应的数据存入temp1,并后移一位
				}
				else {
					//当指针变量读到","，将temp1转换为double并存入result中
					p2 = 0;
					ss1 << temp1;
					for (int m = 0;m < 10;m++) {
						temp1[m] = 0;
					}
					ss1 >> convert;
					data[num1] = convert;
					convert = 0;
					num1 = num1 + 1;
					ss1.str("");
					ss1.clear(); //清空
				}
			}
		}
		cout << "结果为:" << data[0] << "," << data[1] << "," << data[2] << endl;
		for (int b = 0;b < 100;b++) {
			recv_buf[b] = '\0';
		}

	}
	//关闭套接字
	closesocket(s_server);
	//释放DLL资源
	WSACleanup();

	return 0;
}


// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
