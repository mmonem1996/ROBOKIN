//Test program for the Arm Class.
//forces unit = kg.cm/s2
//torque unit = kg.cm2/s2
//output unit will be consistant with values setted for :inertia tensor,mass,length,gravity,
//tip speed and tip acceleration.
#include "stdafx.h"
#include "RobotArm.h"
#include "Joints_Solvers.h"
using namespace Eigen;
int main()
{
	//30,0,0,26.2
	FLOAT a2 = 30, a3 = 30;
	Arm MyArm(0.001,false,Vector3(0,0, 981));//for now accuracy below 0.01f is not supported
	std::vector<Link*> links =
	{
		new Link(Vector4(toRadians(0),0,0,0),1.28f,00),
		new Link(Vector4(toRadians(90),0,0,0),1.25f,0),
		new Link(Vector4(toRadians(0),a2,0,0),1.77f,0),
		new Link(Vector4(toRadians(0),a3,0,0),0.315f,0),
		new Link(Vector4(toRadians(90),0,0,0),0.07792f,0)
	};
	/*links[0]->Set_Inertia_Tensor(0.01347f*10000, 0.00596f*10000, 0.01391f*10000, 0, 0, 0.00188f*10000);
	links[1]->Set_Inertia_Tensor(0.00442f*10000, 0.04116f*10000, 0.03806f*10000,0, 0.00677f*10000, 0);
	links[2]->Set_Inertia_Tensor(0.0008774f*10000, 0.00048444f*10000, 0.0008533f*10000);
	links[3]->Set_Inertia_Tensor(0.00038387f*10000, 0.00390402f*10000, 0.00379697f*10000,0,-0.0003343f*10000,0);
	links[4]->Set_Inertia_Tensor(0.00037f*10000, 0.0001f*10000, 0.00039f*10000);*/

	links[0]->Set_CM(Vector3(0, 0.05f, -0.01788f) * 100);
	links[1]->Set_CM(Vector3(0, 0.226, -0.04f) * 100);
	links[2]->Set_CM(Vector3(0.136, 0, -0.00738) * 100);
	links[3]->Set_CM(Vector3(0, 0, -0.006f) * 100);
	links[4]->Set_CM(Vector3(0, 1.73f, 0) * 100);

	for each (auto var in links)
	{
		MyArm.Add_Link(var);
	}
	Vector3 tip_speed(100, 0, 0);
	
	

	std::vector<FLOAT> rates = { 0,0,0,0,0 };
	std::vector<FLOAT> accelerations = { 0,0,0,0,0 };
	//solver.Set_Rotation_Control(false,true,true);
	//MyArm.Set_Tip_Location(Vector3(0, 0, 6));
	/*auto T = MyArm.Compute_Tip_Transformation();
	auto tt = T(0, 3);
	JRS_NONE_REDUNDANT solver(0.01);
	IK_Solver ik(&solver);
	
	MatrixX pos;
	pos.resize(6, 1);

	for (int i = 0; i < 6; i++)
		pos(i, 0) = 0;
	pos(0, 0) = 40.0;
	std::vector<FLOAT> ppp;
	ik.Use_DLS(true);
	ik.Solve(ppp, &MyArm, pos);
	solver.Compute_Jacobian(&MyArm);
	if (solver.Check_Validity(&MyArm) && !solver.is_Singular(&MyArm))
	{
		solver.Compute_Joints_Rates(&MyArm, rates, tip_speed);
	}*/
	
	MyArm.Set_Joints_Rates(rates);
	MyArm.Set_Joints_Accelerations(accelerations);
	MyArm.Inverse_Dynamics();
	for each (auto var in MyArm.Get_Links())
	{
		static auto In = 1u;
		printf("Link %d\t joint rate = %0.2f Rad/sec\tjoint Torque = %0.2f N.m\n", In, var->Joint_rate,var->Link_Torque.z()/10000);
		In++;
	}
	system("Pause");
	return 0;
}

