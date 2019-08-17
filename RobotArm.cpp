#include "RobotArm.h"
using namespace Eigen;

Link::Link():mass(1),alpha(0),a(10),theta(0),d(0),CM(5,0,0),length(10)
{
	Set_Inertia_Tensor(1, 9, 9);
}

Link::Link(const Vector4& DHP, FLOAT m, FLOAT l): length(l),mass(m),alpha(DHP.x()), a(DHP.y()), d(DHP.z()), theta(DHP.w()), CM(l/2, 0, 0)
{
	Set_Inertia_Tensor(10, 90, 90);
}

void Link::Set_Inertia_Tensor(FLOAT Ixx, FLOAT Iyy, FLOAT Izz, FLOAT Ixy, FLOAT Ixz, FLOAT Iyz)
{
	InertiaTensor(0,0) = Ixx;
	InertiaTensor(1,1) = Iyy;
	InertiaTensor(2,2) = Izz;
	InertiaTensor(1,0) = InertiaTensor(0,1) = -Ixy;
	InertiaTensor(2,0) = InertiaTensor(0,2) = -Ixz;
	InertiaTensor(1,2) = InertiaTensor(2,1) = -Iyz;
}

FLOAT Link::Get_Variable()
{
	return theta;
}

void Link::Set_Variable(FLOAT value)
{
	theta = value;
}

Matrix4x4 Link::Get_DH_Matrix()
{
	Matrix4x4 mat;
	
	mat << cos(theta), (FLOAT)-sin(theta), 0, a,
		(FLOAT)sin(theta)*(FLOAT)cos(alpha), (FLOAT)cos(theta)*(FLOAT)cos(alpha), (FLOAT)-sin(alpha), (FLOAT)-sin(alpha)*d,
		(FLOAT)sin(theta)*(FLOAT)sin(alpha), (FLOAT)cos(theta)*(FLOAT)sin(alpha), (FLOAT)cos(alpha), (FLOAT)cos(alpha)*d,
		0, 0, 0, 1;
	return mat;
}

void Link::Compute_Velocities(Link* pPrevious_Link)
{
	auto DHM = Get_DH_Matrix();
	auto iP = Vector3(DHM(0, 3), DHM(1, 3), DHM(2, 3));

	Matrix3x3 toLocal = Get_Rotation(DHM.inverse());
	
	Link_Angular_Velocity = Vector3(0, 0, Joint_rate);
	Link_Linear_Velocity = Vector3(0, 0, 0);
	if (pPrevious_Link) {
		Link_Angular_Velocity = Link_Angular_Velocity + toLocal* pPrevious_Link->Link_Angular_Velocity;
		auto v = pPrevious_Link->Link_Linear_Velocity + pPrevious_Link->Link_Angular_Velocity.cross(iP);
		Link_Linear_Velocity = toLocal*v;
	}
}

void Link::Compute_Accelerations(Link * pPrevious_Link, Vector3 gravity)
{
	auto DHM = Get_DH_Matrix();
	auto iP = Vector3(DHM(0, 3), DHM(1, 3), DHM(2, 3));
	
	Matrix3x3 toLocal = Get_Rotation(DHM.inverse());
	Link_Angular_Acceleration = Vector3(0, 0, Joint_acceleration);
	if (!pPrevious_Link)
		Link_Linear_Acceleration = toLocal* gravity;
	else
	{
		Link_Angular_Acceleration = Link_Angular_Acceleration + toLocal* pPrevious_Link->Link_Angular_Acceleration;
		auto prv_ang_speed = toLocal*pPrevious_Link->Link_Angular_Velocity;
		Link_Angular_Acceleration = Link_Angular_Acceleration + prv_ang_speed.cross(Vector3(0, 0, Joint_rate));
		auto prv_l =
			pPrevious_Link->Link_Angular_Acceleration.cross(iP) +
			pPrevious_Link->Link_Angular_Velocity.cross(pPrevious_Link->Link_Angular_Velocity.cross(iP)) +
			pPrevious_Link->Link_Linear_Acceleration;
		Link_Linear_Acceleration = toLocal* prv_l;
	}
	CM_Acceleration = 
		Link_Angular_Acceleration.cross(CM) +
		Link_Angular_Velocity.cross(Link_Angular_Velocity.cross(CM)) +
		Link_Linear_Acceleration;
}

void Link::Compute_Kinematic_Force_And_Torque()
{
	Kinematic_Force = mass * CM_Acceleration;
	Vector3 ang_momentum = InertiaTensor* Link_Angular_Velocity;
	
	Kinematic_Torque =
		InertiaTensor*Link_Angular_Acceleration +
		Link_Angular_Velocity.cross(ang_momentum)
		;
}

void Link::Compute_Joint_Force_And_Torque(Link * pNext_Link)
{
	if (pNext_Link)
	{
		auto DHM = pNext_Link->Get_DH_Matrix();
		auto iP = Vector3(DHM(0, 3), DHM(1, 3), DHM(2, 3));
		Matrix3x3 toLocal = Get_Rotation(DHM);
		Link_Force = toLocal*pNext_Link->Link_Force
			+ Kinematic_Force
			;
		Link_Torque = Kinematic_Torque
			+ toLocal* pNext_Link->Link_Torque
			+ CM.cross(Kinematic_Force)
			+ iP.cross(toLocal* pNext_Link->Link_Force)
			;
	}
}

void Link::Compute_Joint_Force_And_Torque(Vector3 Tip_Location, Vector3 Force, Vector3 Torque )
{
	Link_Force = Force
		+ Kinematic_Force;
	Link_Torque = Kinematic_Torque
		+ Torque
		+ CM.cross(Kinematic_Force)
		+ Tip_Location.cross(Force)
		;
}

Vector3 Arm::_SO_Partial(unsigned int Link_Index1, unsigned int Link_Index2, FLOAT delta)
{
	if (Link_Index1 != Link_Index2)
	{
		FLOAT
			Variable1 = mLinks[Link_Index1]->Get_Variable(),
			Variable2 = mLinks[Link_Index2]->Get_Variable();
		mLinks[Link_Index1]->Set_Variable(Variable1 + delta);
		mLinks[Link_Index2]->Set_Variable(Variable2 + delta);
		auto f11 = Compute_Tip_Transformation();
		mLinks[Link_Index1]->Set_Variable(Variable1 + delta);
		mLinks[Link_Index2]->Set_Variable(Variable2 - delta);
		auto f1_1 = Compute_Tip_Transformation();
		mLinks[Link_Index1]->Set_Variable(Variable1 - delta);
		mLinks[Link_Index2]->Set_Variable(Variable2 + delta);
		auto f_11 = Compute_Tip_Transformation();
		mLinks[Link_Index1]->Set_Variable(Variable1 - delta);
		mLinks[Link_Index2]->Set_Variable(Variable2 - delta);
		auto f_1_1 = Compute_Tip_Transformation();
		FLOAT x = f11(0, 3) - f1_1(0, 3) - f_11(0, 3) + f_1_1(0, 3);
		FLOAT y = f11(1, 3) - f1_1(1, 3) - f_11(1, 3) + f_1_1(1, 3);
		FLOAT z = f11(2, 3) - f1_1(2, 3) - f_11(2, 3) + f_1_1(2, 3);
		mLinks[Link_Index1]->Set_Variable(Variable1);
		mLinks[Link_Index2]->Set_Variable(Variable2);
		return Vector3(x / (4 * delta*delta), y / (4 * delta*delta), z / (4 * delta*delta));
	}
	else
	{
		FLOAT Variable1 = mLinks[Link_Index1]->Get_Variable();
		mLinks[Link_Index1]->Set_Variable(Variable1 + delta);
		
		auto f1 = Compute_Tip_Transformation();
		mLinks[Link_Index1]->Set_Variable(Variable1);
		auto f0 = Compute_Tip_Transformation();
		mLinks[Link_Index1]->Set_Variable(Variable1 - delta);
		auto f_1 = Compute_Tip_Transformation();
		FLOAT x = f1(0, 3) - 2 * f0(0, 3) + f_1(0, 3);
		FLOAT y = f1(1, 3) - 2 * f0(1, 3) + f_1(1, 3);
		FLOAT z = f1(2, 3) - 2 * f0(2, 3) + f_1(2, 3);
		mLinks[Link_Index1]->Set_Variable(Variable1);
		return Vector3(x / ( delta*delta), y / ( delta*delta), z / ( delta*delta));
	}
}

Vector3 Arm::_Partial(unsigned int Link_Index, FLOAT delta)
{
	FLOAT LinkVariable = mLinks[Link_Index]->Get_Variable();
	mLinks[Link_Index]->Set_Variable(LinkVariable + delta / 2.f);
	auto T2 = Compute_Tip_Transformation();
	mLinks[Link_Index]->Set_Variable(LinkVariable - delta / 2.f);
	auto T1 = Compute_Tip_Transformation();
	mLinks[Link_Index]->Set_Variable(LinkVariable);
	auto dT = (T2 - T1) / delta;
	auto derivative = Vector3(dT(0, 3), dT(1, 3), dT(2, 3));
	return derivative;
}

void Arm::Add_Link(Link * l)
{
	mLinks.push_back(l);
}

MatrixX Arm::Get_Variables()
{
	MatrixX vars;
	vars.resize(mLinks.size(), 1);
	for (auto link : mLinks) {
		static int i = 0;
		vars(i,0)=link->Get_Variable();
		i++;
	}
	return vars;
}

void Arm::Set_Variables(MatrixX vars)
{
	for (size_t i = 0; i < mLinks.size(); i++)
	{
		mLinks[i]->Set_Variable(vars(i, 0));
	}
}

Vector3 Arm::Partial_Derivative_Lin(unsigned int Link_Index)
{
	//numerically compute partial derivative for x,y,z with respect to specified link variable
	
	static FLOAT delta = 0.05f;
	FLOAT error = 0.1;
	auto pd = _Partial(Link_Index, delta);
	while (error > accuracy)
	{
		delta /= 2.f;
		auto pdt = _Partial(Link_Index, delta);
		error = Max_Of({ (FLOAT)fabs(pdt.x() - pd.x()),(FLOAT)fabs(pdt.y() - pd.y()),(FLOAT)fabs(pdt.z() - pd.z()) });
		pd = pdt;
	}
	delta *= 2.0;
	return pd;
}

Vector3 Arm::Partial_Derivative_Rot(unsigned int Link_Index)
{
	Vector3 infinitesimal_rotations(0, 0, 0);
	if (mLinks[Link_Index]->is_Revolute())
		infinitesimal_rotations = Get_Rotation(Compute_Link_Transformation(0, Link_Index))*Vector3(0, 0, 1);
	return infinitesimal_rotations;
}

Vector3 Arm::SO_Partial_Derivative(unsigned int Link_Index1, unsigned int Link_Index2)
{
	FLOAT delta = 0.05;
	FLOAT error = 0.1;
	Vector3 pd2 = _SO_Partial(Link_Index1, Link_Index2, delta);
	while (error > accuracy)
	{
		delta /= 2.0;
		Vector3 pd2t = _SO_Partial(Link_Index1, Link_Index2, delta);
		error = Max_Of({ (FLOAT)fabs(pd2t.x() - pd2.x()),(FLOAT)fabs(pd2t.y() - pd2.y()),(FLOAT)fabs(pd2t.z() - pd2.z()) });
		pd2 = pd2t;
	}
	
	return pd2;
}

Matrix4x4 Arm::Compute_Link_Transformation(unsigned int i, unsigned int j)
{
	Matrix4x4 T;
	T.setIdentity();
	for (size_t k = i; k <= j; k++)
	{
		T *= mLinks[k]->Get_DH_Matrix();
	}
	return T;
}

Matrix4x4 Arm::Compute_Tip_Transformation(FLOAT x,FLOAT y,FLOAT z)
{
	Matrix4x4 T,Tn;
	T.setIdentity();
	for (auto ln : mLinks) {
		T *= ln->Get_DH_Matrix();
	}
	Tn << 
		1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1;
	T *= Tn;
	
	return T;
}

void Arm::Set_Joints_Rates(std::vector<FLOAT>& Rates)
{
	int i = 0;
	for (auto link : mLinks) {
		link->Joint_rate = Rates[i];
		i++;
	}
}

void Arm::Set_Joints_Accelerations(std::vector<FLOAT>& Joints_acceleration)
{
	int i = 0;
	for (auto link : mLinks) {
		link->Joint_acceleration = Joints_acceleration[i];
		i++;
	}
}

void Arm::Inverse_Dynamics(Vector3 Force_At_tip, Vector3 Torque_At_tip)
{
	mLinks[0]->Compute_Velocities();
	mLinks[0]->Compute_Accelerations(0, mGravity);
	mLinks[0]->Compute_Kinematic_Force_And_Torque();
	for (int i = 1; i < (int)mLinks.size(); i++)
	{
		mLinks[i]->Compute_Velocities(mLinks[i - 1]);
		mLinks[i]->Compute_Accelerations(mLinks[i - 1]);
		mLinks[i]->Compute_Kinematic_Force_And_Torque();
	}
	mLinks[mLinks.size() - 1]->Compute_Joint_Force_And_Torque( mTip_Location,Force_At_tip, Torque_At_tip);
	for (int i = (int)mLinks.size() - 2; i > -1; i--)
	{
		mLinks[i]->Compute_Joint_Force_And_Torque(mLinks[i + 1]);
	}
}

Arm::~Arm()
{
	for (auto l : mLinks)
		delete l;
}

TLink::TLink(const Vector4& DHP, FLOAT mass,FLOAT l) :Link(DHP, mass,l)
{}

FLOAT TLink::Get_Variable()
{
	return d;
}

void TLink::Set_Variable(FLOAT value)
{
	d = value;
}

