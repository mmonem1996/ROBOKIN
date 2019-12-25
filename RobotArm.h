// Dynamic simulation of robotic arm that include iterative algorithms which
// calculates the forces and torques required to achieve the desired performance.
// derivatives are computed numerically with an adjustable accuracy.
// written by Mohammed Abdelmoneim Hassan



#pragma once
#include "stdafx.h"


//Revolute link
class Link {
protected:
	FLOAT alpha, a, theta, d; //DH Representation
	FLOAT mass,length;
	Vector3 CM;
	Matrix3x3 InertiaTensor;

	//Link* Parent;
public:
	FLOAT Joint_rate, Joint_acceleration;
	Vector3 Link_Angular_Velocity, Link_Angular_Acceleration;
	Vector3 Link_Linear_Velocity, Link_Linear_Acceleration;
	Vector3
		CM_Acceleration, //Center of mass acceleration
		Kinematic_Force, //force accociated with the acceleration of center of mass
		Kinematic_Torque,//torque accociated with the rotation of Link
		Link_Force,
		Link_Torque
		;

	Link();
	Link(const Vector4& DHP, FLOAT mass,FLOAT length);
	void Set_Inertia_Tensor(FLOAT Ixx, FLOAT Iyy, FLOAT Izz, FLOAT Ixy = 0.f, FLOAT Ixz = 0.f, FLOAT Iyz = 0.f);
	virtual FLOAT Get_Variable();
	virtual void Set_Variable(FLOAT value);
	//void SetParent(Link* p) { Parent = p; }
	void Set_CM(Vector3 CenterOfMassVector) { CM = CenterOfMassVector; }
	Matrix4x4 Get_DH_Matrix();
	FLOAT Get_Length()const { return length; }
	virtual void Compute_Velocities(Link* pPrevious_Link = 0 );
	virtual void Compute_Accelerations(Link* pPrevious_Link = 0, Vector3 gravity = Vector3(0,0,0));
	virtual void Compute_Kinematic_Force_And_Torque();
	void Compute_Joint_Force_And_Torque(Link * pNext_Link);
	virtual void Compute_Joint_Force_And_Torque(Vector3 Tip_Location, Vector3 Force= Vector3(0, 0, 0), Vector3 Torque= Vector3(0, 0, 0));
	virtual bool is_Revolute() { return true; }
};
//prismatic link
class TLink : public Link {//Translation Link
public:
	TLink(const Vector4& DHP, FLOAT mass,FLOAT length);
	FLOAT Get_Variable()override;
	void Set_Variable(FLOAT value)override;
	bool is_Revolute()override { return false; }
};

class Arm {
	std::vector<Link*> mLinks;
	FLOAT accuracy;
	bool mPlanar;
	Vector3 _SO_Partial(unsigned int Link_Index1, unsigned int Link_Index2,FLOAT delta);
	Vector3 _Partial(unsigned int Link_Index, FLOAT delta);
	Vector3 mGravity,mTip_Location;
public:
	Arm(FLOAT _accuracy, bool planar = false, Vector3 gravity = Vector3(0,981.f,0)):
		accuracy(_accuracy),
		mPlanar(planar),
		mGravity(gravity){}
	void Set_Tip_Location(Vector3 Tip_Location) { mTip_Location = Tip_Location; }
	void Add_Link(Link* l);
	std::vector<Link*> Get_Links() { return mLinks; }
	MatrixX Get_Variables();
	void Set_Variables(MatrixX vars);
	bool Is_Planner() { return mPlanar; }
	Vector3 Partial_Derivative_Lin(unsigned int Link_Index);
	Vector3 Partial_Derivative_Rot(unsigned int Link_Index);
	Vector3 SO_Partial_Derivative(unsigned int Link_Index1, unsigned int Link_Index2);
	Matrix4x4 Compute_Link_Transformation(unsigned int i, unsigned int j);
	Matrix4x4 Compute_Tip_Transformation(FLOAT x,FLOAT y,FLOAT z);
	Matrix4x4 Compute_Tip_Transformation() { return Compute_Tip_Transformation(mTip_Location.x(), mTip_Location.y(), mTip_Location.z()); }
	void Set_Joints_Rates(std::vector<FLOAT>& Rates);
	void Set_Joints_Accelerations(std::vector<FLOAT>& Joints_acceleration);
	void Inverse_Dynamics(Vector3 Force_At_tip = Vector3(0, 0, 0), Vector3 Torque_At_tip = Vector3(0, 0, 0));
	~Arm();
};