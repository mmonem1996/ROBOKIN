#pragma once
#include "stdafx.h"

class Arm;

class JRS {
protected:
	FLOAT accuracy;
public:
	/*virtual void Compute_Joints_Rates(Arm* pArm, std::vector<FLOAT>& Rates, const Vector3& tip_speed_Lin, const Vector3& tip_speed_Rot = Vector3(0,0,0)) = 0;*/
	virtual void Compute_Joints_Rates(Arm* pArm, std::vector<FLOAT>& Rates, const Vector3& tip_speed, const Vector3& tip_speed_Rot = Vector3(0, 0, 0)) = 0;
	virtual  bool Check_Validity(Arm* pArm) = 0;
	virtual bool is_Singular(Arm* pArm) = 0;
	virtual ~JRS(){}
};

class JRS_NONE_REDUNDANT :public JRS {
	bool xyzRotations[3];
	MatrixX mJacobian, mJacobianInverse;
public:
	std::vector<size_t> Get_Rotation_Control_Indices()const;
	MatrixX Get_Jacobian()const { return mJacobian; }
	MatrixX Get_Jacobian_Inverse()const { return mJacobianInverse; }
	void Compute_Jacobian(Arm* pArm,bool compute_inverse = true);
	JRS_NONE_REDUNDANT(FLOAT _accuracy);
	void Set_Rotation_Control(bool xyz[3]);
	void Set_Rotation_Control(bool x, bool y, bool z);
	void Compute_Joints_Rates(Arm* pArm, std::vector<FLOAT>& Rates, const Vector3& tip_speed, const Vector3& tip_speed_Rot = Vector3(0, 0, 0))override;
	bool is_Singular(Arm* pArm)override;
	bool Check_Validity(Arm* pArm)override;
};

class IK_Solver {
	JRS_NONE_REDUNDANT* mJrs;
	bool mDLS;
	FLOAT DLS_Lambda;
public:
	IK_Solver(JRS_NONE_REDUNDANT* pJrs);
	void Solve(std::vector<FLOAT>& Joints_Position, Arm * pArm, MatrixX P);
	void Solve(std::vector<FLOAT>& Joints_Position, Arm * pArm, MatrixX P,MatrixX IG);
	void Use_DLS(bool use) { mDLS = use; }
};