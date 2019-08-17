#include "Joints_Solvers.h"
#include "RobotArm.h"

using namespace Eigen;

std::vector<size_t> JRS_NONE_REDUNDANT::Get_Rotation_Control_Indices() const
{
	std::vector<size_t> indices;
	for (size_t i = 0; i < 3; i++)
	{
		if (xyzRotations[i])
			indices.push_back(i);
	}
	return indices;
}

void JRS_NONE_REDUNDANT::Compute_Jacobian(Arm * pArm, bool compute_inverse)
{
	MatrixX  Jacobian;
	auto n = pArm->Get_Links().size();
	Jacobian.resize(n, n);
	std::vector<size_t>inds;
	for (size_t i = 0; i < 3; i++)
	{
		if (xyzRotations[i])
		{
			inds.push_back(i);
		}
	}
	for (size_t j = 0; j < Jacobian.cols(); j++)
	{
		auto pdl = pArm->Partial_Derivative_Lin(j);
		auto pdr = pArm->Partial_Derivative_Rot(j);
		for (size_t i = 0; i < Jacobian.rows(); i++)
		{
			if (i < 3)
			{
				Jacobian(i, j) = pdl(i, 0);
			}
			else
			{
				Jacobian(i, j) = pdr(inds[i - 3], 0);
			}
		}
	}
	mJacobian = Jacobian;
	if (compute_inverse)
	{
		if (!is_Singular(pArm))
			mJacobianInverse = mJacobian.inverse();
	}
	
}

JRS_NONE_REDUNDANT::JRS_NONE_REDUNDANT(FLOAT _accuracy)
{
	accuracy = _accuracy;
	for (size_t i = 0; i < 3; i++)
	{
		xyzRotations[i] = true;
	}
	 
}

void JRS_NONE_REDUNDANT::Set_Rotation_Control(bool xyz[3])
{
	for (size_t i = 0; i < 3; i++)
	{
		xyzRotations[i] = xyz[i];
	}
}

void JRS_NONE_REDUNDANT::Set_Rotation_Control(bool x, bool y, bool z)
{
	bool xyz[3] = { x,y,z };
	Set_Rotation_Control(xyz);
}

bool JRS_NONE_REDUNDANT::is_Singular(Arm * pArm)
{
	if (fabs(mJacobian.determinant()) <= 0.001)
		return true;
	return false;
}

void JRS_NONE_REDUNDANT::Compute_Joints_Rates(Arm * pArm, std::vector<FLOAT>& Rates, const Vector3& tip_speed_Lin, const Vector3& tip_speed_Rot)
{
	Rates.clear();
	MatrixX speeds_vec;
	speeds_vec.resize(pArm->Get_Links().size(), 1);
	for (size_t i = 0; i < 3; i++)
	{
		speeds_vec(i, 0) = tip_speed_Lin(i,0);
	}
	std::vector<size_t>inds;
	for (size_t i = 0; i < 3; i++)
	{
		if (xyzRotations[i])
		{
			inds.push_back(i);
		}
	}
	for (size_t i = 0; i < inds.size(); i++)
	{
		speeds_vec(i + 3, 0) = tip_speed_Rot(inds[i], 0);
	}
	mJacobianInverse = mJacobian.inverse();
	auto Joints_Speed_Vec = mJacobianInverse*speeds_vec;
	for (size_t i = 0; i < Joints_Speed_Vec.rows(); i++)
	{
		Rates.push_back(Joints_Speed_Vec(i, 0));
	}
}

bool JRS_NONE_REDUNDANT::Check_Validity(Arm * pArm)
{
	auto n = 3;
	for (size_t i = 0; i < 3; i++)
	{
		if (xyzRotations[i])
		{
			n++;
		}
	}
	if (n != pArm->Get_Links().size())
	{
		return false;
	}
	return true;
}

IK_Solver::IK_Solver(JRS_NONE_REDUNDANT * pJrs)
{
	mJrs = pJrs;
	mDLS = false;
	DLS_Lambda = 1.5;
}

void IK_Solver::Solve( std::vector<FLOAT>& Joints_Position, Arm * pArm, MatrixX P)
{
	auto ig = P;
	for (size_t i = 0; i < ig.rows(); i++)
	{
		ig(i, 0) = 0;
	}
	Solve(Joints_Position, pArm, P, ig);
}

FLOAT Clamp_Angle_in_2PI_Range(FLOAT Angle) {
	float angle_in_range = Angle;
	while (angle_in_range >= 2*PI)
	{
		angle_in_range -= 2 * PI;
	}
	while (angle_in_range < 0 )
	{
		angle_in_range += 2 * PI;
	}
	return angle_in_range;
}

void IK_Solver::Solve(std::vector<FLOAT>& Joints_Position, Arm * pArm, MatrixX P, MatrixX IG)
{
	Joints_Position.clear();
	MatrixX e = P - IG,t;
	auto vars = pArm->Get_Variables(),V = vars;
	FLOAT  error = e.norm(),lerror=error;
	e.normalize();
	t = IG;
	FLOAT div = 1;
	while (error > 0.001)
	{
		MatrixX dP = (P - t) / div;
		MatrixX dV;
		if (!mDLS)
		{
			mJrs->Compute_Jacobian(pArm);

			if (mJrs->is_Singular(pArm)) {
				Throw_Error("cannot solve based on the provided initial guess\n");
				pArm->Set_Variables(vars);
				return;
			}
			dV = mJrs->Get_Jacobian_Inverse()*dP;
		}
		else
		{
			mJrs->Compute_Jacobian(pArm,false);
			auto J = mJrs->Get_Jacobian();
			MatrixX JJT = J*J.transpose();
			auto I = JJT;
			I.setIdentity();
			MatrixX Ji = J.transpose()*((JJT + (DLS_Lambda*DLS_Lambda*I)).inverse());
			dV = Ji*dP;
		}
		V += dV;
		pArm->Set_Variables(V);
		auto T = pArm->Compute_Tip_Transformation();
		AngleAxis<FLOAT> _AA(Get_Rotation(T));
		for (size_t i = 0; i < 3; i++)
		{
			t(i, 0) = T(i, 3);
		}
		auto cis = mJrs->Get_Rotation_Control_Indices();
		Vector3 rot = _AA.angle()*_AA.axis();
		for (auto index : cis) {
			t(index + 3, 0) = rot(index, 0);
		}
		e = P - t;
		error = e.norm();
		e.normalize();
		if (lerror <= error) {
			div *= 2;
			V -= dV;
			pArm->Set_Variables(V);
			T = pArm->Compute_Tip_Transformation();
			AngleAxis<FLOAT> _AA(Get_Rotation(T));
			for (size_t i = 0; i < 3; i++)
			{
				t(i, 0) = T(i, 3);
			}
			cis = mJrs->Get_Rotation_Control_Indices();
			rot = _AA.angle()*_AA.axis();
			for (auto index : cis) {
				t(index + 3, 0) = rot(index, 0);
			}
			error = lerror;
		}
		else
		{
			div = 1;
			lerror = error;
		}
	}
	auto test = Joints_Position;
	for (size_t i = 0; i < V.rows(); i++)
	{
		V(i, 0) = Clamp_Angle_in_2PI_Range(V(i, 0));
		Joints_Position.push_back(V(i, 0));
		test.push_back(t(i, 0));
	}
	pArm->Set_Variables(vars);
}
