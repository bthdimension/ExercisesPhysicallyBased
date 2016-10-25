//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include "SimpleFEMDefs.h"
#include "FEMElementTri.h"
#include "FEMMesh.h"

double area(double a, double b, double c)
{
	double s = (a + b + c) / 2;
	return sqrt(s*(s - a)*(s - b)*(s - c));
}

// TASK 3
void FEMElementTri::Assemble(FEMMesh *pMesh) const
{
	//find area of element
	Vector2 n1 = pMesh->GetNodePosition(m_nodes[0]);
	Vector2 n2 = pMesh->GetNodePosition(m_nodes[1]);
	Vector2 n3 = pMesh->GetNodePosition(m_nodes[2]);
	double a = abs((n1 - n2).length());
	double b = abs((n2 - n3).length());
	double c = abs((n3 - n1).length());
	double A = area(a, b, c);

	for (int i_local = 0; i_local < 3; i_local++) {
		int i_global = m_nodes[i_local];
		for (int j_local = 0; j_local < 3; j_local++) {			
				int j_global = m_nodes[j_local];
				if (i_global >= j_global) {
					Vector2 basisDerivGlobal_I, basisDerivGlobal_J;
					computeSingleBasisDerivGlobalLES(i_local, basisDerivGlobal_I, pMesh);
					computeSingleBasisDerivGlobalLES(j_local, basisDerivGlobal_J, pMesh);

					//computeSingleBasisDerivGlobalGeom(i_local, basisDerivGlobal_I, pMesh);
					//computeSingleBasisDerivGlobalGeom(j_local, basisDerivGlobal_J, pMesh);

					double value = A*(basisDerivGlobal_I.x()*basisDerivGlobal_J.x() + basisDerivGlobal_I.y()*basisDerivGlobal_J.y());

					pMesh->AddToStiffnessMatrix(i_global, j_global, value);
				}
				
		}
	}
}

// TASK 2
void FEMElementTri::computeSingleBasisDerivGlobalGeom(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const
{
	Vector2 ni = pMesh->GetNodePosition(m_nodes[nodeId]);
	Vector2 n2 = pMesh->GetNodePosition(m_nodes[(nodeId + 1)%3]);
	Vector2 n3 = pMesh->GetNodePosition(m_nodes[(nodeId + 2)%3]);
	double a = abs((ni - n2).length());
	double b = abs((n2 - n3).length());
	double c = abs((n3 - ni).length());

	// find height (from node i to opposite edge)
	double h = 2 * area(a,b,c) / b;
	double invH = 1 / h;
	basisDerivGlobal = Vector2(n3.y() - n2.y(), n3.x() - n2.x()).normalized() * invH;
}

// TASK 1
void FEMElementTri::computeSingleBasisDerivGlobalLES(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const
{
	Vector2 ni = pMesh->GetNodePosition(m_nodes[nodeId]);
	Vector2 n2 = pMesh->GetNodePosition(m_nodes[(nodeId + 1) % 3]);
	Vector2 n3 = pMesh->GetNodePosition(m_nodes[(nodeId + 2) % 3]);

	Matrix3x3 K;
	K(0, 0) = ni.x(); K(0, 1) = ni.y(); K(0, 2) = 1;
	K(1, 0) = n2.x(); K(1, 1) = n2.y(); K(1, 2) = 1;
	K(2, 0) = n2.x(); K(2, 1) = n3.y(); K(2, 2) = 1;

	Vector3 delta = Vector3(1, 0, 0);
	Vector3 abc = K.inverse() * delta;
	// Derivatives of basis function Ni = ax + by + c: Ni_x = a, Ni_y = b
	basisDerivGlobal = Vector2(abc.x(), abc.y());
}
