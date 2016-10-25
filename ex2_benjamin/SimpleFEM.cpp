//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include "SimpleFEMDefs.h"
#include "SimpleFEM.h"
#include "MeshViewer.h"

// size of grid
static const int gridSize = 20;
// use a graded mesh, or a regular mesh
static const bool gradedMesh = true;
// laplace or poisson problem?
static const bool laplaceProblem = false;
// display debug information?
static const bool debugOut = false;


double eval_u(double x, double y)
{
	if (laplaceProblem)
		return exp(x) * sin(y);
	else
		return 	3 * x * x + 2 * y * y * y * x;
}

double eval_f(double x, double y)
{
	if (laplaceProblem)
		return 0;
	else
		return -6 - 12 * y * x;
}

FEMMesh SimpleFEM::CreateUniformGridMesh(int nodesX, int nodesY)
{
	assert(nodesX >= 2);
	assert(nodesY >= 2);

	FEMMesh mesh;

	// Setup positions of nodes
	int nodecnt = 0;
	for (int y = 0; y < nodesY; y++)
	{
		for (int x = 0; x < nodesX; x++)
		{
			Vector2 pos = Vector2((double)x / (double)(nodesX - 1), (double)y / (double)(nodesY - 1));

			// Shift mesh positions for graded mesh
			if (gradedMesh)
			{
				pos[0] *= pos[0];
				pos[1] *= pos[1];
			}
			mesh.AddNode(pos);
			nodecnt++;
		}
	}
	std::cout << "Added " << nodecnt << " nodes to mesh.\n";

	// Create elements
	int cnt = 0;
	for (int y = 0; y < nodesY - 1; y++)
	{
		for (int x = 0; x < nodesX - 1; x++)
		{
			// bottom-left:
			int node00 = y*nodesX + x;

			// bottom-right:
			int node10 = node00 + 1;

			// top-left:
			int node01 = node00 + nodesX;

			// top-right:
			int node11 = node00 + nodesX + 1;

			// add two element for this quad
			mesh.AddElement(FEMElementTri(node00, node10, node11));
			mesh.AddElement(FEMElementTri(node00, node11, node01));
			cnt += 2;
		}
	}
	std::cout << "Added " << cnt << " elements to mesh.\n";

	return mesh;
}

void SimpleFEM::ComputeBoundaryConditions(const FEMMesh &mesh, std::vector<BoundaryCondition> &boundaryConditions)
{
	boundaryConditions.clear();

	for (int nodeID = 0; nodeID < mesh.GetNumNodes(); nodeID++)
	{
		const Vector2 &pos = mesh.GetNodePosition(nodeID);

		if (isOnBoundary(pos)) {
			double x = pos[0];
			double y = pos[1];

			// compute reference solution on boundary
			double val = eval_u(x, y);

			// this fixes the solution for node "nodeID" to "val" when
			// solving the system later on
			boundaryConditions.push_back(BoundaryCondition(nodeID, val));
		}
	}
}

// perform a simple boundary check
// is either of the components 0 or 1?
bool SimpleFEM::isOnBoundary(const Vector2 &pos)
{
	return pos[0] <= 0. || pos[0] >= 1. || pos[1] <= 0. || pos[1] >= 1.;
}

double getArea(double a, double b, double c)
{
	double s = (a + b + c) / 2;
	return sqrt(s*(s - a)*(s - b)*(s - c));
}

double computeBasisFunction(int nodeId, const FEMMesh *mesh, const FEMElementTri *elem, double x, double y)
{
	Vector2 ni = mesh->GetNodePosition(elem->GetGlobalNodeForElementNode(nodeId));
	Vector2 n2 = mesh->GetNodePosition(elem->GetGlobalNodeForElementNode((nodeId + 1) % 3));
	Vector2 n3 = mesh->GetNodePosition(elem->GetGlobalNodeForElementNode((nodeId + 2) % 3));

	Matrix3x3 K;
	K(0, 0) = ni.x(); K(0, 1) = ni.y(); K(0, 2) = 1;
	K(1, 0) = n2.x(); K(1, 1) = n2.y(); K(1, 2) = 1;
	K(2, 0) = n3.x(); K(2, 1) = n3.y(); K(2, 2) = 1;

	Vector3 delta = Vector3(1, 0, 0);
	Vector3 abc = K.inverse() * delta;
	
	return abc.x()*x + abc.y()*y + abc.z();
}

// TASK 4
void SimpleFEM::ComputeRHS(const FEMMesh &mesh, std::vector<double> &rhs)
{
	for (int ie = 0; ie < mesh.GetNumElements(); ie++)
	{
		const FEMElementTri& elem = mesh.GetElement(ie);

		//Task4 starts here
		
		// find centroid
		Vector2 n1 = mesh.GetNodePosition(elem.GetGlobalNodeForElementNode(0));
		Vector2 n2 = mesh.GetNodePosition(elem.GetGlobalNodeForElementNode(1));
		Vector2 n3 = mesh.GetNodePosition(elem.GetGlobalNodeForElementNode(2));
		Vector2 centroid = (n1 + n2 + n3) / 3;
		double f_center = eval_f(centroid.x(), centroid.y());

		// get area of elemnt
		double a = abs((n1 - n2).length());
		double b = abs((n2 - n3).length());
		double c = abs((n3 - n1).length());
		double A = getArea(a, b, c);

		// get N
		double N1 = computeBasisFunction(0, &mesh, &elem, centroid.x(), centroid.y());
		double N2 = computeBasisFunction(1, &mesh, &elem, centroid.x(), centroid.y());
		double N3 = computeBasisFunction(2, &mesh, &elem, centroid.x(), centroid.y());

		// Add contribution of element to fi
		rhs[elem.GetGlobalNodeForElementNode(0)] += f_center * N1 * A; // fi += fe(xq,yq)*Ni(xq,yq)*Ae
		rhs[elem.GetGlobalNodeForElementNode(1)] += f_center * N2 * A; // fi += fe(xq,yq)*Ni(xq,yq)*Ae
		rhs[elem.GetGlobalNodeForElementNode(2)] += f_center * N3 * A; // fi += fe(xq,yq)*Ni(xq,yq)*Ae

		//Task4 ends here
	}
}

// TASK 5
void SimpleFEM::computeError(FEMMesh &mesh, const std::vector<double> &sol_num, std::vector<double> &verror, double &err_nrm)
{
	//Task 5 starts here
	
	//Task 5 ends here
}

int main(int argc, char *argv[])
{
	// Create a uniform mesh:
	FEMMesh mesh = SimpleFEM::CreateUniformGridMesh(gridSize, gridSize);
	int nNodes = mesh.GetNumNodes();

	// Build its stiffness matrix:
	// loop over all elements, and compute their contributions
	// for the equations of their respective nodes
	mesh.getMat().ClearResize(mesh.GetNumNodes());
	for (int i = 0; i < mesh.GetNumElements(); i++)
	{
		if (debugOut)
			std::cout << "Assembling " << i << "\n";
		mesh.GetElement(i).Assemble(&mesh);
	}

	// Compute boundary conditions and right-hand side:
	std::vector<BoundaryCondition> boundaryConditions;

	SimpleFEM::ComputeBoundaryConditions(mesh, boundaryConditions);

	// Apply right-hand side:
	std::vector<double> rhs(nNodes);
	SimpleFEM::ComputeRHS(mesh, rhs);
	mesh.SetRHS(rhs);

	// Solve the problem, this calls a preconditioned CG solver
	// for the sparse matrix with right hand side rhs
	// all nodes stored in "boundaryConditions" are fixed to certain values
	std::vector<double> solution;
	bool isSolved = mesh.Solve(solution, boundaryConditions);
	assert(isSolved);

	// Debug output: Print matrix for non-boundary nodes
	if (debugOut)
	{
		for (int i = 0; i < mesh.GetNumNodes(); i++)
		{
			const Vector2 & pi = mesh.GetNodePosition(i);
			if (SimpleFEM::isOnBoundary(pi))
				continue;
			for (int j = 0; j < mesh.GetNumNodes(); j++)
			{
				const Vector2 & pj = mesh.GetNodePosition(j);
				if (SimpleFEM::isOnBoundary(pj))
					continue;
				if (j > i)
					std::cout << mesh.getMat()(j, i) << "\t";
				else
					std::cout << mesh.getMat()(i, j) << "\t";
			}
			std::cout << std::endl;

			//std::cout << " rhs=" << rhs[i] << "\n";
		}
	}

	double err_nrm = 0;
	std::vector<double> verr(nNodes);
	SimpleFEM::computeError(mesh, solution, verr, err_nrm);
	printf("Error norm is %f\n", err_nrm);
	// Visualize the solution:
	// draw the triangles with colors according to solution
	// blue means zero, red means maxValue.
	// the default problem goes from 0-5 , for other problems, 
	// adjust the maxValue parameter below (values <0, or >maxValue
	// are clamped for the display)
	MeshViewer viewer(argc, argv);

	viewer.InitializeVisualization();
	viewer.CreateSolutionVisualization(mesh, solution);
	viewer.CreateErrorVisualization(mesh, verr);
	viewer.RunVisualization();

	return 0;
}
