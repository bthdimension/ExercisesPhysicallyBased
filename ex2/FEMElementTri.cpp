//=============================================================================
//  Physically-based Simulation in Computer Graphics
//  ETH Zurich
//
//  Author: Peter Kaufmann, Christian Schumacher
//=============================================================================

#include "SimpleFEMDefs.h"
#include "FEMElementTri.h"
#include "FEMMesh.h"

double FEMElementTri::evalSingleBasisGlobalLES(int nodeId, const FEMMesh *pMesh, double x, double y) const
{
    
    Vector2 node_p0 = pMesh->GetNodePosition( m_nodes[0] );
    Vector2 node_p1 = pMesh->GetNodePosition( m_nodes[1] );
    Vector2 node_p2 = pMesh->GetNodePosition( m_nodes[2] );
    
    /* Barycenter (centroid) of each element*/
    Vector2 point_q(x,y);//= (node_p0 + node_p1 + node_p2) / 3.0;
    
    double N;
    if (m_nodes[0] == nodeId)
    {
        Vector2 midpointp1p2( (node_p1 + node_p2) / 2.0 );
        // length from midpoint to node
        double lengthp0mp1p2 = (node_p0 - midpointp1p2).length();
        // length from midpoint to barycenter q
        double lengthqmp1p2 = (point_q - midpointp1p2).length();
        // Ni is then the ratio
        N = lengthqmp1p2 / lengthp0mp1p2;
    }
    else if (m_nodes[1] == nodeId)
    {
        Vector2 midpointp0p2( (node_p0 + node_p2) / 2.0 );
        double lengthp1mp0p2 = (node_p1 - midpointp0p2).length();
        double lengthqmp0p2 = (point_q - midpointp0p2).length();
        N = lengthqmp0p2 / lengthp1mp0p2;
    }
    else if (m_nodes[2] == nodeId)
    {
        Vector2 midpointp0p1( (node_p0 + node_p1) / 2.0 );
        double lengthp2mp0p1 = (node_p2 - midpointp0p1).length();
        double lengthqmp0p1 = (point_q - midpointp0p1).length();
        N = lengthqmp0p1 / lengthp2mp0p1;
    }
    else
    {
        N = 0.0;
        std::cout << "N(x_q,y_q) could not be evaluated!" << std::endl;
    }
    
    return N;
}

void FEMElementTri::computeElementArea(const FEMMesh *pMesh, double &area) const
{
    // First we want to calculate the area A_e of the mesh element,
    // therefore we need the positions of the nodes
    Vector2 node_p0 = pMesh->GetNodePosition( m_nodes[0] );
    Vector2 node_p1 = pMesh->GetNodePosition( m_nodes[1] );
    Vector2 node_p2 = pMesh->GetNodePosition( m_nodes[2] );
    
    Vector2 vecp0p1 = node_p1 - node_p0;
    Vector2 vecp0p2 = node_p2 - node_p0;
    Vector2 vecp1p2 = node_p2 - node_p1;
    
    if( (vecp0p1 | vecp0p2) == 0.0 )
    {
        area = 0.5 * vecp0p1.length() * vecp0p2.length();
    }
    else if( (vecp0p1 | vecp1p2) == 0.0 )
    {
        area = 0.5 * vecp0p1.length() * vecp1p2.length();
    }
    else if( (vecp0p2 | vecp1p2) == 0.0 )
    {
        area = 0.5 * vecp0p2.length() * vecp1p2.length();
    }
    else{
        std::cout << (vecp0p1 | vecp0p2) << " " << (vecp0p1 | vecp1p2) << " " << (vecp0p2 | vecp1p2) << " No orthogonal nodes" << std::endl;
        area = 0.0;
    }
    
}

// TASK 3
void FEMElementTri::Assemble(FEMMesh *pMesh) const
{
    double area;
    computeElementArea(pMesh, area);
    
    /* Basic function derivatives */
    Vector2 node_p0_a0b0;
    Vector2 node_p1_a1b1;
    Vector2 node_p2_a2b2;
    computeSingleBasisDerivGlobalLES( m_nodes[0], node_p0_a0b0, pMesh);
    computeSingleBasisDerivGlobalLES( m_nodes[1], node_p1_a1b1, pMesh);
    computeSingleBasisDerivGlobalLES( m_nodes[2], node_p2_a2b2, pMesh);
    
    
    /* calculate values for stiffness matrix: area * <aibi,ajbj> */
    // <,> denotes the inner product
    
    // the diagonal elements K_ii
    pMesh->AddToStiffnessMatrix( m_nodes[0], m_nodes[0], area * (node_p0_a0b0 | node_p0_a0b0) );
    pMesh->AddToStiffnessMatrix( m_nodes[1], m_nodes[1], area * (node_p1_a1b1 | node_p1_a1b1) );
    pMesh->AddToStiffnessMatrix( m_nodes[2], m_nodes[2], area * (node_p2_a2b2 | node_p2_a2b2) );
    
    //std::cout << m_nodes[0] << " " << m_nodes[1] << " " << std::max(m_nodes[0],m_nodes[1]) << std::endl;
    
    // the mixed elements K_ij
    pMesh->AddToStiffnessMatrix( std::max(m_nodes[0], m_nodes[1]), std::min(m_nodes[0], m_nodes[1]), area * (node_p0_a0b0 | node_p1_a1b1) );
    pMesh->AddToStiffnessMatrix( std::max(m_nodes[0], m_nodes[2]), std::min(m_nodes[0], m_nodes[2]), area * (node_p0_a0b0 | node_p2_a2b2) );
    pMesh->AddToStiffnessMatrix( std::max(m_nodes[2], m_nodes[1]), std::min(m_nodes[2], m_nodes[1]), area * (node_p2_a2b2 | node_p1_a1b1) );
    
}

// TASK 2
void FEMElementTri::computeSingleBasisDerivGlobalGeom(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const
{
    int index = 0;
    
    if(m_nodes[0] == nodeId)
    {
        index = 0;
    }
    else if(m_nodes[1] == nodeId)
    {
        index = 1;
    }
    else if(m_nodes[2] == nodeId)
    {
        index = 2;
    }
    else
    {
        std::cout << "nodeId does not match with available nodes " << std::endl;
    }
    
    // position of current node
    Vector2 node_pc = pMesh->GetNodePosition(nodeId);
    
    // the other two nodes of the triangle
    Vector2 node_p1 = pMesh->GetNodePosition( m_nodes[(index + 1) % 3] );
    Vector2 node_p2 = pMesh->GetNodePosition( m_nodes[(index + 2) % 3] );
    
    Vector2 edgeBetweenP1P2 = node_p2 - node_p1;
    
    Vector2 normalToEdgeP1P2( -edgeBetweenP1P2.y(), edgeBetweenP1P2.x() );
    
    // calculate the height of triangle correspondig to the current node
    double edgeLengthP1Cu = (node_pc - node_p1).length();
    double edgeLengthP2Cu = (node_pc - node_p2).length();
    double heightOfTriangle;
    
    if (edgeBetweenP1P2.length() > edgeLengthP1Cu && edgeBetweenP1P2.length() > edgeLengthP2Cu)
    {
        heightOfTriangle = edgeLengthP1Cu * edgeLengthP2Cu / edgeBetweenP1P2.length();
    }
    else if (edgeLengthP2Cu > edgeLengthP1Cu)
    {
        heightOfTriangle = edgeLengthP1Cu;
    }
    else
    {
        heightOfTriangle = edgeLengthP2Cu;
    }
    
    basisDerivGlobal = normalToEdgeP1P2.normalized() / heightOfTriangle;
    
	
}

// TASK 1
void FEMElementTri::computeSingleBasisDerivGlobalLES(int nodeId, Vector2 &basisDerivGlobal, const FEMMesh *pMesh) const
{
    
    Vector2 node_p0 = pMesh->GetNodePosition( m_nodes[0] );
    Vector2 node_p1 = pMesh->GetNodePosition( m_nodes[1] );
    Vector2 node_p2 = pMesh->GetNodePosition( m_nodes[2] );
    
    
    Vector3 kron_delta(0.0, 0.0, 0.0);
    
    if(m_nodes[0] == nodeId)
    {
        kron_delta.x() = 1.0;
    }
    
    if(m_nodes[1] == nodeId)
    {
        kron_delta.y() = 1.0;
    }
    
    if(m_nodes[2] == nodeId)
    {
        kron_delta.z() = 1.0;
    }
    
    
    
    // create Matrix of positions, s.t. M = [x_0 y_0 1; x_1 y_1 1; x_2 y_2 1]
    Matrix3x3 M;
    
    M(0,0) = node_p0.x();
    M(1,0) = node_p1.x();
    M(2,0) = node_p2.x();
    
    M(0,1) = node_p0.y();
    M(1,1) = node_p1.y();
    M(2,1) = node_p2.y();
    
    M(0,2) = 1.0;
    M(1,2) = 1.0;
    M(2,2) = 1.0;
    
    // aibici = M^-1 * delta
    Vector3 aibici = M.inverse() * kron_delta;
    
    //std::cout << aibici.x() << " " << aibici.y() << std::endl;
    
    basisDerivGlobal = Vector2(aibici.x(), aibici.y());
    
}
