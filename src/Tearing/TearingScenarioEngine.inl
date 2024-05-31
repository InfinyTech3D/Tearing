#pragma once
#include <Tearing/TearingScenarioEngine.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

#include <sofa/component/mechanicalload/ConstantForceField.h>
#include <sofa/component/constraint/projective/FixedProjectiveConstraint.h>

namespace sofa::component::engine
{

using sofa::type::Vec3;

template <class DataTypes>
TearingScenarioEngine<DataTypes>::TearingScenarioEngine()
        : d_showFracturePath(initData(&d_showFracturePath, true, "showFracturePath", "Flag activating rendering of fracture path"))
        , d_input_positions(initData(&d_input_positions, "input_position", "Input position"))
        , d_startVertexId(initData(&d_startVertexId, int(-1), "startVertexId", "Vertex ID to start a given tearing scenario, -1 if none"))
        , d_startTriId(initData(&d_startTriId, int(-1), "startTriangleId", "Triangle ID from which the starting point is chosen", -1 if none))
        , d_startDirection(initData(&d_startDirection, Vec3(1.0, 0.0, 0.0), "startDirection", "If startVertexId is set, define the direction of the tearing scenario. x direction by default"))
        , d_startLength(initData(&d_startLength, Real(1.0), "startLength", "If startVertexId is set, define the length of the tearing, to be combined with startDirection"))
    {

        addInput(&d_input_positions);
        addInput(&d_startVertexId);
        addInput(&d_startTriId);
        addInput(&d_startDirection);
        addInput(&d_startLength);
    }

    template <class DataTypes>
    void TearingScenarioEngine<DataTypes>::init()
    {
        BaseTearingEngine<DataTypes>::init();
    }

    template <class DataTypes>
    void TearingScenarioEngine<DataTypes>::reinit()
    {
        BaseTearingEngine<DataTypes>::update();
    }

    // --------------------------------------------------------------------------------------
    // --- Computation methods
    // --------------------------------------------------------------------------------------
    //template<class DataTypes>
    //inline void TearingScenarioEngine<DataTypes>::computePerpendicular(Coord dir, Coord& normal)
    //{
    //    int  triID = d_startTriId.getValue();
    //    int indexA = d_startVertexId.getValue();

    //    if (triID == -1) 
    //        return;
    //    

    //    const Triangle& VertexIndicies = m_topology->getTriangle(triID);
    //    constexpr size_t numVertices = 3;

    //    Index B_id = -1, C_id = -1;

    //    for (unsigned int vertex_id = 0; vertex_id < numVertices; vertex_id++)
    //    {
    //        if (VertexIndicies[vertex_id] == indexA)
    //        {
    //            B_id = VertexIndicies[(vertex_id + 1) % 3];
    //            C_id = VertexIndicies[(vertex_id + 2) % 3];
    //            break;
    //        }
    //    }

    //    helper::ReadAccessor< Data<VecCoord> > x(d_input_positions.getValue());
    //    Coord A = x[triID];
    //    Coord B = x[B_id];
    //    Coord C = x[C_id];

    //    Coord AB = B - A;
    //    Coord AC = C - A;

    //    Coord triangleNormal = sofa::type::cross(AB, AC);
    //    normal = sofa::type::cross(triangleNormal, dir);
    //}



    //template <class DataTypes>
    //void TearingScenarioEngine<DataTypes>::performFractureScenario()
    //{
    //   
    //    // perform scenario only once
    //    //d_nbFractureMax.setValue(1);

    //    int indexA = d_startVertexId.getValue();
    //    int triID = d_startTriId.getValue();
    //    const Vec3& dir = d_startDirection.getValue();
    //    const Real& alpha = d_startLength.getValue();

    //    helper::ReadAccessor< Data<VecCoord> > x(d_input_positions.getValue());
    //    Coord Pa = x[indexA];
    //    //Coord principalStressDirection = m_triangleInfoTearing[m_maxStressTriangleIndex].principalStressDirection;
    //    Coord normal;
    //    computePerpendicular(dir,normal);


    //    Coord Pb = Pa + alpha * dir;
    //    Coord Pc = Pa - alpha * dir;

    //    m_tearingAlgo->algoFracturePath(Pa, indexA, Pb, Pc, triID, normal, d_input_positions.getValue());
    //   
    //}
    //template <class DataTypes>
    //void TearingScenarioEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
    //{
    //    int triID = d_startTriId.getValue();

    //    std::vector<Vec3> Tri;
    //    sofa::type::RGBAColor color2(0.0f, 1.0f, 0.0f, 1.0f);
    //   
    //    const VecTriangles& triangleList = m_topology->getTriangles();
    //    
    //    const Triangle& tri = triangleList[triID]; // Is this correct?
    //    
    //    Coord Pa = x[tri[0]];
    //    Coord Pb = x[tri[1]];
    //    Coord Pc = x[tri[2]];

    //    Tri.push_back(Pa);
    //    Tri.push_back(Pb);
    //    Tri.push_back(Pc);

    //    vparams->drawTool()->drawTriangles(maxStressTri, color2);

    //    if (d_showFracturePath.getValue())
    //    {

    //    }

    //}
}
