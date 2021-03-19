#pragma once
#include "TearingEngine.h"

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/types/RGBAColor.h>

#include <SofaBaseTopology/TopologyData.inl>

namespace sofa::component::engine
{


template <class DataTypes>
TearingEngine<DataTypes>::TearingEngine()
    : input_position(initData(&input_position, "input_position", "Input position"))
    , triangleInfo(initData(&triangleInfo, "triangleInfo", "Internal triangle data"))
    , vertexInfo(initData(&vertexInfo, "vertexInfo", "Internal point data"))
    , d_triangles(initData(&d_triangles, "triangles", "Triangle Topology"))
    , d_area(initData(&d_area, "area","list of area"))
	, l_topology(initLink("topology", "link to the topology container"))
	, m_topology(nullptr)

    , showChangedTriangle( initData(&showChangedTriangle,true,"showChangedTriangle", "Flag activating rendering of changed triangle"))
{

    addInput(&d_triangles);
    addInput(&input_position);
}

template <class DataTypes>
void TearingEngine<DataTypes>::init()
{

    if (l_topology.empty())
    {
        msg_info() << "link to Topology container should be set to ensure right behavior. First Topology found in current context will be used.";
        l_topology.set(this->getContext()->getMeshTopologyLink());
    }

    m_topology = l_topology.get();
    msg_info() << "Topology path used: '" << l_topology.getLinkedPath() << "'";

    if (m_topology == nullptr)
    {
        msg_error() << "No topology component found at path: " << l_topology.getLinkedPath() << ", nor in current context: " << this->getContext()->name;
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }


    //A BOUGER
    //créer un vecteur stockant les triangles et un autre leurs aire
    VecElement triangleList;
    helper::vector<double> initAreaList;
    triangleList = m_topology->getTriangles();
    //helper::vector<TriangleInformation>& triangleInf = *(triangleInfo.beginEdit());
    //triangleInf.resize(triangleList.size());
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    helper::WriteAccessor< Data<vector<double>> > area(d_area);
    area.resize(triangleList.size());
    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        Element triangle = triangleList[i];
        Index a = triangle[0];
        Index b = triangle[1];
        Index c = triangle[2];

       

        //double a_x = m_topology->getPX(a);
        Coord Pa = x[a];
        Coord Pb = x[b];
        Coord Pc = x[c];

        Real determinant;
        area[i] = Pa[0];
        //sofa::helper::types::RGBAColor color(1.0f, 0.76078431372f, 0.0f, 1.0f); //dans draw
        /*
        Coord ab_cross_ac = cross(b - a, c - a);
        determinant = ab_cross_ac.norm();
        */


        //triangleInf[i].area = 0.0;//determinant*0.5f;
    }
    //triangleInfo.endEdit();
}

template <class DataTypes>
void TearingEngine<DataTypes>::reinit()
{
	update();
}

template <class DataTypes>
void TearingEngine<DataTypes>::doUpdate()
{}


template <class DataTypes>
void TearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{/*
    if (showChangedTriangle.getValue())
    {
        helper::vector<VertexInformation>& vertexInf = *(vertexInfo.beginEdit());
        for (unsigned int i = 0; i < vertexInf.size(); i++)
        {
            const core::topology::BaseMeshTopology::TrianglesAroundVertex& triangles = m_topology->getTrianglesAroundVertex(i);
            double ACHANGER = 0.0;
            for (unsigned int v = 0; v < triangles.size(); v++)
            {
                ACHANGER += triangleInfo.getValue()[triangles[v]].area;
            }
        }
    }
*/
}


} //namespace sofa::component::engine