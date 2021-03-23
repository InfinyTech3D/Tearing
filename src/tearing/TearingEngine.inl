#pragma once
#include "TearingEngine.h"

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/types/RGBAColor.h>
#include <sofa/helper/ColorMap.h>
#include <SofaBaseTopology/TopologyData.inl>


namespace sofa::component::engine
{
template <class DataTypes>
TearingEngine<DataTypes>::TearingEngine()
    : input_position(initData(&input_position, "input_position", "Input position"))
    , d_area(initData(&d_area, "area","list of area"))
    , d_initArea(initData(&d_initArea, "initArea", "list of initial area"))
    , d_seuil(initData(&d_seuil, -1.0, "seuil", "threshold value for area"))
    
    , d_triangleList_TEST(initData(&d_triangleList_TEST, "triangleList_TEST", "valeur TEST a supprimer"))

	, l_topology(initLink("topology", "link to the topology container"))
	, m_topology(nullptr)
    , showChangedTriangle( initData(&showChangedTriangle,true,"showChangedTriangle", "Flag activating rendering of changed triangle"))
{
    addInput(&input_position);
    addInput(&d_seuil);
    addOutput(&d_area);
    addOutput(&d_triangleList_TEST);
    p_drawColorMap = new helper::ColorMap(256, "Blue to Red");
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

    initComputeArea();
    computeArea();
    
    
    VecElement TESTouille;
    triangleOverThreshold(TESTouille);

}

template <class DataTypes>
void TearingEngine<DataTypes>::reinit()
{
	update();
}

template <class DataTypes>
void TearingEngine<DataTypes>::doUpdate()
{
    computeArea();

    VecElement TESTouille;
    triangleOverThreshold(TESTouille);
}


template <class DataTypes>
void TearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    VecElement triangleList;
    triangleList = m_topology->getTriangles();
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    helper::ReadAccessor< Data<vector<double>> > area(d_area);
    helper::ReadAccessor< Data<vector<double>> > initArea(d_initArea);

    double minDeltaArea = std::numeric_limits<double>::max();
    double maxDeltaArea = 0.0;
    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        Real deltaArea = abs(initArea[i] - area[i]);
        if (deltaArea < minDeltaArea)
            minDeltaArea = deltaArea;
        if (deltaArea > maxDeltaArea)
            maxDeltaArea = deltaArea;
    }

    std::vector<sofa::defaulttype::Vector3> vertices;
    std::vector<sofa::helper::types::RGBAColor> colorVector;
    helper::ColorMap::evaluator<double> evalColor = p_drawColorMap->getEvaluator(0.0, maxDeltaArea);

    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        Element triangle = triangleList[i];
        Index a = triangle[0];
        Index b = triangle[1];
        Index c = triangle[2];

        Coord Pa = x[a];
        Coord Pb = x[b];
        Coord Pc = x[c];

        colorVector.push_back(evalColor(abs(initArea[i] - area[i])));
        vertices.push_back(Pa);
        colorVector.push_back(evalColor(abs(initArea[i] - area[i])));
        vertices.push_back(Pb);
        colorVector.push_back(evalColor(abs(initArea[i] - area[i])));
        vertices.push_back(Pc);
    }
    vparams->drawTool()->drawTriangles(vertices, colorVector);
    vertices.clear();
    colorVector.clear();
}

template <class DataTypes>
void TearingEngine<DataTypes>::computeArea()
{
    VecElement triangleList;
    triangleList = m_topology->getTriangles();
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    helper::WriteAccessor< Data<vector<double>> > area(d_area);
    if(area.size() != triangleList.size() )
    {
        area.resize(triangleList.size());
    }
    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        Element triangle = triangleList[i];
        Index a = triangle[0];
        Index b = triangle[1];
        Index c = triangle[2];

        Coord Pa = x[a];
        Coord Pb = x[b];
        Coord Pc = x[c];

        Real determinant;
        Coord ab_cross_ac = cross(Pb - Pa, Pc - Pa);
        determinant = ab_cross_ac.norm();
        area[i] = determinant * 0.5f;
    }
}

template <class DataTypes>
void TearingEngine<DataTypes>::initComputeArea()
{
    VecElement triangleList;
    triangleList = m_topology->getTriangles();
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    helper::WriteAccessor< Data<vector<double>> > area(d_initArea);
    if (area.size() != triangleList.size())
    {
        area.resize(triangleList.size());
    }
    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        Element triangle = triangleList[i];
        Index a = triangle[0];
        Index b = triangle[1];
        Index c = triangle[2];

        Coord Pa = x[a];
        Coord Pb = x[b];
        Coord Pc = x[c];

        Real determinant;

        Coord ab_cross_ac = cross(Pb - Pa, Pc - Pa);
        determinant = ab_cross_ac.norm();
        area[i] = determinant * 0.5f;
    }
}

template <class DataTypes>
void TearingEngine<DataTypes>::triangleOverThreshold(VecElement& triangleOverThresholdList)
{
    VecElement triangleList;
    triangleList = m_topology->getTriangles();
    helper::ReadAccessor< Data<vector<double>> > area(d_area);
    helper::ReadAccessor< Data<double> > threshold(d_seuil);
    helper::WriteAccessor< Data<VecElement> > TEST(d_triangleList_TEST);
    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        if (area[i] >= threshold)
        {
            triangleOverThresholdList.push_back(triangleList[i]);
            TEST.push_back(triangleList[i]);
        }
            
            
    }
}





} //namespace sofa::component::engine