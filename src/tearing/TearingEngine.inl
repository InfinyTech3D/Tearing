#pragma once
#include "TearingEngine.h"

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/types/RGBAColor.h>
#include <sofa/helper/ColorMap.h>
#include <SofaBaseTopology/TopologyData.inl>

//#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>


namespace sofa::component::engine
{
template <class DataTypes>
TearingEngine<DataTypes>::TearingEngine()
    : input_position(initData(&input_position, "input_position", "Input position"))
    , d_initArea(initData(&d_initArea, "initArea", "list of initial area"))
    , d_seuilArea(initData(&d_seuilArea, 0.1, "seuilArea", "threshold value for area"))
    , d_seuilPrincipalStress(initData(&d_seuilPrincipalStress, 55.0, "seuilStress", "threshold value for stress"))
    , d_triangleOverThresholdList(initData(&d_triangleOverThresholdList, "triangleOverThresholdList", "triangles with maxStress over threshold value"))
	, l_topology(initLink("topology", "link to the topology container"))
	, m_topology(nullptr)
    , m_triangleGeo(nullptr)
    , m_triangularFEM(nullptr)
    , showChangedTriangle(initData(&showChangedTriangle, true,"showChangedTriangle", "Flag activating rendering of changed triangle"))
    , showTearableTriangle(initData(&showTearableTriangle, false, "showTearableTriangle", "Flag activating rendering of fracturable triangle"))
    , d_triangleInfoTearing(initData(&d_triangleInfoTearing, "triangleInfoTearing", "Internal triangle data"))
    , d_triangleFEMInfo(initData(&d_triangleFEMInfo, "triangleFEMInfo", "Internal triangle data"))
    , d_maxStress(initData(&d_maxStress, "maxStress", "maxStress"))
    , d_indexTriangleMaxStress(initData(&d_indexTriangleMaxStress, "indexTriangleMaxStress", "index of triangle where the principal stress is maximum"))
    , d_indexVertexMaxStress(initData(&d_indexVertexMaxStress, "indexVertexMaxStress", "index of vertex where the stress is maximum"))

{
    addInput(&input_position);
    addInput(&d_seuilArea);
    addInput(&d_seuilPrincipalStress);
    addOutput(&d_triangleInfoTearing);
    addOutput(&d_triangleFEMInfo);
    addOutput(&d_triangleOverThresholdList);
    addOutput(&d_maxStress);
    addOutput(&d_indexVertexMaxStress);
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

    m_topology->getContext()->get(m_triangleGeo);
    if (!m_triangleGeo)
    {
        msg_error() << "Missing component: Unable to get TriangleSetGeometryAlgorithms from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    m_topology->getContext()->get(m_triangularFEM);
    if (!m_triangularFEM)
    {
        msg_error() << "Missing component: Unable to get TriangleSetGeometryAlgorithms from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    initComputeArea();
    computeArea();
    updateTriangleInformation();
    //triangleOverThresholdArea();
    triangleOverThresholdPrincipalStress();


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
    updateTriangleInformation();
    //triangleOverThresholdArea(); 
    triangleOverThresholdPrincipalStress();
}


template <class DataTypes>
void TearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (showChangedTriangle.getValue())
    {
        VecElement triangleList;
        triangleList = m_topology->getTriangles();
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        helper::WriteAccessor< Data<vector<TriangleInformation>> > triangleInf(d_triangleInfoTearing); //ne fonctionne pas en ReadAccessor
        helper::ReadAccessor< Data<vector<double>> > initArea(d_initArea);

        double minDeltaArea = std::numeric_limits<double>::max();
        double maxDeltaArea = 0.0;

        for (unsigned int i = 0; i < triangleList.size(); i++)
        {
            TriangleInformation* tinfo = &triangleInf[i];
            Real deltaArea = abs(initArea[i] - tinfo->area);

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
            TriangleInformation* tinfo = &triangleInf[i];

            Index a = triangle[0];
            Index b = triangle[1];
            Index c = triangle[2];

            Coord Pa = x[a];
            Coord Pb = x[b];
            Coord Pc = x[c];

            colorVector.push_back(evalColor(abs(initArea[i] - tinfo->area)));
            vertices.push_back(Pa);
            colorVector.push_back(evalColor(abs(initArea[i] - tinfo->area)));
            vertices.push_back(Pb);
            colorVector.push_back(evalColor(abs(initArea[i] - tinfo->area)));
            vertices.push_back(Pc);
        }
        vparams->drawTool()->drawTriangles(vertices, colorVector);
        vertices.clear();
        colorVector.clear();
    }
    
    if (showTearableTriangle.getValue())
    {
        VecElement triangleList = m_topology->getTriangles();
        helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleOverThresholdList);
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        std::vector<sofa::defaulttype::Vector3> vertices;
        sofa::helper::types::RGBAColor color(0.0f, 0.0f, 1.0f, 1.0f);
        if (candidate.size())
        {
            for (unsigned int i = 0; i < candidate.size(); i++)
            {
                Coord Pa = x[triangleList[candidate[i]][0]];
                Coord Pb = x[triangleList[candidate[i]][1]];
                Coord Pc = x[triangleList[candidate[i]][2]];
                vertices.push_back(Pa);
                vertices.push_back(Pb);
                vertices.push_back(Pc);
            }
            vparams->drawTool()->drawTriangles(vertices, color);

            std::vector<sofa::defaulttype::Vector3> vecteur;
            Coord principalStressDirection = d_triangleFEMInfo.getValue()[d_indexTriangleMaxStress.getValue()].principalStressDirection;
            Coord Pa = x[d_indexVertexMaxStress.getValue()];
            vecteur.push_back(Pa);
            vecteur.push_back(Pa + principalStressDirection);
            vparams->drawTool()->drawLines(vecteur, 1, sofa::helper::types::RGBAColor(0, 1, 0, 1));
        }
    }
}



// --------------------------------------------------------------------------------------
// --- Computation methods
// --------------------------------------------------------------------------------------
template <class DataTypes>
void TearingEngine<DataTypes>::initComputeArea()
{
    VecElement triangleList;
    triangleList = m_topology->getTriangles();
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    helper::WriteAccessor< Data<vector<double>> > initArea(d_initArea);
    helper::WriteAccessor< Data<vector<TriangleInformation>> > triangleInf(d_triangleInfoTearing);
    if (initArea.size() != triangleList.size())
        initArea.resize(triangleList.size());
    if (triangleInf.size() != triangleList.size())
        triangleInf.resize(triangleList.size());

    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        Element triangle = triangleList[i];
        TriangleInformation* tinfo = &triangleInf[i];

        Index a = triangle[0];
        Index b = triangle[1];
        Index c = triangle[2];

        Coord Pa = x[a];
        Coord Pb = x[b];
        Coord Pc = x[c];

        Real determinant;

        Coord ab_cross_ac = cross(Pb - Pa, Pc - Pa);
        determinant = ab_cross_ac.norm();
        tinfo->area = determinant * 0.5f;
        initArea[i] = tinfo->area;
    }
}

template <class DataTypes>
void TearingEngine<DataTypes>::computeArea()
{
    VecElement triangleList;
    triangleList = m_topology->getTriangles();
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    helper::WriteAccessor< Data<vector<TriangleInformation>> > triangleInf(d_triangleInfoTearing);
    if(triangleInf.size() != triangleList.size() )
    {
        triangleInf.resize(triangleList.size());
    }
    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        Element triangle = triangleList[i];
        TriangleInformation* tinfo = &triangleInf[i];
        Index a = triangle[0];
        Index b = triangle[1];
        Index c = triangle[2];

        Coord Pa = x[a];
        Coord Pb = x[b];
        Coord Pc = x[c];

        Real determinant;
        Coord ab_cross_ac = cross(Pb - Pa, Pc - Pa);
        determinant = ab_cross_ac.norm();
        tinfo->area = determinant * 0.5f;
    }
}


template <class DataTypes>
void TearingEngine<DataTypes>::triangleOverThresholdArea()
{
    VecElement triangleList;
    triangleList = m_topology->getTriangles();
    helper::ReadAccessor< Data<double> > threshold(d_seuilArea);
    helper::WriteAccessor< Data<vector<Index>> > TEST(d_triangleOverThresholdList);
    helper::WriteAccessor< Data<vector<TriangleInformation>> > triangleInf(d_triangleInfoTearing);
    TEST.clear();
    //ne pas oublier à la fin d'un step de clear cette liste sinon on accumule les triangles
    Index max = 0;
    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        TriangleInformation* tinfo = &triangleInf[i];
        if (tinfo->area >= threshold)
        {
            TEST.push_back(i);
            if (tinfo->area > triangleInf[max].area)
                max = i;
        }         
    }
}

template <class DataTypes>
void TearingEngine<DataTypes>::triangleOverThresholdPrincipalStress()
{
    VecElement triangleList;
    triangleList = m_topology->getTriangles();
    helper::ReadAccessor< Data<double> > threshold(d_seuilPrincipalStress);
    helper::WriteAccessor< Data<vector<Index>> > candidate(d_triangleOverThresholdList);
    helper::WriteAccessor< Data<vector<TriangleInformation>> > triangleInf(d_triangleInfoTearing);
    Real& maxStress = *(d_maxStress.beginEdit());
    Index& indexTriangleMaxStress = *(d_indexTriangleMaxStress.beginEdit());
    candidate.clear();
    maxStress = 0;
    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        TriangleInformation* tinfo = &triangleInf[i];
        if (tinfo->maxStress >= threshold)
        {
            candidate.push_back(i);
            if (tinfo->maxStress > maxStress)
            {
                indexTriangleMaxStress = i;
                maxStress = tinfo->maxStress;
            }
        }
    }
    if (candidate.size())
    {
        Index& indexVertexMaxStress = *(d_indexVertexMaxStress.beginEdit());
        TriangleInformation* tinfo = &triangleInf[indexTriangleMaxStress];
        Index k = (tinfo->stress[0] > tinfo->stress[1]) ? 0 : 1;
        k = (tinfo->stress[k] > tinfo->stress[2]) ? k : 2;
        indexVertexMaxStress = triangleList[indexTriangleMaxStress][k];      
        d_indexVertexMaxStress.endEdit();
    }
    d_maxStress.endEdit();
    d_indexTriangleMaxStress.endEdit();
}

template <class DataTypes>
void TearingEngine<DataTypes>::updateTriangleInformation()
{
    //update triangleInformation
    VecElement triangleList;
    triangleList = m_topology->getTriangles();
    d_triangleFEMInfo = m_triangularFEM->triangleInfo.getValue();
    helper::WriteAccessor< Data<vector<TriangleInformation>> > triangleInf(d_triangleInfoTearing);
    helper::WriteAccessor< Data<VecTriangleFEMInformation> > triangleFEMInf(d_triangleFEMInfo);

    if (triangleInf.size() != triangleList.size())
    {
        triangleInf.resize(triangleList.size());
    }
    if (triangleFEMInf.size() != triangleList.size())
    {
        triangleFEMInf.resize(triangleList.size());
    }

    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        TriangleFEMInformation tFEMinfo = triangleFEMInf[i];
        TriangleInformation* tinfo = &triangleInf[i];

        //tinfo->area =  tFEMinfo.area;;
        tinfo->stress = tFEMinfo.stress;
        tinfo->maxStress = tFEMinfo.maxStress;
    }
}

} //namespace sofa::component::engine




