#pragma once
#include "TearingEngine.h"

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/types/RGBAColor.h>
#include <sofa/helper/ColorMap.h>
#include <SofaBaseTopology/TopologyData.inl>
#include <sofa/simulation/Simulation.h>


namespace sofa::component::engine
{
template <class DataTypes>
TearingEngine<DataTypes>::TearingEngine()
    : input_position(initData(&input_position, "input_position", "Input position"))
    , d_seuilPrincipalStress(initData(&d_seuilPrincipalStress, 55.0, "seuilStress", "threshold value for stress"))
    , d_triangleOverThresholdList(initData(&d_triangleOverThresholdList, "triangleOverThresholdList", "triangles with maxStress over threshold value"))
    , d_triangleToIgnoreList(initData(&d_triangleToIgnoreList, "triangleToIgnoreList", "triangles that can't be choosen as starting fracture point"))
	, l_topology(initLink("topology", "link to the topology container"))
	, m_topology(nullptr)
    , m_triangleGeo(nullptr)
    , m_triangularFEM(nullptr)
    , m_modifier(nullptr)
    , showChangedTriangle(initData(&showChangedTriangle, false,"showChangedTriangle", "Flag activating rendering of changed triangle"))
    , showTearableTriangle(initData(&showTearableTriangle, true, "showTearableTriangle", "Flag activating rendering of fracturable triangle"))
    , d_triangleInfoTearing(initData(&d_triangleInfoTearing, "triangleInfoTearing", "Internal triangle data"))
    , d_triangleFEMInfo(initData(&d_triangleFEMInfo, "triangleFEMInfo", "Internal triangle data"))
    , d_maxStress(initData(&d_maxStress, "maxStress", "maxStress"))
    , d_indexTriangleMaxStress(initData(&d_indexTriangleMaxStress, "indexTriangleMaxStress", "index of triangle where the principal stress is maximum"))
    , d_indexVertexMaxStress(initData(&d_indexVertexMaxStress, "indexVertexMaxStress", "index of vertex where the stress is maximum"))
    , stepByStep(initData(&stepByStep, true, "stepByStep", "Flag activating step by step option for tearing"))
    , d_step(initData(&d_step, 20, "step", "step size"))
    , d_counter(initData(&d_counter, 0, "counter", "counter for the step by step option"))

    , showFracturePath(initData(&showFracturePath, true, "showFracturePath", "Flag activating rendering of fracture path"))
    , d_fractureMaxLength(initData(&d_fractureMaxLength, 1.0, "fractureMaxLength", "fracture max length by time step"))
    , d_nbFractureMax(initData(&d_nbFractureMax, 15, "nbFractureMax", "number of fracture max done by the algorithm"))
    , d_scenario(initData(&d_scenario, 0, "scenario", "choose scenario, zero is default"))
    , ignoreTriangleAtStart(initData(&ignoreTriangleAtStart, true, "ignoreTriangleAtStart","option to ignore some triangles at start of the tearing algo"))
    , m_tearingAlgo(nullptr)
{
    addInput(&input_position);
    addInput(&d_seuilPrincipalStress);
    addInput(&d_fractureMaxLength);
    addInput(&d_step);
    addInput(&d_scenario);
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
    this->f_listening.setValue(true);
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
        msg_error() << "Missing component: Unable to get TriangularFEMForceField from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    m_topology->getContext()->get(m_modifier);
    if (!m_modifier)
    {
        msg_error() << "Missing component: Unable to get TriangleSetTopologyModifier from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    if (ignoreTriangleAtStart.getValue())
        computeTriangleToSkip();
    updateTriangleInformation();
    triangleOverThresholdPrincipalStress();
    d_counter.setValue(0);
    
    if (m_tearingAlgo == nullptr)
        m_tearingAlgo = new TearingAlgorithms<DataTypes>(m_topology, m_modifier, m_triangleGeo);
}

template <class DataTypes>
void TearingEngine<DataTypes>::reinit()
{
	update();
}

template <class DataTypes>
void TearingEngine<DataTypes>::doUpdate()
{
    d_counter.setValue(d_counter.getValue() + 1);
    std::cout << "counter=" << d_counter.getValue() << std::endl;

    if (ignoreTriangleAtStart.getValue())
    {
        if(m_tearingAlgo!= nullptr && m_tearingAlgo->getFractureNumber() == 0)
            computeTriangleToSkip();
    }
    else
    {
        vector<Index> emptyIndexList;
        d_triangleToIgnoreList.setValue(emptyIndexList);
    }

    if (m_tearingAlgo != nullptr)
    {
        const vector< vector<int> >& TjunctionTriangle = m_tearingAlgo->getTjunctionTriangles();
        helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_triangleToIgnoreList);
        for (unsigned int i = 0; i < TjunctionTriangle.size(); i++)
        {
            if (TjunctionTriangle[i][0] == m_tearingAlgo->getFractureNumber())
            {
                if (std::find(triangleToSkip.begin(), triangleToSkip.end(), TjunctionTriangle[i][1]) == triangleToSkip.end())
                    triangleToSkip.push_back(TjunctionTriangle[i][1]);
            }
        }
    }

    updateTriangleInformation();
    triangleOverThresholdPrincipalStress();
}


template <class DataTypes>
void TearingEngine<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
 

    if (showTearableTriangle.getValue())
    {
        VecElement triangleList = m_topology->getTriangles();
        helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleOverThresholdList);
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        std::vector<sofa::defaulttype::Vector3> vertices;
        sofa::helper::types::RGBAColor color(0.0f, 0.0f, 1.0f, 1.0f);
        std::vector<sofa::defaulttype::Vector3> tearTriangleVertices;
        sofa::helper::types::RGBAColor color2(0.0f, 1.0f, 0.0f, 1.0f);
        if (candidate.size() > 0)
        {
            for (unsigned int i = 0; i < candidate.size(); i++)
            {
                if(candidate[i]!= d_indexTriangleMaxStress.getValue())
                {
                Coord Pa = x[triangleList[candidate[i]][0]];
                Coord Pb = x[triangleList[candidate[i]][1]];
                Coord Pc = x[triangleList[candidate[i]][2]];
                vertices.push_back(Pa);
                vertices.push_back(Pb);
                vertices.push_back(Pc);
                }
                else
                {
                    Coord Pa = x[triangleList[candidate[i]][0]];
                    Coord Pb = x[triangleList[candidate[i]][1]];
                    Coord Pc = x[triangleList[candidate[i]][2]];
                    tearTriangleVertices.push_back(Pa);
                    tearTriangleVertices.push_back(Pb);
                    tearTriangleVertices.push_back(Pc);
                }
            }
            vparams->drawTool()->drawTriangles(vertices, color);
            vparams->drawTool()->drawTriangles(tearTriangleVertices, color2);

            std::vector<sofa::defaulttype::Vector3> vecteur;
            Coord principalStressDirection = d_triangleFEMInfo.getValue()[d_indexTriangleMaxStress.getValue()].principalStressDirection;
            Coord Pa = x[d_indexVertexMaxStress.getValue()];

            vecteur.push_back(Pa);
            vecteur.push_back(Pa + principalStressDirection);
            vparams->drawTool()->drawLines(vecteur, 1, sofa::helper::types::RGBAColor(0, 1, 0, 1));
            vecteur.clear();
            Coord fractureDirection;
            fractureDirection[0] = -principalStressDirection[1];
            fractureDirection[1] = principalStressDirection[0];
            vecteur.push_back(Pa);
            vecteur.push_back(Pa + fractureDirection);
            vparams->drawTool()->drawLines(vecteur, 1, sofa::helper::types::RGBAColor(1.0, 0.65, 0.0, 1.0));
            vecteur.clear();
        }
    }

    if (showFracturePath.getValue())
    {
        helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleOverThresholdList);
        if (candidate.size() > 0)
        {
            helper::ReadAccessor< Data<VecCoord> > x(input_position);
            Coord principalStressDirection = d_triangleFEMInfo.getValue()[d_indexTriangleMaxStress.getValue()].principalStressDirection;
            Coord Pa = x[d_indexVertexMaxStress.getValue()];
            Coord fractureDirection;
            fractureDirection[0] = -principalStressDirection[1];
            fractureDirection[1] = principalStressDirection[0];

            vector<Coord> points;
            Real norm_fractureDirection = fractureDirection.norm();
            Coord Pb = Pa + d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
            Coord Pc = Pa - d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
            points.push_back(Pb);
            points.push_back(Pc);
            vparams->drawTool()->drawPoints(points, 10, sofa::helper::types::RGBAColor(1, 0.5, 0.5, 1));
            vparams->drawTool()->drawLines(points, 1, sofa::helper::types::RGBAColor(1, 0.5, 0, 1));
            points.clear();

            const sofa::helper::vector<Coord>& path = m_tearingAlgo->getFracturePath();
            if (!path.empty())
                vparams->drawTool()->drawPoints(path, 10, sofa::helper::types::RGBAColor(0, 1, 0, 1));
        }
    }
}

template <class DataTypes>
void TearingEngine<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (/* simulation::AnimateBeginEvent* ev = */simulation::AnimateBeginEvent::checkEventType(event))
    {
        if ( ((d_counter.getValue() % d_step.getValue()) == 0) && (m_tearingAlgo->getFractureNumber() < d_nbFractureMax.getValue()) || !stepByStep.getValue())
        {
            if(d_counter.getValue()>d_step.getValue())
                algoFracturePath();
        }
    }
}



// --------------------------------------------------------------------------------------
// --- Computation methods
// --------------------------------------------------------------------------------------


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
    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_triangleToIgnoreList);

    for (unsigned int i = 0; i < triangleList.size(); i++)
    {
        if (std::find(triangleToSkip.begin(), triangleToSkip.end(), i) == triangleToSkip.end())
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

        tinfo->stress = tFEMinfo.stress;
        tinfo->maxStress = tFEMinfo.maxStress;
    }
}


template <class DataTypes>
void TearingEngine<DataTypes>::algoFracturePath()
{
    helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleOverThresholdList);
    int choice;

    //start with specific scenario
    if (m_tearingAlgo->getFractureNumber() == 0)
    {
        choice = d_scenario.getValue();
    }
    else
    {
        choice = 0;
    }

    //if no candidate available we don't call intersection process
    if (candidate.size() || choice)
    {
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        double EPS = 1e-8;
        bool PATH_IS_OK = false;

        //Calculate fracture starting point (Pa)
        int indexA;
        Coord Pa;
        Coord principalStressDirection;
        //Calculate fracture end points (Pb and Pc)
        Coord Pb;
        Coord Pc;
        Coord dir;
        double alpha;

        //scenario with specific starting point and specific direction
        switch (choice)
        {
        default:
            indexA = d_indexVertexMaxStress.getValue();
            Pa = x[indexA];
            principalStressDirection = d_triangleFEMInfo.getValue()[d_indexTriangleMaxStress.getValue()].principalStressDirection;
            computeEndPoints(Pa, principalStressDirection, Pb, Pc);
            break;

        case 1:
            //CasTest1-1
            indexA = 421;
            Pa = x[indexA];
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 2.5;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 2:
            //CasTest1-2
            indexA = 1;
            Pa = x[indexA];
            dir[0] = 0.707; dir[1] = 0.707; dir[2] = 0.0;
            alpha = 3.5;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 3:
            //CasTest2-1
            indexA = 470;
            Pa = x[indexA];
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 4.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 4:
            //CasTest2-2
            indexA = 84;
            Pa = x[indexA];
            dir[0] = 0.707; dir[1] = 0.707; dir[2] = 0.0;
            alpha = 3.5;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 5:
            //CasTest4-1
            indexA = 45;
            Pa = x[indexA];
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 2.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 6:
            //CasTest4-1
            indexA = 51;
            Pa = x[indexA];
            dir[0] = 0.0; dir[1] = 1.0; dir[2] = 0.0;
            alpha = 5.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 7:
            //CasTest5_holeCircular-1
            indexA = 318;
            Pa = x[indexA];
            dir[0] = 0.0; dir[1] = 1.0; dir[2] = 0.0;
            alpha = 2.5;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 8:
            //CasTest5_holeCircular-2
            indexA = 282;
            Pa = x[indexA];
            dir[0] = 0.707; dir[1] = -0.707; dir[2] = 0.0;
            alpha = 2.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 9:
            //CasTest5_holeSquare
            indexA = 231;
            Pa = x[indexA];
            dir[0] = 0.0; dir[1] = 1.0; dir[2] = 0.0;
            alpha = 5.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - dir;
            break;

        case 10:
            //CasTest5_holeSquareVertical
            indexA = 229;
            Pa = x[indexA];
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 2.0;
            Pb = Pa - alpha * dir;
            Pc = Pa + dir;
            break;

        case 11:
            //CasTest5_incision-1
            indexA = 179;
            Pa = x[indexA];
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 2.0;
            Pb = Pa - alpha * dir;
            Pc = Pa + alpha * dir;
            break;

        case 12:
            //CasTest5_incision-2
            indexA = 188;
            Pa = x[indexA];
            dir[0] = 0.0; dir[1] = 1.0; dir[2] = 0.0;
            alpha = 5.0;
            Pb = Pa;
            Pc = Pa - alpha * dir;
            break;
        }

        m_tearingAlgo->algoFracturePath(Pa, indexA, Pb, Pc, d_indexTriangleMaxStress.getValue(), principalStressDirection, input_position.getValue());
    }
}


template <class DataTypes>
void TearingEngine<DataTypes>::computeEndPoints(
    Coord Pa,
    Coord direction,
    Coord& Pb, Coord& Pc)
{
    Coord fractureDirection;
    fractureDirection[0] = - direction[1];
    fractureDirection[1] = direction[0];
    Real norm_fractureDirection = fractureDirection.norm();
    Pb = Pa + d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
    Pc = Pa - d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
}

template <class DataTypes>
void TearingEngine<DataTypes>::computeTriangleToSkip()
{
    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_triangleToIgnoreList);
    sofa::helper::vector<sofa::component::forcefield::ConstantForceField<DataTypes>*>  m_ConstantForceFields;
    this->getContext()->get< sofa::component::forcefield::ConstantForceField<DataTypes> >(&m_ConstantForceFields, sofa::core::objectmodel::BaseContext::SearchUp);

    for each (sofa::component::forcefield::ConstantForceField<DataTypes>* cff_i in m_ConstantForceFields)
    {
        vector<Index> vertexToSkip = cff_i->d_indices.getValue();
        for (unsigned int i = 0; i < vertexToSkip.size(); i++)
        {
            vector<Index> triangleAroundVertex_i = m_topology->getTrianglesAroundVertex(vertexToSkip[i]);
            for (unsigned int j = 0; j < triangleAroundVertex_i.size(); j++)
            {
                if (std::find(triangleToSkip.begin(), triangleToSkip.end(), triangleAroundVertex_i[j]) == triangleToSkip.end())
                    triangleToSkip.push_back(triangleAroundVertex_i[j]);
            }
        }
    }   
}

} //namespace sofa::component::engine
