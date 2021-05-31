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
    , d_fracturePath(initData(&d_fracturePath,"fracturePath","path created by algoFracturePath"))
    , d_fractureNumber(initData(&d_fractureNumber, 0, "fractureNumber", "number of fracture done by the algorithm"))
    , d_nbFractureMax(initData(&d_nbFractureMax, 15, "nbFractureMax", "number of fracture max done by the algorithm"))
    , d_scenario(initData(&d_scenario, 0, "scenario", "choose scenario, zero is default"))
    , ignoreTriangleAtStart(initData(&ignoreTriangleAtStart, true, "ignoreTriangleAtStart","option to ignore some triangles at start of the tearing algo"))
    , d_TjunctionTriangle(initData(&d_TjunctionTriangle, "TjunctionTriangle", "list of triangle where a T junction is blocking the algorithm"))
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
    addOutput(&d_fractureNumber);
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
        msg_error() << "Missing component: Unable to get TriangleSetGeometryAlgorithms from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    m_topology->getContext()->get(m_modifier);
    if (!m_modifier)
    {
        msg_error() << "Missing component: Unable to get TriangleSetGeometryAlgorithms from the current context.";
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        return;
    }

    if (ignoreTriangleAtStart.getValue())
        computeTriangleToSkip();
    updateTriangleInformation();
    triangleOverThresholdPrincipalStress();
    d_counter.setValue(0);
    d_fractureNumber.setValue(0);
    
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
        if(d_fractureNumber.getValue() == 0)
            computeTriangleToSkip();
    }
    else
    {
        vector<Index> emptyIndexList;
        d_triangleToIgnoreList.setValue(emptyIndexList);
    }

    helper::ReadAccessor< Data<vector<vector<int>>> > TjunctionTriangle(d_TjunctionTriangle);
    helper::WriteAccessor< Data<vector<Index>> >triangleToSkip(d_triangleToIgnoreList);
    for (unsigned int i = 0; i < TjunctionTriangle.size(); i++)
    {
        if (TjunctionTriangle[i][0] == d_fractureNumber.getValue())
        {
            if (std::find(triangleToSkip.begin(), triangleToSkip.end(), TjunctionTriangle[i][1]) == triangleToSkip.end())
                triangleToSkip.push_back(TjunctionTriangle[i][1]);
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

            helper::ReadAccessor< Data<vector<Coord>> > path(d_fracturePath);
            if (path.size())
                vparams->drawTool()->drawPoints(path, 10, sofa::helper::types::RGBAColor(0, 1, 0, 1));
        }
    }
}

template <class DataTypes>
void TearingEngine<DataTypes>::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (/* simulation::AnimateBeginEvent* ev = */simulation::AnimateBeginEvent::checkEventType(event))
    {
        if ( ((d_counter.getValue() % d_step.getValue()) == 0) && (d_fractureNumber.getValue()< d_nbFractureMax.getValue()) || !stepByStep.getValue())
        {
            std::cout << "  enter fracture" << std::endl;
            if(d_counter.getValue()>d_step.getValue())
                algoFracturePath();
        }
    }
}



// --------------------------------------------------------------------------------------
// --- Computation methods
// --------------------------------------------------------------------------------------

/// <summary>
/// put in d_triangleOverThresholdList triangle with a maxStress greater than a threshold value (d_seuilPrincipalStress)
/// </summary>
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

/// <summary>
/// update d_triangleInfoTearing with value from d_triangleFEMInfo
/// </summary>
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

/// <summary>
/// compute fracture path intersection point and cut through them
/// </summary>
template <class DataTypes>
void TearingEngine<DataTypes>::algoFracturePath()
{
    helper::ReadAccessor< Data<vector<Index>> > candidate(d_triangleOverThresholdList);
    int choice;
    if (d_fractureNumber.getValue() == 0)
    {
        choice = d_scenario.getValue();
    }
    else
    {
        choice = 0;
    }

    if (candidate.size() || choice)
    {
        helper::ReadAccessor< Data<VecCoord> > x(input_position);
        helper::WriteAccessor< Data<vector<Coord>> > path(d_fracturePath);
        path.clear();
        double EPS = 1e-8;
        bool PATH_IS_OK = false;


        //looking for starting point
        int indexA;
        Coord Pa;
        Coord principalStressDirection;
        Coord Pb;
        Coord Pc; 
        Coord dir;
        double alpha;
        
        //scenario
        switch (choice)
        {
        default:
            indexA = d_indexVertexMaxStress.getValue();
            Pa = x[indexA];
            path.push_back(Pa);
            principalStressDirection = d_triangleFEMInfo.getValue()[d_indexTriangleMaxStress.getValue()].principalStressDirection;
            computeEndPoints(Pa, principalStressDirection, Pb, Pc);
            break;

        case 1 :
            //CasTest1-1
            indexA = 421;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 2.5;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;
        
        case 2 :
            //CasTest1-2
            indexA = 1;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 0.707; dir[1] = 0.707; dir[2] = 0.0;
            alpha = 3.5;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 3:
            //CasTest2-1
            indexA = 470;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 4.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 4:
            //CasTest2-2
            indexA = 84;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 0.707; dir[1] = 0.707; dir[2] = 0.0;
            alpha = 3.5;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 5:
            //CasTest4-1
            indexA = 45;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 2.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 6:
            //CasTest4-1
            indexA = 51;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 0.0; dir[1] = 1.0; dir[2] = 0.0;
            alpha = 5.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 7:
            //CasTest5_holeCircular-1
            indexA = 318;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 0.0; dir[1] = 1.0; dir[2] = 0.0;
            alpha = 2.5;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 8:
            //CasTest5_holeCircular-2
            indexA = 282;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 0.707; dir[1] = -0.707; dir[2] = 0.0;
            alpha = 2.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - alpha * dir;
            break;

        case 9:
            //CasTest5_holeSquare
            indexA = 231;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 0.0; dir[1] = 1.0; dir[2] = 0.0;
            alpha = 5.0;
            Pb = Pa + alpha * dir;
            Pc = Pa - dir;
            break;

        case 10:
            //CasTest5_holeSquareVertical
            indexA = 229;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 2.0;
            Pb = Pa - alpha * dir;
            Pc = Pa + dir;
            break;

        case 11:
            //CasTest5_incision-1
            indexA = 179;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 1.0; dir[1] = 0.0; dir[2] = 0.0;
            alpha = 2.0;
            Pb = Pa - alpha * dir;
            Pc = Pa + alpha * dir;
            break;

        case 12:
            //CasTest5_incision-2
            indexA = 188;
            Pa = x[indexA];
            path.push_back(Pa);
            dir[0] = 0.0; dir[1] = 1.0; dir[2] = 0.0;
            alpha = 5.0;
            Pb = Pa;
            Pc = Pa - alpha * dir;
            break;
        }


        //Side C
        bool sideC_resumed = true;
        Index current_triangle = m_triangleGeo->getTriangleInDirection(indexA, Pc - Pa);
        bool triangleInDirectionC = true;
        if (current_triangle > m_topology->getNbTriangles() - 1)
        {
            sideC_resumed = false;
            triangleInDirectionC = false;
        }
        bool pointC_inTriangle = false;
        Index triangleC = -1;
        sofa::helper::vector<Index> edges_listC;
        sofa::helper::vector< double > coordsEdge_listC;

        bool PATH_C_IS_OK = false;
        if (sideC_resumed)
            PATH_C_IS_OK = computeSegmentMeshIntersection(Pa, indexA, Pc, pointC_inTriangle, triangleC, edges_listC, coordsEdge_listC);
        path.push_back(Pc);

        //Side B
        bool sideB_resumed = true;
        current_triangle = m_triangleGeo->getTriangleInDirection(indexA, Pb - Pa);
        bool triangleInDirectionB = true;
        if (current_triangle > m_topology->getNbTriangles() - 1)
        {
            sideB_resumed = false;
            triangleInDirectionB = false;
        }

        bool pointB_inTriangle = false;
        Index triangleB = -1;
        sofa::helper::vector<Index> edges_listB;
        sofa::helper::vector< double > coordsEdge_listB;

        bool PATH_B_IS_OK = false;
        if (sideB_resumed)
            PATH_B_IS_OK = computeSegmentMeshIntersection(Pa, indexA, Pb, pointB_inTriangle, triangleB, edges_listB, coordsEdge_listB);
        path.push_back(Pb);


        if (PATH_C_IS_OK || PATH_B_IS_OK)
        {
            //output de STEP 4
            sofa::helper::vector< sofa::core::topology::TopologyElementType> topoPath_list;
            sofa::helper::vector<Index> indices_list;
            sofa::helper::vector< sofa::defaulttype::Vec<3, double> > coords_list;
            int sizeB, sizeC;

            std::cout << "DEBUT STEP 4-------------------------------------" << std::endl;
            pathAdaptationObject(
                EPS,
                pointB_inTriangle, triangleB, Pb, edges_listB, coordsEdge_listB, sizeB,
                Pa, indexA,
                pointC_inTriangle, triangleC, Pc, edges_listC, coordsEdge_listC, sizeC,
                topoPath_list, indices_list, coords_list);
            std::cout << "FIN STEP 4-------------------------------------" << std::endl;

            if (topoPath_list.size() > 1)
            {
                //STEP 5: Splitting elements along path (incision path is stored inside "new_edges")
                int snapingValue = 20;
                int snapingBorderValue = 0;
                sofa::helper::vector< Index > new_edges;
                int result;
                result = splitting(snapingValue, snapingBorderValue, Pa, Pb, Pc, sizeB, sizeC, topoPath_list, indices_list, coords_list, new_edges);

                if (result > 0)
                {
                    //STEP 6: Incise along new_edges path (i.e duplicating edges to create an incision)
                    std::cout << "DEBUT STEP 6-------------------------------------" << std::endl;
                    sofa::helper::vector<Index> new_points;
                    sofa::helper::vector<Index> end_points;
                    bool reachBorder = false;
                    bool incision_ok = m_triangleGeo->InciseAlongEdgeList(new_edges, new_points, end_points, reachBorder);
                    if (!incision_ok)
                    {
                        dmsg_error("TopologicalChangeManager") << " in InciseAlongEdgeList";
                        return;
                    }
                    d_fractureNumber.setValue(d_fractureNumber.getValue()+1);
                    std::cout << "FIN STEP 6-------------------------------------" << std::endl;
                }
            }


            topoPath_list.clear();
            indices_list.clear();
            coords_list.clear();
        }
        else if (!triangleInDirectionB && !triangleInDirectionC && m_topology->getTrianglesAroundVertex(indexA).size() > 1)
        {
            std::cout << "DEBUT T jUNCTION SABLIER-------------------------------------" << std::endl;
            sofa::helper::vector<Index> indexTriangleList = m_topology->getTrianglesAroundVertex(indexA);
            sofa::helper::vector<Index> indexTriangleListSide1;
            sofa::helper::vector<Index> indexTriangleListSide2;
            VecElement triangleList2;


            //get index 
            Index indexTriangleA = d_indexTriangleMaxStress.getValue();
            indexTriangleListSide1.push_back(indexTriangleA);
            Element t = m_topology->getTriangle(indexTriangleA);
            int k0= (t[0] == indexA) ? 0 : 1;
            k0 = (t[k0] == indexA) ? k0 : 2;

            Coord normal = d_triangleFEMInfo.getValue()[d_indexTriangleMaxStress.getValue()].principalStressDirection;
            Coord p0a = x[t[k0]];
            Coord p1a = x[t[(3 + (k0 + 1)) % 3]];
            Coord vec_a = p1a - p0a;

            for (unsigned int i = 0; i < indexTriangleList.size(); i++)
            {
                if (indexTriangleList[i] != indexTriangleA)
                {
                    Element t_i = m_topology->getTriangle(indexTriangleList[i]);
                    int k_i = (t_i[0] == indexA) ? 0 : 1;
                    k_i = (t_i[k_i] == indexA) ? k_i : 2;

                    Coord p0b = x[t_i[k_i]];
                    Coord p1b = x[t_i[(3 + (k_i + 1)) % 3]];
                    Coord vec_b = p1b - p0b;
                    
                    if ((vec_a * normal) * (vec_b * normal) > 0)
                    {
                        indexTriangleListSide1.push_back(indexTriangleList[i]);
                    }
                    else
                    {
                        indexTriangleListSide2.push_back(indexTriangleList[i]);
                        triangleList2.push_back(m_topology->getTriangle(indexTriangleList[i]));
                    }
                }
            }

            if (indexTriangleListSide2.size()>0)
            {
                Index indexNewPoint=m_topology->getNbPoints();
                Index indexNewTriangle = m_topology->getNbTriangles();

                sofa::helper::vector<Index> indexs_ancestor;
                indexs_ancestor.push_back(indexA);
                sofa::helper::vector< sofa::helper::vector<Index> > ancestors;
                ancestors.push_back(indexs_ancestor);
                sofa::helper::vector<double> barycoefs;
                barycoefs.push_back(1.0);
                sofa::helper::vector< sofa::helper::vector<double> > coefs;
                coefs.push_back(barycoefs);
                m_modifier->addPoints(1, ancestors, coefs);

                
                sofa::helper::vector< Element > triangles2Add;
                sofa::helper::vector< Index > trianglesIndex2Add;
                sofa::helper::vector< sofa::helper::vector< Index > > triangles_ancestors;
                sofa::helper::vector< sofa::helper::vector< SReal > > triangles_baryCoefs;
                barycoefs.clear();
                barycoefs.push_back(1.0);
                barycoefs.push_back(1.0);
                barycoefs.push_back(1.0);
                for (unsigned int i = 0; i < triangleList2.size(); i++)
                {
                    Element t = triangleList2[i];
                    Element new_triangle;
                    for(unsigned int j = 0; j < 3; j++)
                    { 
                        if (t[j] == indexA)
                        {
                            new_triangle[j] = indexNewPoint;
                        }
                        else
                        {
                            new_triangle[j] = t[j];
                        }
                    }    
                    triangles2Add.push_back(new_triangle);
                    trianglesIndex2Add.push_back(indexNewTriangle++);
                    triangles_ancestors.resize(triangles_ancestors.size() + 1);
                    triangles_baryCoefs.resize(triangles_baryCoefs.size() + 1);
                    triangles_ancestors[triangles_ancestors.size() - 1].push_back(indexTriangleListSide2[i]);
                    triangles_baryCoefs[triangles_baryCoefs.size() - 1].push_back(1.0);

                }
                m_modifier->addRemoveTriangles(indexTriangleListSide2.size(), triangles2Add, trianglesIndex2Add, triangles_ancestors, triangles_baryCoefs, indexTriangleListSide2);
                d_fractureNumber.setValue(d_fractureNumber.getValue() + 1);
            }
            else
            {
                std::cout << "                  TjunctionTriangle add" << std::endl;
                helper::WriteAccessor< Data<vector<vector<int>>> > TjunctionTriangle(d_TjunctionTriangle);
                int a = d_fractureNumber.getValue();
                int b = d_indexTriangleMaxStress.getValue();
                TjunctionTriangle.push_back({ {a},{b} });
                std::cout << "          fractureNumber" << a << std::endl;
                std::cout << "          indexTriangleA" << b << std::endl;

            }
            std::cout << "FIN T jUNCTION SABLIER-------------------------------------" << std::endl;
        }
    }
}


/// <summary>
/// compute extremities of fracture Pb and Pc from a start point Pa
/// </summary>
/// @param Pa - point with maxStress where fracture start
/// @param direction - direction of maximum principal stress
/// @return Pb - one of the extremities of fracture
/// @return Pc - one of the extremities of fracture
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

/// <summary>
/// get intersection point between Pa and one of the endPoint Pb or Pc
/// </summary>
/// @param Pa - point with maxStress 
/// @param indexA - index of vertex of point Pa
/// @param endPoint - point Pb or Pc
/// @param endPoint_inTriangle - boolean tell if endPoint is in an triangle
/// @param endPointTriangle - index of endPoint triangle
/// @param edges_list - list of edges intersect by the segment
/// @param coordsEdge_list - list of baryCoef for intersected edges
template <class DataTypes>
bool TearingEngine<DataTypes>::computeSegmentMeshIntersection(
    Coord Pa,
    Index indexA,
    Coord endPoint,
    bool& endPoint_inTriangle,
    Index& endPointTriangle,
    sofa::helper::vector<Index>& edges_list,
    sofa::helper::vector<double>& coordsEdge_list)
{
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    helper::WriteAccessor< Data<vector<Coord>> > path(d_fracturePath);
    bool PATH_IS_OK = false;
    double EPS = 1e-8;
    sofa::helper::vector<Index> triangle_list;

    bool resume = true;
    Coord current_point = Pa;
    Index current_triangle = m_triangleGeo->getTriangleInDirection(indexA, endPoint - current_point);
    triangle_list.push_back(current_triangle);
    if (current_triangle > m_topology->getNbTriangles() - 1)
        resume = false;
    Index ind_edge;

    //loop start
    while (resume)
    {
        sofa::helper::vector<Index> candidateIndice;
        sofa::helper::vector<double> candidateBarycoef;
        sofa::helper::vector<double> candidateCoordKmin;
        bool intersection_exist = m_triangleGeo->computeIntersectionsLineTriangle(false, current_point, endPoint, current_triangle, candidateIndice, candidateBarycoef, candidateCoordKmin);
        if (intersection_exist == false)
        {
            resume = false;
            return PATH_IS_OK;
        }


        //choose in candidats
        int j = -1;
        if (candidateBarycoef.size() > 1)
        {
            for (unsigned int i = 0; i < candidateBarycoef.size(); i++)
            {
                Coord next_point_candidat = x[candidateIndice[2 * i]] + candidateBarycoef[i] * (x[candidateIndice[2 * i + 1]] - x[candidateIndice[2 * i]]);
                if ((current_point - next_point_candidat) * (current_point - next_point_candidat) > EPS)
                {
                    j = i;
                    break;
                }
            }
        }
        else
        {
            j = 0;
        }

        //check if endPoint is passed
        if (candidateCoordKmin[j] >= 1)
        {
            endPoint_inTriangle = true;
            endPointTriangle = current_triangle;
            resume = false;
            PATH_IS_OK = true;
            return PATH_IS_OK;
        }
        Coord next_point = x[candidateIndice[2 * j]] + candidateBarycoef[j] * (x[candidateIndice[2 * j + 1]] - x[candidateIndice[2 * j]]);
        //std::cout << "                      next point=" << next_point << std::endl;
        path.push_back(next_point);

        Index next_triangle = -1;
        if (candidateBarycoef[j] < EPS || abs(candidateBarycoef[j] - 1) < EPS)
        {
            //next_point is on an vertex
            if (candidateBarycoef[j] < EPS)
            {
                next_triangle = m_triangleGeo->getTriangleInDirection(candidateIndice[2 * j], endPoint - x[candidateIndice[2 * j]]);
            }
            else
            {
                next_triangle = m_triangleGeo->getTriangleInDirection(candidateIndice[2 * j + 1], endPoint - x[candidateIndice[2 * j + 1]]);
            }
        }
        else
        {
            //next_point is on an edge
            Index next_point_edgeId = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
            sofa::helper::vector<Index>next_triangle_candidate = m_topology->getTrianglesAroundEdge(next_point_edgeId);

            if (next_triangle_candidate.size() > 1)
                next_triangle = (current_triangle == next_triangle_candidate[0]) ? next_triangle_candidate[1] : next_triangle_candidate[0];

        }

        //if on an border, there is no next_triangle
        if (next_triangle > m_topology->getNbTriangles() - 1)
        {
            ind_edge = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
            edges_list.push_back(ind_edge);
            Edge e = m_topology->getEdge(ind_edge);
            if (e[0] == candidateIndice[2 * j])
            {
                coordsEdge_list.push_back(candidateBarycoef[j]);
            }
            else
            {
                coordsEdge_list.push_back(1 - candidateBarycoef[j]);
            }
            resume = false;
            PATH_IS_OK = true;
            return PATH_IS_OK;
        }

        //check if we are not all ready pass in this triangle
        for (unsigned int i = 0; i < triangle_list.size(); i++)
        {
            if (next_triangle == triangle_list[i])
                return PATH_IS_OK;
        }

        //MAJ
        current_triangle = next_triangle;
        current_point = next_point;
        triangle_list.push_back(current_triangle);

        ind_edge = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
        edges_list.push_back(ind_edge);
        Edge e = m_topology->getEdge(ind_edge);
        if (e[0] == candidateIndice[2 * j])
        {
            coordsEdge_list.push_back(candidateBarycoef[j]);
        }
        else
        {
            coordsEdge_list.push_back(1 - candidateBarycoef[j]);
        }
        PATH_IS_OK = true;
        candidateIndice.clear();
        candidateBarycoef.clear();
    }
}

/// <summary>
/// creating path through different element POINT or EDGE or TRIANGLE
/// </summary>
/// @param EPS - value for zero
/// @param pointB_inTriangle - boolean tell if Pb is in an triangle
/// @param triangleB - index of triangle where Pb is
/// @param Pb - coord of Pb
/// @param edges_listB - list of edges intersect by the segment Pa to Pb
/// @param coordsEdge_listB - list of baryCoef for intersected edges on sideB
/// @param sizeB - number of edges intersect on sideB
/// @param Pa - coord of Pa, point with maxStress
/// @param indexA - index of vertex Pa
/// @param pointC_inTriangle - boolean tell if Pc is in an triangle
/// @param triangleC - index of triangle where Pc is
/// @param Pc - coord of Pc
/// @param edges_listC - list of edges intersect by the segment Pa to Pc
/// @param coordsEdge_listC - list of baryCoef for intersected edges on sideC
/// @param sizeC - number of edges intersect on sideC
/// @return topoPath_list - List of object intersect
/// @return indices_list - List of indices of these objetcs
/// @return coords_list - List of barycentric coordinate defining the position of the intersection in each object
template <class DataTypes>
void TearingEngine<DataTypes>::pathAdaptationObject(
    double EPS,
    bool pointB_inTriangle, Index triangleB, Coord Pb, sofa::helper::vector<Index> edges_listB, sofa::helper::vector<double> coordsEdge_listB, int& sizeB,
    Coord Pa, Index indexA,
    bool pointC_inTriangle, Index triangleC, Coord Pc, sofa::helper::vector<Index> edges_listC, sofa::helper::vector<double> coordsEdge_listC, int& sizeC,
    sofa::helper::vector< sofa::core::topology::TopologyElementType>& topoPath_list,
    sofa::helper::vector<Index>& indices_list,
    sofa::helper::vector< sofa::defaulttype::Vec<3, double> >& coords_list)
{
    sofa::defaulttype::Vec<3, double> baryCoords;
    //equivalent STEP 4
    sizeB = edges_listB.size();
    sizeC = edges_listC.size();

    //adaptation start
    //add point B ?
    if (pointB_inTriangle)
    {
        //compute barycoef of Pb in triangleB
        sofa::helper::vector< double > coefs_b = m_triangleGeo->computeTriangleBarycoefs(triangleB, Pb);

        //Pb is on an vertex ? 
        bool B_isOnVertex = false;
        Index indexPointB = -1;
        for (unsigned int i = 0; i < coefs_b.size(); i++)
        {
            if (abs(coefs_b[i] - 1.0) < EPS)
            {
                indexPointB = m_topology->getTriangle(triangleB)[i];
                B_isOnVertex = true;
                break;
            }
        }

        if (B_isOnVertex) //Pb is on an vertex
        {
            topoPath_list.push_back(core::topology::TopologyElementType::POINT);
            indices_list.push_back(indexPointB);
            coords_list.push_back(Pb);
        }
        else//Pb is in an triangle
        {
            topoPath_list.push_back(core::topology::TopologyElementType::TRIANGLE);
            indices_list.push_back(triangleB);
            for (unsigned int i = 0; i < 3; i++)
                baryCoords[i] = coefs_b[i];
            coords_list.push_back(baryCoords);
        }
    } //pointB_inTriangle

    //intersection between B and A
    if (sizeB > 0)
    {
        for (unsigned int i = 0; i < sizeB; i++)
        {
            topoPath_list.push_back(core::topology::TopologyElementType::EDGE);
            indices_list.push_back(edges_listB[sizeB - 1 - i]);
            baryCoords[0] = coordsEdge_listB[sizeB - 1 - i];
            baryCoords[1] = 0.0;
            baryCoords[2] = 0.0;
            coords_list.push_back(baryCoords);
        }
    }

    //add Pa
    topoPath_list.push_back(core::topology::TopologyElementType::POINT);
    indices_list.push_back(indexA);
    coords_list.push_back(Pa);

    //intersection between A and C
    if (sizeC > 0)
    {
        for (unsigned int i = 0; i < sizeC; i++)
        {
            topoPath_list.push_back(core::topology::TopologyElementType::EDGE);
            indices_list.push_back(edges_listC[i]);
            Edge e = m_topology->getEdge(edges_listC[i]);
            baryCoords[0] = coordsEdge_listC[i];
            baryCoords[1] = 0.0;
            baryCoords[2] = 0.0;
            coords_list.push_back(baryCoords);
        }
    }

    //add point C ?
    if (pointC_inTriangle)
    {
        //compute barycoef of Pc in triangleC
        sofa::helper::vector< double > coefs_c = m_triangleGeo->computeTriangleBarycoefs(triangleC, Pc);

        //Pc is on an vertex ? 
        bool C_isOnVertex = false;
        Index indexPointC = -1;
        for (unsigned int i = 0; i < coefs_c.size(); i++)
        {
            if (abs(coefs_c[i] - 1.0) < EPS)
            {
                indexPointC = m_topology->getTriangle(triangleC)[i];
                C_isOnVertex = true;
                break;
            }
        }

        if (C_isOnVertex) //Pc is on an vertex
        {
            topoPath_list.push_back(core::topology::TopologyElementType::POINT);
            indices_list.push_back(indexPointC);
            coords_list.push_back(Pc);
        }
        else//Pc is in an triangle
        {
            topoPath_list.push_back(core::topology::TopologyElementType::TRIANGLE);
            indices_list.push_back(triangleC);
            for (unsigned int i = 0; i < 3; i++)
                baryCoords[i] = coefs_c[i];
            coords_list.push_back(baryCoords);
        }
    } //pointC_inTriangle

    edges_listB.clear();
    coordsEdge_listB.clear();
    edges_listC.clear();
    coordsEdge_listC.clear();
}

/// <summary>
/// Split triangles to create edges along a path given as a the list of existing edges and triangles crossed by it
/// </summary>
/// @param snapingValue - snaping value
/// @param snapingBorderValue - snaping border value
/// @param Pa - maxStress
/// @param Pb - extremity of fracture
/// @param Pc - extremity of fracture
/// @param sizeB - number of edges intersect on sideB
/// @param sizeC - number of edges intersect on sideC
/// @param topoPath_list - List of object intersect
/// @param indices_list - List of indices of these objetcs
/// @param coords_list - List of barycentric coordinate defining the position of the intersection in each object
/// @returns new_edges - the indice of the end point, or -1 if the incision failed
template <class DataTypes>
int TearingEngine<DataTypes>::splitting(
    int snapingValue, int snapingBorderValue,
    Coord Pa, Coord Pb, Coord Pc,
    int sizeB, int sizeC,
    sofa::helper::vector< sofa::core::topology::TopologyElementType> topoPath_list,
    sofa::helper::vector<Index> indices_list,
    sofa::helper::vector< sofa::defaulttype::Vec<3, double> > coords_list,
    sofa::helper::vector< Index >& new_edges)
{

    // Snaping value: input are percentages, we need to transform it as real epsilon value;
    double epsilonSnap = (double)snapingValue / 200;
    double epsilonBorderSnap = (double)snapingBorderValue / 210; // magic number (0.5 is max value and must not be reached, as threshold is compared to barycoord value)
    
    int result = -1;
    if (sizeB == 0)
    {
        result = m_triangleGeo->SplitAlongPath(core::topology::BaseMeshTopology::InvalidID, Pa, core::topology::BaseMeshTopology::InvalidID, Pc, topoPath_list, indices_list, coords_list, new_edges, epsilonSnap, epsilonBorderSnap);
    }
    else if (sizeC == 0)
    {
        result = m_triangleGeo->SplitAlongPath(core::topology::BaseMeshTopology::InvalidID, Pb, core::topology::BaseMeshTopology::InvalidID, Pa, topoPath_list, indices_list, coords_list, new_edges, epsilonSnap, epsilonBorderSnap);
    }
    else
    {
        result = m_triangleGeo->SplitAlongPath(core::topology::BaseMeshTopology::InvalidID, Pb, core::topology::BaseMeshTopology::InvalidID, Pc, topoPath_list, indices_list, coords_list, new_edges, epsilonSnap, epsilonBorderSnap);
    }
    return result;
}

/// <summary>
/// compute ignored triangle at start of the tearing algo
/// </summary>
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
