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
    , showChangedTriangle(initData(&showChangedTriangle, false,"showChangedTriangle", "Flag activating rendering of changed triangle"))
    , showTearableTriangle(initData(&showTearableTriangle, true, "showTearableTriangle", "Flag activating rendering of fracturable triangle"))
    , d_triangleInfoTearing(initData(&d_triangleInfoTearing, "triangleInfoTearing", "Internal triangle data"))
    , d_triangleFEMInfo(initData(&d_triangleFEMInfo, "triangleFEMInfo", "Internal triangle data"))
    , d_maxStress(initData(&d_maxStress, "maxStress", "maxStress"))
    , d_indexTriangleMaxStress(initData(&d_indexTriangleMaxStress, "indexTriangleMaxStress", "index of triangle where the principal stress is maximum"))
    , d_indexVertexMaxStress(initData(&d_indexVertexMaxStress, "indexVertexMaxStress", "index of vertex where the stress is maximum"))
    , stepByStep(initData(&stepByStep, true, "stepByStep", "Flag activating step by step option for tearing"))
    , d_counter(initData(&d_counter, 0, "counter", "counter for the step by step option"))

    , showFracturePath(initData(&showFracturePath, true, "showFracturePath", "Flag activating rendering of fracture path"))
    , d_fractureMaxLength(initData(&d_fractureMaxLength, 2.0, "fractureMaxLength", "fracture max length by time step"))
    , d_fracturePath(initData(&d_fracturePath,"d_fracturePath","path created by algoFracturePath"))
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
    d_counter.setValue(0);
    
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
    computeArea();
    updateTriangleInformation();
    //triangleOverThresholdArea(); 
    triangleOverThresholdPrincipalStress();
    int step = 50;
    if ((d_counter.getValue() % step) == 0 || !stepByStep.getValue())
    {
        std::cout << "  enter fracture" << std::endl;
        if(d_counter.getValue()>step)
            algoFracturePath();
    }
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
        if (candidate.size() > 0)
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

template <class DataTypes>
void TearingEngine<DataTypes>::algoFracturePath()
{
    helper::ReadAccessor< Data<VecCoord> > x(input_position);
    helper::WriteAccessor< Data<vector<Coord>> > path(d_fracturePath);
    path.clear();
    double EPS = 1e-8;
    bool PATH_IS_OK = false;

    //On cherche le point de départ
    Coord principalStressDirection = d_triangleFEMInfo.getValue()[d_indexTriangleMaxStress.getValue()].principalStressDirection;
    Coord Pa = x[d_indexVertexMaxStress.getValue()];
    path.push_back(Pa);

    //On détermine les B et C, extrémités de la fracture
    Coord fractureDirection;
    fractureDirection[0] = -principalStressDirection[1];
    fractureDirection[1] = principalStressDirection[0];
    Real norm_fractureDirection = fractureDirection.norm();
    Coord Pb = Pa + d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
    Coord Pc = Pa - d_fractureMaxLength.getValue() / norm_fractureDirection * fractureDirection;
    

    //Côté C
    bool sideC_resumed = true;
    Coord current_point = Pa;
    Index current_triangle = m_triangleGeo->getTriangleInDirection(d_indexVertexMaxStress.getValue(), Pc - current_point);
    if (current_triangle > m_topology->getNbTriangles() - 1)
        sideC_resumed = false;
    sofa::helper::vector<Index> triangles_listC;
    Index ind_edgeC;
    sofa::helper::vector<Index> edges_listC;
    sofa::helper::vector< double > coordsEdge_listC;

    bool pointC_inTriangle = false;
    Index triangleC = -1;

    //début de la boucle
    while (sideC_resumed)
    {
        sofa::helper::vector<Index> candidateIndice;
        sofa::helper::vector<double> candidateBarycoef;
        sofa::helper::vector<double> candidateCoordKmin;
        bool intersection_exist = m_triangleGeo->computeSegmentTriangleIntersections(false, current_point, Pc, current_triangle, candidateIndice, candidateBarycoef, candidateCoordKmin);
        if (intersection_exist == false)
        {
            sideC_resumed = false;
            break;
        }


        //choisir parmis les candidats
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

        //on vérifie que nous n'avons pas dépassé l'extrémité
        if (candidateCoordKmin[j] >= 1)
        {
            pointC_inTriangle = true;
            triangleC = current_triangle;
            sideC_resumed = false;
            PATH_IS_OK = true;
            break;
        }
        Coord next_point = x[candidateIndice[2 * j]] + candidateBarycoef[j] * (x[candidateIndice[2 * j + 1]] - x[candidateIndice[2 * j]]);
        path.push_back(next_point);

        Index next_triangle = -1;
        if (candidateBarycoef[j] < EPS || abs(candidateBarycoef[j] - 1) < EPS)
        {
            //next_point est sur un sommet
            if (candidateBarycoef[j] < EPS)
            {
                next_triangle = m_triangleGeo->getTriangleInDirection(candidateIndice[2 * j], Pc - x[candidateIndice[2 * j]]);
            }
            else
            {
                next_triangle = m_triangleGeo->getTriangleInDirection(candidateIndice[2 * j + 1], Pc - x[candidateIndice[2 * j + 1]]);
            }
        }
        else
        {
            //next_point est sur un edge
            Index next_point_edgeId = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
            sofa::helper::vector<Index>next_triangle_candidate = m_topology->getTrianglesAroundEdge(next_point_edgeId);

            if (next_triangle_candidate.size() > 1)
                next_triangle = (current_triangle == next_triangle_candidate[0]) ? next_triangle_candidate[1] : next_triangle_candidate[0];

        }

        //si on est au bord, il n'y a pas de next_triangle
        if (next_triangle > m_topology->getNbTriangles() - 1)
        {
            triangles_listC.push_back(current_triangle);
            ind_edgeC = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
            edges_listC.push_back(ind_edgeC);
            Edge e = m_topology->getEdge(ind_edgeC);
            if (e[0] == candidateIndice[2 * j])
            {
                coordsEdge_listC.push_back(candidateBarycoef[j]);
            }
            else
            {
                coordsEdge_listC.push_back(1-candidateBarycoef[j]);
            }
            sideC_resumed = false;
            PATH_IS_OK = true;
            break;
        }

        //MAJ
        current_triangle = next_triangle;
        current_point = next_point;

        triangles_listC.push_back(current_triangle);
        ind_edgeC = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
        edges_listC.push_back(ind_edgeC);
        Edge e = m_topology->getEdge(ind_edgeC);
        if (e[0] == candidateIndice[2 * j])
        {
            coordsEdge_listC.push_back(candidateBarycoef[j]);
        }
        else
        {
            coordsEdge_listC.push_back(1-candidateBarycoef[j]);
        }
        PATH_IS_OK = true;
        candidateIndice.clear();
        candidateBarycoef.clear();
    }
    path.push_back(Pc);


    //Côté B
    bool sideB_resumed = true;
    current_point = Pa;
    current_triangle = m_triangleGeo->getTriangleInDirection(d_indexVertexMaxStress.getValue(), Pb - current_point);
    if (current_triangle > m_topology->getNbTriangles() - 1)
        sideB_resumed = false;
    sofa::helper::vector<Index> triangles_listB;
    Index ind_edgeB;
    sofa::helper::vector<Index> edges_listB;
    sofa::helper::vector< double > coordsEdge_listB;

    bool pointB_inTriangle = false;
    Index triangleB = -1;

    //début de la boucle
    while (sideB_resumed)
    {
        sofa::helper::vector<Index> candidateIndice;
        sofa::helper::vector<double> candidateBarycoef;
        sofa::helper::vector<double> candidateCoordKmin;
        bool intersection_exist = m_triangleGeo->computeSegmentTriangleIntersections(false, current_point, Pb, current_triangle, candidateIndice, candidateBarycoef, candidateCoordKmin);
        if (intersection_exist == false)
        {
            sideB_resumed = false;
            break;
        }

        //choisir parmis les candidats
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

        //on vérifie que nous n'avons pas dépassé l'extrémité
        if (candidateCoordKmin[j] >= 1)
        {
            pointB_inTriangle = true;
            triangleB = current_triangle;
            sideB_resumed = false;
            PATH_IS_OK = true;
            break;
        }
        Coord next_point = x[candidateIndice[2 * j]] + candidateBarycoef[j] * (x[candidateIndice[2 * j + 1]] - x[candidateIndice[2 * j]]);
        path.push_back(next_point);

        Index next_triangle = -1;
        if (candidateBarycoef[j] < EPS || abs(candidateBarycoef[j] - 1) < EPS)
        {
            //next_point est sur un sommet
            if (candidateBarycoef[j] < EPS)
            {
                next_triangle = m_triangleGeo->getTriangleInDirection(candidateIndice[2 * j], Pc - x[candidateIndice[2 * j]]);
            }
            else
            {
                next_triangle = m_triangleGeo->getTriangleInDirection(candidateIndice[2 * j + 1], Pc - x[candidateIndice[2 * j + 1]]);
            }
        }
        else
        {
            //next_point est sur un edge
            Index next_point_edgeId = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
            sofa::helper::vector<Index>next_triangle_candidate = m_topology->getTrianglesAroundEdge(next_point_edgeId);

            if (next_triangle_candidate.size() > 1)
                next_triangle = (current_triangle == next_triangle_candidate[0]) ? next_triangle_candidate[1] : next_triangle_candidate[0];

        }

        //si on est au bord, il n'y a pas de next_triangle
        if (next_triangle > m_topology->getNbTriangles() - 1)
        {
            triangles_listB.push_back(current_triangle);
            ind_edgeB = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
            edges_listB.push_back(ind_edgeB);
            Edge e = m_topology->getEdge(ind_edgeB);
            if (e[0] == candidateIndice[2 * j])
            {
                coordsEdge_listB.push_back(candidateBarycoef[j]);
            }
            else
            {
                coordsEdge_listB.push_back(1-candidateBarycoef[j]);
            }

            sideB_resumed = false;
            PATH_IS_OK = true;
            break;
        }

        //MAJ
        current_triangle = next_triangle;
        current_point = next_point;

        triangles_listB.push_back(current_triangle);
        ind_edgeB = m_topology->getEdgeIndex(candidateIndice[2 * j], candidateIndice[2 * j + 1]);
        edges_listB.push_back(ind_edgeB);
        Edge e = m_topology->getEdge(ind_edgeB);
        if (e[0] == candidateIndice[2 * j])
        {
            coordsEdge_listB.push_back(candidateBarycoef[j]);
        }
        else
        {
            coordsEdge_listB.push_back(1-candidateBarycoef[j]);
        }
        PATH_IS_OK = true;
        candidateIndice.clear();
        candidateBarycoef.clear();
    }
    path.push_back(Pb);


    if (PATH_IS_OK)
    {
    //equivalent STEP 4
    std::cout << "DEBUT STEP 4-------------------------------------"<< std::endl;
    int sizeB = triangles_listB.size();
    int sizeC = triangles_listC.size();
    std::cout << "    sizeB=" << sizeB << std::endl;
    std::cout << "    sizeC=" << sizeC << std::endl;
    
        //output de STEP 4
        sofa::helper::vector< sofa::core::topology::TopologyElementType> topoPath_list;
        sofa::helper::vector<Index> indices_list;
        sofa::helper::vector< sofa::defaulttype::Vec<3, double> > coords_list;
        sofa::defaulttype::Vec<3, double> baryCoords;

        //début de l'adaptation
        std::cout << "      pointB=" << std::endl;
        //doit on mettre le point B ?
        if (pointB_inTriangle)
        {
            //calcul des coo barycentric du point B dans le triangle B
            sofa::helper::vector< double > coefs_b = m_triangleGeo->computeTriangleBarycoefs(triangleB, Pb);

            //le point est-il sur un sommet
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

            if (B_isOnVertex) //le point B est sur un sommet
            {
                topoPath_list.push_back(core::topology::TopologyElementType::POINT);
                indices_list.push_back(indexPointB);
                coords_list.push_back(Pb);
            }
            else//le point B est dans le triangle
            {
                topoPath_list.push_back(core::topology::TopologyElementType::TRIANGLE);
                indices_list.push_back(triangleB);
                for (unsigned int i = 0; i < 3; i++)
                    baryCoords[i] = coefs_b[i];
                coords_list.push_back(baryCoords);
            }
        } //pointB_inTriangle
        std::cout << "      fin pointB=" << std::endl;

        //point d'intersection entre B et A
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

        std::cout << "      pointA=" << std::endl;
        //ajout du point A
        //Index triangleA;
        //if (sizeC > 0)
        //{
        //    triangleA = m_triangleGeo->getTriangleInDirection(d_indexVertexMaxStress.getValue(), Pc - Pa);
        //}
        //std::cout << "          triangleA=" << triangleA<< std::endl;
        //if (!(triangleA > m_topology->getNbTriangles() - 1))
        //{
        //    std::cout << "              in=" << std::endl;
            topoPath_list.push_back(core::topology::TopologyElementType::POINT);
            indices_list.push_back(d_indexVertexMaxStress.getValue());
            coords_list.push_back(Pa);
        //}
        //-----------------------------------------------------
        
        std::cout << "      fin pointA=" << std::endl;

        //point d'intersection entre A et C
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

        std::cout << "      pointC=" << std::endl;
        //doit on mettre le point C ?
        if (pointC_inTriangle)
        {
            //calcul des coo barycentric du point C dans le triangle C
            sofa::helper::vector< double > coefs_c = m_triangleGeo->computeTriangleBarycoefs(triangleC, Pc);

            //le point est-il sur un sommet
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

            if (C_isOnVertex) //le point C est sur un sommet
            {
                topoPath_list.push_back(core::topology::TopologyElementType::POINT);
                indices_list.push_back(indexPointC);
                coords_list.push_back(Pc);
            }
            else//le point C est dans le triangle
            {
                topoPath_list.push_back(core::topology::TopologyElementType::TRIANGLE);
                indices_list.push_back(triangleC);
                for (unsigned int i = 0; i < 3; i++)
                    baryCoords[i] = coefs_c[i];
                coords_list.push_back(baryCoords);
            }
        } //pointC_inTriangle
        std::cout << "      fin pointC=" << std::endl;

        std::cout << "          indices_list=" << indices_list << std::endl;
        triangles_listB.clear();
        edges_listB.clear();
        coordsEdge_listB.clear();
        triangles_listC.clear();
        edges_listC.clear();
        coordsEdge_listC.clear();
        std::cout << "FIN STEP 4-------------------------------------" << std::endl;

        if (topoPath_list.size() > 1)
        {
            //STEP 5: Splitting elements along path (incision path is stored inside "new_edges")
            std::cout << "DEBUT STEP 5-------------------------------------" << std::endl;
            int snapingValue = 20;
            int snapingBorderValue = 0;
            // Snaping value: input are percentages, we need to transform it as real epsilon value;
            double epsilonSnap = (double)snapingValue / 200;
            double epsilonBorderSnap = (double)snapingBorderValue / 210; // magic number (0.5 is max value and must not be reached, as threshold is compared to barycoord value)
            sofa::helper::vector< Index > new_edges;
            int result = -1;
            if (sizeB == 0)
            {
                result = m_triangleGeo->SplitAlongPath(core::topology::BaseMeshTopology::InvalidID, Pa, core::topology::BaseMeshTopology::InvalidID, Pc, topoPath_list, indices_list, coords_list, new_edges, epsilonSnap, epsilonBorderSnap);
            }
            else if (sizeC==0)
            {
                result = m_triangleGeo->SplitAlongPath(core::topology::BaseMeshTopology::InvalidID, Pb, core::topology::BaseMeshTopology::InvalidID, Pa, topoPath_list, indices_list, coords_list, new_edges, epsilonSnap, epsilonBorderSnap);
            }
            else
            {
                result = m_triangleGeo->SplitAlongPath(core::topology::BaseMeshTopology::InvalidID, Pb, core::topology::BaseMeshTopology::InvalidID, Pc, topoPath_list, indices_list, coords_list, new_edges, epsilonSnap, epsilonBorderSnap);
            }
            std::cout << "              results SplitAlongPath=" << result << std::endl;
            if (result == -1)
            {
                // incision.indexPoint = last_indexPoint;
                return;
            }

            std::cout << "FIN STEP 5-------------------------------------" << std::endl;



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
            std::cout << "FIN STEP 6-------------------------------------" << std::endl;
        }


        topoPath_list.clear();
        indices_list.clear();
        coords_list.clear();
    }
}

} //namespace sofa::component::engine
