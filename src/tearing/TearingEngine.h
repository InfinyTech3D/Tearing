#pragma once

#include <tearing/config.h>
#include <tearing/TearingAlgorithms.h>

#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/topology/BaseMeshTopology.h>

#include <SofaBaseTopology/TopologyData.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>
#include <SofaMiscFem/TriangularFEMForceField.h>
#include <SofaBaseTopology/TriangleSetTopologyModifier.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <SofaBoundaryCondition/ConstantForceField.h>

namespace sofa::helper
{
	class ColorMap;
}

namespace sofa::component::engine
{

	using core::DataEngine;
	using helper::vector;

template <class DataTypes>
class TearingEngine : public core::DataEngine
{
public:
	SOFA_CLASS(SOFA_TEMPLATE(TearingEngine, DataTypes), core::DataEngine);
	typedef typename DataTypes::Real Real;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::VecCoord VecCoord;

	typedef sofa::core::topology::BaseMeshTopology::Edge Edge;

	typedef sofa::core::topology::BaseMeshTopology::Index Index;
	typedef sofa::core::topology::BaseMeshTopology::Triangle Element;
	typedef sofa::core::topology::BaseMeshTopology::SeqTriangles VecElement;

	typedef typename sofa::component::forcefield::TriangularFEMForceField<DataTypes>::TriangleInformation TriangleFEMInformation;
	typedef sofa::helper::vector<TriangleFEMInformation> VecTriangleFEMInformation;

protected:
	TearingEngine();
	~TearingEngine() override {}
	typedef defaulttype::Mat<3, 3, Real > Transformation;				    ///< matrix for rigid transformations like rotations
	typedef defaulttype::Mat<6, 3, Real> StrainDisplacement;				    ///< the strain-displacement matrix
	typedef defaulttype::Mat<3, 3, Real > Transformation;				    ///< matrix for rigid transformations like rotations

public:
	void init() override;
	void reinit() override;
	void doUpdate() override;
	void draw(const core::visual::VisualParams* vparams) override;

	Data<VecCoord> input_position; ///< Input position
	Data<bool> showChangedTriangle;

	//Looking for triangle will tear first
	Data<double> d_seuilPrincipalStress; ///< threshold value for principal stress
	Data<vector<Index>> d_triangleOverThresholdList;
	Data<double> d_maxStress;
	Data<Index> d_indexTriangleMaxStress;
	Data<Index> d_indexVertexMaxStress;
	Data<bool> showTearableTriangle;
	Data<bool> stepByStep;
	Data<int> d_step;
	Data<int> d_counter;
	Data<double> d_fractureMaxLength;
	Data<vector<Index>> d_triangleToIgnoreList;
	Data<bool> ignoreTriangleAtStart;
	Data<vector<vector<int>>> d_TjunctionTriangle;

	/// <summary>
	/// put in d_triangleOverThresholdList triangle with a maxStress greater than a threshold value (d_seuilPrincipalStress)
	/// </summary>
	void triangleOverThresholdPrincipalStress();
	
	class TriangleInformation
	{
	public:
		//Real area;
		defaulttype::Vec<3, Real> stress;
		Real maxStress;

		TriangleInformation() { }

		/// Output stream
		inline friend std::ostream& operator<< (std::ostream& os, const TriangleInformation& /*ti*/)
		{
			return os;
		}

		/// Input stream
		inline friend std::istream& operator>> (std::istream& in, TriangleInformation& /*ti*/)
		{
			return in;
		}
	};
	Data<vector<TriangleInformation> > d_triangleInfoTearing;
	Data<VecTriangleFEMInformation> d_triangleFEMInfo;

	/// <summary>
	/// update d_triangleInfoTearing with value from d_triangleFEMInfo
	/// </summary>
	void updateTriangleInformation();

	/// Test algoFracturePath
	Data<int> d_fractureNumber;
	Data<int> d_nbFractureMax;

	/// <summary>
	/// compute fracture path intersection point and cut through them
	/// </summary>
	void algoFracturePath();

	Data<vector<Coord>> d_fracturePath;
	Data<bool> showFracturePath;

	/// <summary>
	/// compute extremities of fracture Pb and Pc from a start point Pa
	/// </summary>
	/// @param Pa - point with maxStress where fracture start
	/// @param direction - direction of maximum principal stress
	/// @return Pb - one of the extremities of fracture
	/// @return Pc - one of the extremities of fracture
	void computeEndPoints(Coord Pa, Coord direction, Coord& Pb, Coord& Pc);

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
	bool computeSegmentMeshIntersection(Coord Pa,Index indexA, Coord endPoint, bool& endPoint_inTriangle, Index& endPointTriangle, sofa::helper::vector<Index>& edges_list, sofa::helper::vector<double>& coordsEdge_list);
	
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
	void pathAdaptationObject(
		double EPS,
		bool pointB_inTriangle, Index triangleB, Coord Pb, sofa::helper::vector<Index> edges_listB, sofa::helper::vector<double> coordsEdge_listB, int& sizeB,
		Coord Pa, Index indexA,
		bool pointC_inTriangle, Index triangleC, Coord Pc, sofa::helper::vector<Index> edges_listC, sofa::helper::vector<double> coordsEdge_listC, int& sizeC,
		sofa::helper::vector< sofa::core::topology::TopologyElementType>& topoPath_list,
		sofa::helper::vector<Index>& indices_list,
		sofa::helper::vector< sofa::defaulttype::Vec<3, double> >& coords_list);

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
	int splitting(
		int snapingValue, int snapingBorderValue,
		Coord Pa, Coord Pb, Coord Pc,
		int sizeB, int sizeC,
		sofa::helper::vector< sofa::core::topology::TopologyElementType> topoPath_list,
		sofa::helper::vector<Index> indices_list,
		sofa::helper::vector< sofa::defaulttype::Vec<3, double> > coords_list,
		sofa::helper::vector< Index >& new_edges);
	
	/// Link to be set to the topology container in the component graph
	SingleLink<TearingEngine<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_topology;

	Data<int> d_scenario;

	/// <summary>
	/// compute ignored triangle at start of the tearing algo
	/// </summary>
	void computeTriangleToSkip();

protected:
	/// Pointer to the current topology
	sofa::core::topology::BaseMeshTopology* m_topology;
	sofa::component::topology::TriangleSetGeometryAlgorithms<DataTypes>* m_triangleGeo;
	sofa::component::forcefield::TriangularFEMForceField<DataTypes>* m_triangularFEM;
	sofa::component::topology::TriangleSetTopologyModifier* m_modifier;
private:
	sofa::helper::ColorMap* p_drawColorMap;

	TearingAlgorithms<DataTypes>* m_tearingAlgo;
public:
	void handleEvent(sofa::core::objectmodel::Event* event) override;

};
	
#if !defined(SOFA_COMPONENT_ENGINE_TEARINGENGINE_CPP)
extern template class TEARING_API TearingEngine<defaulttype::Vec3Types>;
#endif
		
}//namespace sofa::component::engine