#pragma once

#include <tearing/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <SofaBaseTopology/TriangleSetTopologyModifier.h>
#include <SofaBaseTopology/TriangleSetGeometryAlgorithms.h>

namespace sofa::component
{

template <class DataTypes>
class TearingAlgorithms
{
public:
	typedef typename DataTypes::Real Real;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::VecCoord VecCoord;

	typedef sofa::core::topology::BaseMeshTopology::Index Index;
	typedef sofa::core::topology::BaseMeshTopology::Edge Edge;	
	typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;

	TearingAlgorithms(sofa::core::topology::BaseMeshTopology* _topology,
		sofa::component::topology::TriangleSetTopologyModifier* _modifier,
		sofa::component::topology::TriangleSetGeometryAlgorithms<DataTypes>* _triangleGeo);

	virtual ~TearingAlgorithms();
	

	/// <summary>
	/// compute fracture path intersection point and cut through them
	/// </summary>
	/// @param Pa - point with maxStress 
	/// @param indexA - index of vertex of point Pa
	/// @param Pb - coord of Pb
	/// @param Pc - coord of Pc
	/// @param indexTriangleMaxStress - index of triangle where the principal stress is maximum
	void algoFracturePath(Coord Pa, Index indexA, Coord Pb, Coord Pc, 
		const Index indexTriangleMaxStress, const Coord principalStressDirection, const VecCoord& input_position);

	int getFractureNumber() const { return m_fractureNumber; }

	const sofa::helper::vector< sofa::helper::vector<int> >& getTjunctionTriangles() const { return m_TjunctionTriangle; }

	const sofa::helper::vector<Coord>& getFracturePath() const { return m_fracturePath; }

protected:
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
	bool computeSegmentMeshIntersection(Coord Pa, Index indexA, Coord endPoint, bool& endPoint_inTriangle, 
		Index& endPointTriangle, sofa::helper::vector<Index>& edges_list, 
		sofa::helper::vector<double>& coordsEdge_list, const VecCoord& input_position);

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

protected:
	/// number of fracture done by the algorithm
	int m_fractureNumber;
	
	/// list of triangle where a T junction is blocking the algorithm
	sofa::helper::vector< sofa::helper::vector<int> > m_TjunctionTriangle;

	/// path created by algoFracturePath
	sofa::helper::vector<Coord> m_fracturePath;

	sofa::core::topology::BaseMeshTopology* m_topology;
	sofa::component::topology::TriangleSetTopologyModifier* m_modifier;
	sofa::component::topology::TriangleSetGeometryAlgorithms<DataTypes>* m_triangleGeo;
};

	
#if !defined(SOFA_TEARING_ALGORITHMS_CPP)
extern template class TEARING_API TearingAlgorithms<defaulttype::Vec3Types>;
#endif
		
}//namespace sofa::component