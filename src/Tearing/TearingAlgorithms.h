/*****************************************************************************
 *                 - Copyright (C) - 2020 - InfinyTech3D -                   *
 *                                                                           *
 * This file is part of the Tearing plugin for the SOFA framework.           *
 *                                                                           *
 * Commercial License Usage:                                                 *
 * Licensees holding valid commercial license from InfinyTech3D may use this *
 * file in accordance with the commercial license agreement provided with    *
 * the Software or, alternatively, in accordance with the terms contained in *
 * a written agreement between you and InfinyTech3D. For further information *
 * on the licensing terms and conditions, contact: contact@infinytech3d.com  *
 *                                                                           *
 * GNU General Public License Usage:                                         *
 * Alternatively, this file may be used under the terms of the GNU General   *
 * Public License version 3. The licenses are as published by the Free       *
 * Software Foundation and appearing in the file LICENSE.GPL3 included in    *
 * the packaging of this file. Please review the following information to    *
 * ensure the GNU General Public License requirements will be met:           *
 * https://www.gnu.org/licenses/gpl-3.0.html.                                *
 *                                                                           *
 * Authors: see Authors.txt                                                  *
 * Further information: https://infinytech3d.com                             *
 ****************************************************************************/
#pragma once

#include <Tearing/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/component/topology/container/dynamic/TriangleSetTopologyContainer.h>
#include <sofa/component/topology/container/dynamic/TriangleSetTopologyModifier.h>
#include <sofa/component/topology/container/dynamic/TriangleSetGeometryAlgorithms.h>

namespace sofa::component
{
using sofa::component::topology::container::dynamic::TriangleSetTopologyModifier;
using sofa::component::topology::container::dynamic::TriangleSetGeometryAlgorithms;

template <class DataTypes>
class TearingAlgorithms
{
public:
	typedef typename DataTypes::Real Real;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::VecCoord VecCoord;

	using Index = sofa::core::topology::BaseMeshTopology::Index;
	using Edge = sofa::core::topology::BaseMeshTopology::Edge;
	using Triangle = sofa::core::topology::BaseMeshTopology::Triangle;
	using VecIds = sofa::type::vector<Index>;

	TearingAlgorithms(sofa::core::topology::BaseMeshTopology* _topology,
		TriangleSetTopologyModifier* _modifier,
		TriangleSetGeometryAlgorithms<DataTypes>* _triangleGeo);

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

	const sofa::type::vector< sofa::type::vector<int> >& getTjunctionTriangles() const { return m_TjunctionTriangle; }
	const sofa::type::vector< sofa::type::vector<int> >& getTjunctionVertices() const { return m_TjunctionVertex; }

	const sofa::type::vector<Coord>& getFracturePath() const { return m_fracturePath; }

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
		Index& endPointTriangle, VecIds& edges_list, 
		sofa::type::vector<double>& coordsEdge_list, const VecCoord& input_position);

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
		bool pointB_inTriangle, Index triangleB, Coord Pb, VecIds edges_listB, sofa::type::vector<double> coordsEdge_listB, int& sizeB,
		Coord Pa, Index indexA,
		bool pointC_inTriangle, Index triangleC, Coord Pc, VecIds edges_listC, sofa::type::vector<double> coordsEdge_listC, int& sizeC,
        sofa::type::vector< sofa::geometry::ElementType>& topoPath_list,
		VecIds& indices_list,
		sofa::type::vector< sofa::type::Vec<3, double> >& coords_list);

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
        sofa::type::vector< sofa::geometry::ElementType> topoPath_list,
		VecIds indices_list,
		sofa::type::vector< sofa::type::Vec<3, double> > coords_list,
		sofa::type::vector< Index >& new_edges);

private:
    /// Pointers to the topology components
    sofa::core::topology::BaseMeshTopology* m_topology;
    TriangleSetTopologyModifier* m_modifier;
    TriangleSetGeometryAlgorithms<DataTypes>* m_triangleGeo;

	/// number of fracture done by the algorithm
	int m_fractureNumber;
	
	/// list of triangle where a T junction is blocking the algorithm
	sofa::type::vector< sofa::type::vector<int> > m_TjunctionTriangle;
	/// <summary>
	/// List of vertices at which a T-junction has heppened
	/// </summary>
	sofa::type::vector< sofa::type::vector<int> > m_TjunctionVertex;

	/// path created by algoFracturePath
	sofa::type::vector<Coord> m_fracturePath;
};

	
#if !defined(SOFA_TEARING_ALGORITHMS_CPP)
extern template class TEARING_API TearingAlgorithms<defaulttype::Vec3Types>;
#endif
		
}//namespace sofa::component
