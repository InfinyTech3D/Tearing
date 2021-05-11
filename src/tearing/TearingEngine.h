#pragma once

#include <tearing/config.h>

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

	void updateTriangleInformation();

	/// Test algoFracturePath
	void algoFracturePath();
	Data<vector<Coord>> d_fracturePath;
	Data<bool> showFracturePath;
	void computeEndPoints(Coord Pa, Coord direction, Coord& Pb, Coord& Pc);
	bool computeSegmentMeshIntersection(Coord Pa,Index indexA, Coord endPoint, bool& endPoint_inTriangle, Index& endPointTriangle, sofa::helper::vector<Index>& edges_list, sofa::helper::vector<double>& coordsEdge_list);
	void pathAdaptationObject(
		double EPS,
		bool pointB_inTriangle, Index triangleB, Coord Pb, sofa::helper::vector<Index> edges_listB, sofa::helper::vector<double> coordsEdge_listB, int& sizeB,
		Coord Pa, Index indexA,
		bool pointC_inTriangle, Index triangleC, Coord Pc, sofa::helper::vector<Index> edges_listC, sofa::helper::vector<double> coordsEdge_listC, int& sizeC,
		sofa::helper::vector< sofa::core::topology::TopologyElementType>& topoPath_list,
		sofa::helper::vector<Index>& indices_list,
		sofa::helper::vector< sofa::defaulttype::Vec<3, double> >& coords_list);
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

protected:
	/// Pointer to the current topology
	sofa::core::topology::BaseMeshTopology* m_topology;
	sofa::component::topology::TriangleSetGeometryAlgorithms<DataTypes>* m_triangleGeo;
	sofa::component::forcefield::TriangularFEMForceField<DataTypes>* m_triangularFEM;
	sofa::component::topology::TriangleSetTopologyModifier* m_modifier;
private:
	sofa::helper::ColorMap* p_drawColorMap;


public:
	void handleEvent(sofa::core::objectmodel::Event* event) override;

};
	
#if !defined(SOFA_COMPONENT_ENGINE_TEARINGENGINE_CPP)
extern template class TEARING_API TearingEngine<defaulttype::Vec3Types>;
#endif
		
}//namespace sofa::component::engine