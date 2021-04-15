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

	void initComputeArea();
	void computeArea();

	//Data
	Data<vector<double> > d_initArea;
	
	Data<VecCoord> input_position; ///< Input position
	Data<bool> showChangedTriangle;

	//Looking for triangle will tear first
	Data<double> d_seuilArea; ///<  threshold value for area
	Data<double> d_seuilPrincipalStress; ///< threshold value for principal stress
	Data<vector<Index>> d_triangleOverThresholdList;
	Data<double> d_maxStress;
	Data<Index> d_indexTriangleMaxStress;
	Data<Index> d_indexVertexMaxStress;
	Data<bool> showTearableTriangle;
	Data<bool> stepByStep;
	Data<int> d_counter;
	Data<double> d_fractureMaxLength;

	void triangleOverThresholdArea();
	void triangleOverThresholdPrincipalStress();
	
	class TriangleInformation
	{
	public:
		Real area;

		//StrainDisplacement strainDisplacementMatrix;
		//Transformation rotation;
		//defaulttype::Vec<3, Real> strain;
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

	void doFracture();
	Data<vector<Index> > d_fractureIndices;
	Data<double> d_fractureBaryCoef;
	Data<double> d_fractureCoord_kmin;
	Data<bool> d_fractureBool;

	/// Test intersectionFractureEdge
	void intersectionFractureEdge();
	Data<bool> d_intersectionFractureEdgeBool;
	Data<double> d_intersectionFractureEdgeBaryCoef;

	/// Test algoFracturePath
	void algoFracturePath();
	Data<vector<Coord>> d_fracturePath;

	/// Link to be set to the topology container in the component graph
	SingleLink<TearingEngine<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_topology;

protected:
	/// Pointer to the current topology
	sofa::core::topology::BaseMeshTopology* m_topology;
	sofa::component::topology::TriangleSetGeometryAlgorithms<DataTypes>* m_triangleGeo;
	sofa::component::forcefield::TriangularFEMForceField<DataTypes>* m_triangularFEM;

private:
	sofa::helper::ColorMap* p_drawColorMap;

};
	
#if !defined(SOFA_COMPONENT_ENGINE_TEARINGENGINE_CPP)
extern template class TEARING_API TearingEngine<defaulttype::Vec3Types>;
#endif
		
}//namespace sofa::component::engine