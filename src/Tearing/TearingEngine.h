#pragma once

#include <Tearing/config.h>
#include <Tearing/TearingAlgorithms.h>

#include <sofa/core/DataEngine.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/component/solidmechanics/fem/elastic/TriangularFEMForceField.h>
#include <sofa/component/solidmechanics/fem/elastic/TriangularFEMForceFieldOptim.h>


namespace sofa::helper
{
	class ColorMap;
}

namespace sofa::component::engine
{

	using core::DataEngine;
	using type::vector;

template <class DataTypes>
class TearingEngine : public core::DataEngine
{
public:
	SOFA_CLASS(SOFA_TEMPLATE(TearingEngine, DataTypes), core::DataEngine);
	typedef typename DataTypes::Real Real;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::VecCoord VecCoord;

	using Index = sofa::core::topology::BaseMeshTopology::Index;
	using Triangle = sofa::core::topology::BaseMeshTopology::Triangle;
	using VecTriangles = sofa::core::topology::BaseMeshTopology::SeqTriangles;
	
	using Vec3 = sofa::type::Vec3;
	using Transformation = type::Mat3x3;                 ///< matrix for rigid transformations like rotations
	using StrainDisplacement = type::Mat<6, 3, Real>;    ///< the strain-displacement matrix
	
	typedef typename sofa::component::solidmechanics::fem::elastic::TriangularFEMForceField<DataTypes>::TriangleInformation TriangleFEMInformation;
	typedef sofa::type::vector<TriangleFEMInformation> VecTriangleFEMInformation;

protected:
	TearingEngine();
	~TearingEngine() override {}
	
public:
	void init() override;
	void reinit() override;
	void doUpdate() override;
	void draw(const core::visual::VisualParams* vparams) override;

	Data<VecCoord> input_position; ///< Input position
	Data<bool> showChangedTriangle;

	//Looking for triangle will tear first
	Data<Real> d_seuilPrincipalStress; ///< threshold value for principal stress
	Data<vector<Index>> d_triangleOverThresholdList;
	Data<Real> d_maxStress;
	Data<Index> d_indexTriangleMaxStress;
	Data<Index> d_indexVertexMaxStress;
	Data<bool> showTearableTriangle;
	Data<bool> stepByStep;
	Data<int> d_step;
	int m_counter;
	Data<Real> d_fractureMaxLength;
	Data<vector<Index>> d_triangleToIgnoreList;
	Data<bool> ignoreTriangleAtStart;

	/// <summary>
	/// put in d_triangleOverThresholdList triangle with a maxStress greater than a threshold value (d_seuilPrincipalStress)
	/// </summary>
	void triangleOverThresholdPrincipalStress();
	
	struct TriangleTearingInformation
	{
		//Real area;
		type::Vec<3, Real> stress;
		Real maxStress;
		Coord principalStressDirection;
	};
	vector<TriangleTearingInformation> m_triangleInfoTearing;

	/// <summary>
	/// update d_triangleInfoTearing with value from d_triangleFEMInfo
	/// </summary>
	void updateTriangleInformation();

	/// Test algoFracturePath
	Data<int> d_nbFractureMax;

	/// <summary>
	/// compute fracture path intersection point and cut through them
	/// </summary>
	void algoFracturePath();

	Data<bool> showFracturePath;

	/// <summary>
	/// compute extremities of fracture Pb and Pc from a start point Pa
	/// </summary>
	/// @param Pa - point with maxStress where fracture start
	/// @param direction - direction of maximum principal stress
	/// @return Pb - one of the extremities of fracture
	/// @return Pc - one of the extremities of fracture
	void computeEndPoints(Coord Pa, Coord direction, Coord& Pb, Coord& Pc);
	
	/// Link to be set to the topology container in the component graph
	SingleLink<TearingEngine<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_topology;

	Data<int> d_startVertexId;
	Data <Vec3> d_startDirection;
	Data <Real> d_startLength;

	/// <summary>
	/// compute ignored triangle at start of the tearing algo
	/// </summary>
	void computeTriangleToSkip();

protected:
	/// Pointer to the current topology
	sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

	sofa::component::solidmechanics::fem::elastic::TriangularFEMForceField<DataTypes>* m_triangularFEM;
	sofa::component::solidmechanics::fem::elastic::TriangularFEMForceFieldOptim<DataTypes>* m_triangularFEMOptim;
	
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
