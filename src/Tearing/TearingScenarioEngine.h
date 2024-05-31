#pragma once

#include <Tearing/BaseTearingEngine.h>
#include <Tearing/TearingAlgorithms.h>

#include <sofa/core/DataEngine.h>
#include <sofa/core/visual/VisualParams.h>

#include <sofa/component/solidmechanics/fem/elastic/TriangularFEMForceField.h>
#include <sofa/component/solidmechanics/fem/elastic/TriangularFEMForceFieldOptim.h>
#include <sofa/helper/OptionsGroup.h>


namespace sofa::component::engine
{
	using type::vector;

	template <class DataTypes>
	class TearingScenarioEngine : public BaseTearingEngine<DataTypes>
	{
	public:

		SOFA_CLASS(SOFA_TEMPLATE(TearingScenarioEngine, DataTypes), SOFA_TEMPLATE(BaseTearingEngine, DataTypes));
		typedef typename DataTypes::Real Real;
		typedef typename DataTypes::Coord Coord;
		using Vec3 = sofa::type::Vec3;
		using VecTriangles = sofa::core::topology::BaseMeshTopology::SeqTriangles;
		using Triangle = sofa::core::topology::BaseMeshTopology::Triangle;


		Data<int> d_startVertexId; ///< vertex ID to start algofracture (scenario case)
		Data<int> d_startTriId; ///< triangle ID to check junctions in algofracture
		Data<Vec3> d_startDirection; ///< direction to start algofracture (scenario case)
		Data<Real> d_startLength; ///< length of first fracture to start algofracture (scenario case)


	protected:
		TearingScenarioEngine();
		~TearingScenarioEngine() override {}

	public:
		void init() override;
		void reinit() override;
		
		void draw(const core::visual::VisualParams* vparams) override;
	    void handleEvent(sofa::core::objectmodel::Event* event) override;

	protected:
		
		void computePerpendicular(Coord dir, Coord& normal);
		void algoFracturePath() override;

	};
}//namespace sofa::component::engine


	

