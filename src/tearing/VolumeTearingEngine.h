#pragma once

#include <tearing/config.h>

#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
//#include <SofaSimpleFem/TetrahedronFEMForceField.h>
#include <SofaGeneralSimpleFem/TetrahedralCorotationalFEMForceField.h>
#include <SofaBoundaryCondition/ConstantForceField.h>

namespace sofa::helper
{
	class ColorMap;
}

namespace sofa::component::engine
{
	using core::DataEngine;
	using type::vector;

	template <class DataTypes>
	class VolumeTearingEngine : public core::DataEngine
	{
	public:
		SOFA_CLASS(SOFA_TEMPLATE(VolumeTearingEngine, DataTypes), core::DataEngine);
		typedef typename DataTypes::Real Real;
		typedef typename DataTypes::Coord Coord;
		typedef typename DataTypes::VecCoord VecCoord;

		typedef sofa::core::topology::BaseMeshTopology::Index Index;
		typedef sofa::core::topology::BaseMeshTopology::Tetrahedron Tetrahedron;
		typedef sofa::core::topology::BaseMeshTopology::SeqTetrahedra VecTetrahedra;
		typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
		typedef sofa::core::topology::BaseMeshTopology::SeqTriangles VecTriangles;
		typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
		typedef sofa::core::topology::BaseMeshTopology::SeqEdges VecEdges;
		typedef sofa::type::Vec<3, double> Vec3;

		typedef typename sofa::component::forcefield::TetrahedralCorotationalFEMForceField<DataTypes>::TetrahedronInformation TetrahedronFEMInformation;
		typedef vector<TetrahedronFEMInformation> VecTetrahedronFEMInformation;

	protected:
		VolumeTearingEngine();
		~VolumeTearingEngine() override {}

	public:
		void init() override;
		void reinit() override;
		void doUpdate() override;
		void draw(const core::visual::VisualParams* vparams) override;
		void handleEvent(sofa::core::objectmodel::Event* event) override;

		Data<VecCoord> input_position; ///< Input position

		/// Link to be set to the topology container in the component graph
		SingleLink<VolumeTearingEngine<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_topology;

		class TetrahedronInformation
		{
		public:
			Real volume;

			TetrahedronInformation() { }

			/// Output stream
			inline friend std::ostream& operator<< (std::ostream& os, const TetrahedronInformation& /*ti*/)
			{
				return os;
			}

			/// Input stream
			inline friend std::istream& operator>> (std::istream& in, TetrahedronInformation& /*ti*/)
			{
				return in;
			}
		};

		void updateTetrahedronInformation();
		Data<vector<TetrahedronInformation> > d_tetrahedronInfoTearing;
		Data<VecTetrahedronFEMInformation > d_tetrahedronFEMInfo;

		Data<double> d_seuilPrincipalStress;
		vector<Index> candidateVonMises;
		Data<double> d_seuilVonMises;
		Data<bool> showSeuil;
		Data<bool> showVonmises;

		/// put in d_tetraOverThresholdList tetrahedron with a maxStress greater than a threshold value (d_seuilPrincipalStress)
		void computeTetraOverThresholdPrincipalStress();
		Data<vector<Index>> d_tetraToIgnoreList;
		Data<vector<Index>> d_tetraOverThresholdList;
		Data<bool> showTetraOverThreshold;
		Real maxStress;
		Index indexTetraMaxStress;

		Data<int> d_counter;
		Data<bool> stepByStep;
		Data<int> d_step;
		Data<int> d_fractureNumber;
		Data<int> d_nbFractureMax;

		/// compute barycenter of indexMaxTetra and compute the cutting plane
		void computePlane(Coord& vec_P1M, Coord& vec_P2M);

		Data<bool> ignoreTetraAtStart;
		bool drawIgnoredTetraAtStart;
		void computeTetraToSkip();

	protected:
		/// Pointer to the current topology
		sofa::core::topology::BaseMeshTopology* m_topology;
		sofa::component::forcefield::TetrahedralCorotationalFEMForceField<DataTypes>* m_tetraFEM;
	
	private:
		sofa::helper::ColorMap* p_drawColorMap;

	};

#if !defined(SOFA_COMPONENT_ENGINE_TEARINGENGINE_CPP)
	extern template class TEARING_API VolumeTearingEngine<defaulttype::Vec3Types>;
#endif

}//namespace sofa::component::engine

