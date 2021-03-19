#pragma once

#include <tearing/config.h>

#include <sofa/core/DataEngine.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/topology/BaseMeshTopology.h>

#include <SofaBaseTopology/TopologyData.h>

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

	typedef sofa::core::topology::BaseMeshTopology::Index Index;
	typedef sofa::core::topology::BaseMeshTopology::Triangle Element;
	typedef sofa::core::topology::BaseMeshTopology::SeqTriangles VecElement;


protected:
	TearingEngine();
	~TearingEngine() override {}
	typedef defaulttype::Mat<3, 3, Real > Transformation;				    ///< matrix for rigid transformations like rotations

public:
	void init() override;
	void reinit() override;
	void doUpdate() override;
	void draw(const core::visual::VisualParams* vparams) override;

	//Data
	Data<vector<Element> > d_triangles; ///< Triangle Topology
	Data<vector<double> > d_area;

	class TriangleInformation
	{
	public:
		Real area;
		TriangleInformation() { }
		//Transformation rotation;

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

	class VertexInformation
	{
	public:
		VertexInformation()
			:sumEigenValues(0.0), stress(0.0) {}

		Coord meanStrainDirection;
		double sumEigenValues;
		Transformation rotation;

		double stress; //average stress of triangles around (used only for drawing)

		/// Output stream
		inline friend std::ostream& operator<< (std::ostream& os, const VertexInformation& /*vi*/)
		{
			return os;
		}
		/// Input stream
		inline friend std::istream& operator>> (std::istream& in, VertexInformation& /*vi*/)
		{
			return in;
		}
	};


	topology::TriangleData<sofa::helper::vector<TriangleInformation> > triangleInfo;
	topology::PointData<sofa::helper::vector<VertexInformation> > vertexInfo; ///< Internal point data
	Data<VecCoord> input_position; ///< Input position
	Data<bool> showChangedTriangle;

	/// Link to be set to the topology container in the component graph
	SingleLink<TearingEngine<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_topology;

protected:
	/// Pointer to the current topology
	sofa::core::topology::BaseMeshTopology* m_topology;
};
	
#if !defined(SOFA_COMPONENT_ENGINE_TEARINGENGINE_CPP)
extern template class TEARING_API TearingEngine<defaulttype::Vec3Types>;
#endif
		
}//namespace sofa::component::engine