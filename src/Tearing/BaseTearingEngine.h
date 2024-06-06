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
class BaseTearingEngine : public core::DataEngine
{
public:
	SOFA_CLASS(SOFA_TEMPLATE(BaseTearingEngine, DataTypes), core::DataEngine);
	typedef typename DataTypes::Real Real;
	typedef typename DataTypes::Coord Coord;
	typedef typename DataTypes::VecCoord VecCoord;

	using Index = sofa::core::topology::BaseMeshTopology::Index;
	using VecIDs = type::vector<Index>;
	using Triangle = sofa::core::topology::BaseMeshTopology::Triangle;
	using TriangleID = sofa::core::topology::BaseMeshTopology::TriangleID;
	using VecTriangles = sofa::core::topology::BaseMeshTopology::SeqTriangles;
	
	using Vec3 = sofa::type::Vec3;
	using Transformation = type::Mat3x3;                 ///< matrix for rigid transformations like rotations
	using StrainDisplacement = type::Mat<6, 3, Real>;    ///< the strain-displacement matrix
	
	typedef typename sofa::component::solidmechanics::fem::elastic::TriangularFEMForceField<DataTypes>::TriangleInformation TriangleFEMInformation;
	typedef sofa::type::vector<TriangleFEMInformation> VecTriangleFEMInformation;

protected:
	BaseTearingEngine();
	~BaseTearingEngine() override {}
	
public:
	void init() override;
	void reinit() override;
    void doUpdate() final;
	virtual void draw(const core::visual::VisualParams* vparams) override;
    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;

	/// Input Data
	Data<VecCoord> d_input_positions; ///< Input position
    Data<sofa::helper::OptionsGroup> d_computeVertexStressMethod; ///< option to choose a method to compute the starting point for fracture

	Data<bool> d_ignoreTriangles; ///< option to ignore triangle at start
	Data<VecIDs> d_trianglesToIgnore; ///< list of triangles to ignore at start
	
	/// Fracture parameters
	Data<Real> d_stressThreshold; ///< threshold value for principal stress
	Data<Real> d_fractureMaxLength; ///< max length of a fracture
	Data<int> d_stepModulo; ///< to define a number of step between 2 fractures
	Data<int> d_nbFractureMax; ///< Maximum number of fracture

	/// Display parameters
	Data<bool> d_showTearableCandidates; ///< option to activate display of triangles candidates
	Data<bool> d_showFracturePath; ///< option to activate display of fracture path

	/// Output Data
	Data<VecIDs> d_triangleIdsOverThreshold; ///< output vector of triangles candidates from @sa triangleOverThresholdPrincipalStress
	Data<Real> d_maxStress; ///< output of the maximum stress found


	struct TriangleTearingInformation
	{
		//Real area;
		type::Vec<3, Real> stress;
		Real maxStress;
		Coord principalStressDirection;
	};

	/// Link to be set to the topology container in the component graph
	SingleLink<BaseTearingEngine<DataTypes>, sofa::core::topology::BaseMeshTopology, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_topology;

protected:
	/// <summary>
	/// put in d_triangleOverThresholdList triangle with a maxStress greater than a threshold value (d_seuilPrincipalStress)
	/// </summary>
	void triangleOverThresholdPrincipalStress();

	/// <summary>
	/// update d_triangleInfoTearing with value from d_triangleFEMInfo
	/// </summary>
	void updateTriangleInformation();

	/// <summary>
	/// compute fracture path intersection point and cut through them
	/// </summary>
	virtual void algoFracturePath()=0;
	
	void computeFractureDirection(Coord principleStressDirection, Coord& fracture_direction);

	/// <summary>
	/// compute extremities of fracture Pb and Pc from a start point Pa
	/// </summary>
	/// @param Pa - point with maxStress where fracture start
	/// @param direction - direction of maximum principal stress
	/// @return Pb - one of the extremities of fracture
	/// @return Pc - one of the extremities of fracture
	virtual void computeEndPoints(Coord Pa, Coord direction, Coord& Pb, Coord& Pc);

	/// <summary>
	/// compute ignored triangle at start of the tearing algo
	/// </summary>
	void computeTriangleToSkip();

    /// <summary>
    /// add T-junction triangles to the list of ignored triangles
    /// </summary>
	void processTjunctionTriangle(const vector<vector<int>>& TjunctionTriangle, helper::WriteAccessor<Data<vector<Index>>>& triangleToSkip);
	/// <summary>
	/// select the vertex with the maximum (area) weighted average of principal stress values
	/// </summary>
	/// @param selectedTriangle - The triangle with the maximum stress selected for the fracture start
	/// @return Index - The triangle index [0;2] where to start the fracture
	Index computeVertexByArea_WeightedAverage(const Triangle& selectedTriangle);

	/// <summary>
	/// select the vertex with the maximum unweighted average of principal stress values
	/// </summary>
	/// @param selectedTriangle - The triangle with the maximum stress selected for the fracture start
	/// @return Index - The triangle index [0;2] where to start the fracture
	Index computeVertexByUnweightedAverage(const Triangle& selectedTriangle);

	/// <summary>
	/// select the vertex with the maximum (distance) weighted average of principal stress values
	/// </summary>
	/// @param selectedTriangle - The triangle with the maximum stress selected for the fracture start
	/// @return Index - The triangle index [0;2] where to start the fracture
	Index computeVertexByInverseDistance_WeightedAverage(const Triangle& selectedTriangle);

	/// <summary>
	/// for a given vertex, compute the reciprocal of its distance with centroids of triangles
	/// around it
	/// </summary>
	void calculate_inverse_distance_weights(std::vector<double>& result, const Index vertex, sofa::type::vector<TriangleID>& ValidTrianglesAround);

	/// Access to the topology for a drived class
	sofa::core::topology::BaseMeshTopology* getTopology() const {
		return m_topology;
	}

	/// Access to the tearing algorithm for a drived class
	TearingAlgorithms<DataTypes>* getTearingAlgo() const
	{
		return m_tearingAlgo.get();
	}
	
	
	/// Pointer to the current topology
	sofa::core::topology::BaseMeshTopology* m_topology = nullptr;

	std::unique_ptr<TearingAlgorithms<DataTypes> > m_tearingAlgo = nullptr;

	TriangleID m_maxStressTriangleIndex = InvalidID; ///< Triangle ID of the triangle from filter candadites with the max stress
	Index m_maxStressVertexIndex = InvalidID; ///< Global Vertex Id where the stress is maximum. Vertex is part of @sa m_maxStressTriangleIndex Triangle
	sofa::component::solidmechanics::fem::elastic::TriangularFEMForceField<DataTypes>* m_triangularFEM = nullptr;
	sofa::component::solidmechanics::fem::elastic::TriangularFEMForceFieldOptim<DataTypes>* m_triangularFEMOptim = nullptr;

	vector<TriangleTearingInformation> m_triangleInfoTearing; ///< vector of TriangleInfo from FEM
	int m_stepCounter = 0; ///< counter of doUpdate called by the simulation. Used to put gap between consecutives fractures

};
		
}//namespace sofa::component::engine
