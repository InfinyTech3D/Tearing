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
    typedef typename DataTypes::VecCoord VecCoord;
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

    using BaseTearingEngine<DataTypes>::addInput;

public:
    void init() override;
	void draw(const core::visual::VisualParams* vparams) override;

protected:
		
	void computePerpendicular(Coord dir, Coord& normal);
	void algoFracturePath() override;
	void computeEndPoints(Coord Pa, Coord direction, Coord& Pb, Coord& Pc) override;

	/// Value to store scenario fracture path
	Coord m_Pa, m_Pb, m_Pc;

	bool m_fractureDone = false;
};


#if !defined(SOFA_COMPONENT_ENGINE_TEARINGSCENARIOENGINE_CPP)
extern template class TEARING_API TearingScenarioEngine<defaulttype::Vec3Types>;
#endif

} // namespace sofa::component::engine
