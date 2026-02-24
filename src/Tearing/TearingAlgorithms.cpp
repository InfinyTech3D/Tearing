/*****************************************************************************
 *                - Copyright (C) 2020-Present InfinyTech3D -                *
 *                                                                           *
 * This file is part of the Tearing plugin for the SOFA framework.           *
 *                                                                           *
 * This file is dual-licensed:                                               *
 *                                                                           *
 * 1) Commercial License:                                                    *
 *      This file may be used under the terms of a valid commercial license  *
 *      agreement provided wih the software by InfinyTech3D.                 *
 *                                                                           *
 * 2) GNU General Public License (GPLv3) Usage                               *
 *      Alternatively, this file may be used under the terms of the          *
 *      GNU General Public License version 3 as published by the             *
 *      Free Software Foundation: https://www.gnu.org/licenses/gpl-3.0.html  *
 *                                                                           *
 * Contact: contact@infinytech3d.com                                         *
 * Further information: https://infinytech3d.com                             *
 ****************************************************************************/
#define SOFA_TEARING_ALGORITHMS_CPP
#include <Tearing/TearingAlgorithms.inl>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa::component
{
using namespace sofa::defaulttype;

template class TEARING_API TearingAlgorithms<Vec3Types>;

}//namespace sofa::component