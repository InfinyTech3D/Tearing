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
#include <Tearing/initTearing.h>

namespace sofa::component
{

extern "C" {
    TEARING_API void initExternalModule();
    TEARING_API const char* getModuleName();
    TEARING_API const char* getModuleVersion();
    TEARING_API const char* getModuleLicense();
    TEARING_API const char* getModuleDescription();
    TEARING_API const char* getModuleComponentList();
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

const char* getModuleName()
{
    return "tearing";
}

const char* getModuleVersion()
{
    return "1.0";
}

const char* getModuleLicense()
{
    return "GPL";
}

const char* getModuleDescription()
{
    return "tearing plugin";
}

const char* getModuleComponentList()
{
    // string containing the names of the classes provided by the plugin
    return "TearingEngine, TearingAlgorithms, VolumeTearingEngine, VolumeTearingAlgorithms";
}   


void init()
{
    initExternalModule();
}

} // namespace sofa::component
