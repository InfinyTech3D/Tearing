/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <tearing/config.h>

namespace sofa
{
namespace component
{

extern "C" {

TEARING_API
void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

TEARING_API
const char* getModuleName()
{
    return "tearing";
}

TEARING_API
const char* getModuleVersion()
{
    return "1.0";
}

TEARING_API
const char* getModuleLicense()
{
    return "LGPL";
}

TEARING_API
const char* getModuleDescription()
{
    return "tearing plugin crée par Bun pour son stage de fin d'étude";
}

TEARING_API
const char* getModuleComponentList()
{
    // string containing the names of the classes provided by the plugin
    return "ModuleComponent1";
}   
} // extern "C"

} // namespace component
} // namespace sofa
