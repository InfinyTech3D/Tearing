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

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>

using sofa::core::ObjectFactory;

namespace sofa::component
{
extern void registerTriangleCuttingController(sofa::core::ObjectFactory* factory);
}

namespace sofa::component::engine
{
extern void registerTearingEngine(sofa::core::ObjectFactory* factory);
extern void registerTearingScenarioEngine(sofa::core::ObjectFactory* factory);
extern void registerVolumeTearingEngine(sofa::core::ObjectFactory* factory);
}



namespace sofa::component
{

using namespace sofa::component::engine;

extern "C" {
    TEARING_API void initExternalModule();
    TEARING_API const char* getModuleName();
    TEARING_API const char* getModuleVersion();
    TEARING_API const char* getModuleLicense();
    TEARING_API const char* getModuleDescription();
    TEARING_API const char* getModuleComponentList();
}

void initTearing()
{
    static bool first = true;
    if (first)
    {
        // make sure that this plugin is registered into the PluginManager
        sofa::helper::system::PluginManager::getInstance().registerPlugin(sofa_tostring(SOFA_TARGET));

        first = false;
    }
}

void initExternalModule()
{
    initTearing();
}



const char* getModuleName()
{
    return sofa_tostring(SOFA_TARGET);
}

const char* getModuleVersion()
{
    return sofa_tostring(PLUGIN_VERSION);
}

const char* getModuleLicense()
{
    return "GPL";
}

const char* getModuleDescription()
{
    return "SOFA tearing plugin";
}

const char* getModuleComponentList()
{
    /// string containing the names of the classes provided by the plugin
    static std::string classes = ObjectFactory::getInstance()->listClassesFromTarget(sofa_tostring(SOFA_TARGET));
    return classes.c_str();
}

void registerObjects(sofa::core::ObjectFactory* factory)
{
    registerTriangleCuttingController(factory);
    registerTearingEngine(factory);
    registerTearingScenarioEngine(factory);
    registerVolumeTearingEngine(factory);
}


} // namespace sofa::component
