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

#ifdef TEARING_USES_MESHREFINEMENT
extern void registerVolumeTearingEngine(sofa::core::ObjectFactory* factory);
#endif

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
    TEARING_API void registerObjects(sofa::core::ObjectFactory* factory);
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
    return sofa_tostring(TEARING_VERSION);
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
#ifdef TEARING_USES_MESHREFINEMENT
    registerVolumeTearingEngine(factory);
#endif
}


} // namespace sofa::component
