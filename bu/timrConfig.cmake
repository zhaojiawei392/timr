

include(CMakeFindDependencyMacro)

find_dependency(qpOASES REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/timrTargets.cmake")