cmake_minimum_required(VERSION 3.16.3...3.19.7 FATAL_ERROR)

project(URDF_Importer)

#-----------------------------------------------------------------------------
# Extension meta-information
set(EXTENSION_HOMEPAGE "https://www.slicer.org/wiki/Documentation/Nightly/Extensions/URDF_Importer")
set(EXTENSION_CONTRIBUTORS "Anna Delale-O'Connor")
set(EXTENSION_DESCRIPTION "This is an example of a simple extension")
set(EXTENSION_ICONURL "https://www.example.com/Slicer/Extensions/URDF_Importer.png")
set(EXTENSION_SCREENSHOTURLS "https://www.example.com/Slicer/Extensions/URDF_Importer/Screenshots/1.png")
set(EXTENSION_DEPENDS "NA") # Specified as a list or "NA" if no dependencies

#-----------------------------------------------------------------------------
# Extension dependencies
find_package(Slicer REQUIRED)
include(${Slicer_USE_FILE})

#-----------------------------------------------------------------------------
# Extension modules
add_subdirectory(URDF_Import)
## NEXT_MODULE

#-----------------------------------------------------------------------------
include(${Slicer_EXTENSION_GENERATE_CONFIG})
include(${Slicer_EXTENSION_CPACK})
