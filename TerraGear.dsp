# Microsoft Developer Studio Project File - Name="TerraGear" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=TerraGear - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "TerraGear.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "TerraGear.mak" CFG="TerraGear - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "TerraGear - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "TerraGear - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD CPP /nologo /W3 /GX /O2 /D "NDEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /FD /c 
# SUBTRACT CPP /YX
# ADD RSC /l 0xc09 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BSC32 /nologo
LINK32=link.exe
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib uuid.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386 

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /D "_DEBUG" /D "WIN32" /D "_CONSOLE" /D "_MBCS" /FR /FD /GZ /c 
# ADD RSC /l 0xc09 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BSC32 /nologo
LINK32=link.exe
# ADD LINK32 kernel32.lib user32.lib winspool.lib comdlg32.lib gdi32.lib shell32.lib wsock32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept 

!ENDIF 

# Begin Target

# Name "TerraGear - Win32 Release"
# Name "TerraGear - Win32 Debug"
# Begin Group "Lib_Clipper"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\BuildTiles\Clipper\clipper.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Clipper"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Clipper"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Clipper\clipper.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Clipper"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Clipper"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_GenOutput"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\BuildTiles\GenOutput\genobj.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_GenOutput"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_GenOutput"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\GenOutput\genobj.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_GenOutput"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_GenOutput"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_Match"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\BuildTiles\Match\match.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Match"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Match"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Match\match.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Match"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Match"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_Triangulate"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\BuildTiles\Triangulate\triangle.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Triangulate"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Triangulate"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Triangulate\triangle.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Triangulate"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Triangulate"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Triangulate\trieles.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Triangulate"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Triangulate"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Triangulate\trieles.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Triangulate"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Triangulate"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_Osgb36"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\BuildTiles\Osgb36\osgb36.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Osgb36"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Osgb36"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Osgb36\osgb36.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Osgb36"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Osgb36"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Osgb36\osgbtc.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Osgb36"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Osgb36"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Osgb36\osgbtc.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Osgb36"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Osgb36"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Osgb36\uk.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Osgb36"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Osgb36"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\BuildTiles\Osgb36\uk.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Osgb36"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Osgb36"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_Array"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\Array\array.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Array"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Array"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Array\array.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Array"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Array"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_DEM"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\DEM\dem.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_DEM"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_DEM"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\DEM\dem.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_DEM"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_DEM"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_e00"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\e00\e00.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_e00"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_e00"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\e00\e00.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_e00"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_e00"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_Geometry"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\Geometry\contour_tree.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\contour_tree.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\line.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\line.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\poly_support.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\poly_support.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\rectangle.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\rectangle.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\trinodes.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\trinodes.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\trisegs.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\trisegs.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\util.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Geometry\util.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Geometry"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Geometry"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_landcover"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\landcover\landcover.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_landcover"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_landcover"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\landcover\landcover.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_landcover"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_landcover"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_Optimize"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\Optimize\genfans.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Optimize"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Optimize"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Optimize\genfans.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Optimize"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Optimize"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Optimize\genstrips.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Optimize"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Optimize"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Optimize\genstrips.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Optimize"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Optimize"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_Output"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\Output\output.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Output"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Output"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Output\output.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Output"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Output"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_Polygon"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\Polygon\index.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\index.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\names.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\names.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\polygon.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\polygon.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\simple_clip.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\simple_clip.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\split-bin.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\split.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\superpoly.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\Polygon\superpoly.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_Polygon"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_Polygon"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_poly2tri"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\poly2tri\construct.c

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_poly2tri"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_poly2tri"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\poly2tri\interface.h

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_poly2tri"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_poly2tri"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\poly2tri\misc.c

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_poly2tri"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_poly2tri"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\poly2tri\monotone.c

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_poly2tri"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_poly2tri"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\poly2tri\tri.c

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_poly2tri"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_poly2tri"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\poly2tri\triangulate.h

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_poly2tri"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_poly2tri"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_shape"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\shapelib\dbfopen.c

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_shape"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_shape"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\shapelib\shpopen.c

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_shape"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_shape"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\shapelib\shapefil.h

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_shape"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_shape"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_TriangleJRS"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\TriangleJRS\triangle.c

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_TriangleJRS"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_TriangleJRS"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\TriangleJRS\triangle.h

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_TriangleJRS"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_TriangleJRS"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\TriangleJRS\tri_support.c

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_TriangleJRS"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_TriangleJRS"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\TriangleJRS\tri_support.h

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_TriangleJRS"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_TriangleJRS"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_vpf"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Lib\vpf\component.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\coverage.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\database.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\feature.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\label.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\library.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\contour.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\line.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\polygon.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\property.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\table.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\tablemgr.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\tile.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\value.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\vpf.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\vpfbase.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\vpfbase.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\value.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\table.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\tablemgr.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\component.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\database.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\library.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\coverage.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\feature.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\line.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\contour.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\polygon.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\label.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\property.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\tile.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\vpf-summary.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\vpf-dump.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Lib\vpf\vpf-topology.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_vpf"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_vpf"

!ENDIF 

# End Source File
# End Group
# Begin Group "Lib_MergerClipper"

# PROP Default_Filter ""
# Begin Source File

SOURCE=.\src\Prep\MergerClipper\merger.cxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_MergerClipper"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_MergerClipper"

!ENDIF 

# End Source File
# Begin Source File

SOURCE=.\src\Prep\MergerClipper\merger.hxx

!IF  "$(CFG)" == "TerraGear - Win32 Release"

# PROP Intermediate_Dir "Release\Lib_MergerClipper"

!ELSEIF  "$(CFG)" == "TerraGear - Win32 Debug"

# PROP Intermediate_Dir "Debug\Lib_MergerClipper"

!ENDIF 

# End Source File
# End Group
# End Target
# End Project
