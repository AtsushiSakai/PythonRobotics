# Microsoft Developer Studio Project File - Name="RAPID" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 5.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Static Library" 0x0104

CFG=RAPID - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "RAPID.MAK".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "RAPID.MAK" CFG="RAPID - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "RAPID - Win32 Release" (based on "Win32 (x86) Static Library")
!MESSAGE "RAPID - Win32 Debug" (based on "Win32 (x86) Static Library")
!MESSAGE 

# Begin Project
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe

!IF  "$(CFG)" == "RAPID - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\"
# PROP Intermediate_Dir "Release"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /O2 /I "..\\" /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /YX /FD /TP /c
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ELSEIF  "$(CFG)" == "RAPID - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\"
# PROP Intermediate_Dir "Debug"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /Z7 /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /YX /FD /c
# ADD CPP /nologo /W3 /GX /Z7 /Od /I "..\\" /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /YX /FD /TP /c
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo

!ENDIF 

# Begin Target

# Name "RAPID - Win32 Release"
# Name "RAPID - Win32 Debug"
# Begin Source File

SOURCE=..\build.C
# End Source File
# Begin Source File

SOURCE=..\collide.C
# End Source File
# Begin Source File

SOURCE=..\matvec.H
# End Source File
# Begin Source File

SOURCE=..\moments.H
# End Source File
# Begin Source File

SOURCE=..\obb.H
# End Source File
# Begin Source File

SOURCE=..\overlap.C
# End Source File
# Begin Source File

SOURCE=..\overlap.H
# End Source File
# Begin Source File

SOURCE=..\RAPID.C
# End Source File
# Begin Source File

SOURCE=..\RAPID.H
# End Source File
# Begin Source File

SOURCE=..\RAPID_private.H
# End Source File
# Begin Source File

SOURCE=..\RAPID_version.H
# End Source File
# End Target
# End Project
