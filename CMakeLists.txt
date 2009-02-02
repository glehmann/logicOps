#Change PROJECT_NAME to the name of your project
PROJECT(logicOps)

# required to avoid warnings while running cmake
CMAKE_MINIMUM_REQUIRED(VERSION 2.4.2)

# cpack stuff
INCLUDE(CPack)

# set the name of the input image used to run the test
SET(INPUT_IMAGE ${CMAKE_SOURCE_DIR}/images/cthead1.png)
SET(BINARY_INPUT_IMAGE ${CMAKE_SOURCE_DIR}/images/cthead1.png)

#include some macros from another file...
INCLUDE(IJMacros.txt)

#The following lines are required to use Dart
ENABLE_TESTING()
INCLUDE(Dart)
FIND_PROGRAM(IMAGE_COMPARE ImageCompare)

#Declare any external dependencies that your project may have here.
#examples include: ITK, VTK, JPEG, PNG, OpenGL, ZLIB, Perl, Java
#If you're not sure what name to use, look in the Modules directory of your
#cmake install and check that a file named Find(Package).cmake exists
SET(Required_Packages
  ITK
)

#this foreach loads all of the packages that you specified as required.
#It shouldn't need to be modified.
FOREACH(Package ${Required_Packages})
  LOADPACKAGE(${Package})
ENDFOREACH(Package)

#Set any libraries that your project depends on.
#examples: ITKCommon, VTKRendering, etc
SET(Libraries
  ITKCommon
  ITKBasicFilters
  ITKIO
)


# install devel files
OPTION(INSTALL_DEVEL_FILES "Install C++ headers" ON)
IF(INSTALL_DEVEL_FILES)
FILE(GLOB develFiles *.h *.txx) 
FOREACH(f ${develFiles})
  INSTALL_FILES(/include/InsightToolkit/BasicFilters FILES ${f})
ENDFOREACH(f)
ENDIF(INSTALL_DEVEL_FILES)



# option for wrapping
OPTION(BUILD_WRAPPERS "Wrap library" OFF)
IF(BUILD_WRAPPERS)
   SUBDIRS(Wrapping)
ENDIF(BUILD_WRAPPERS)
   
   

#the following block of code is an example of how to build an executable in
#cmake.  Unmodified, it will add an executable called "MyExe" to the project.
#MyExe will be built using the files MyClass.h and MyClass.cxx, and it will
#be linked to all the libraries you specified above. 
#You can build more than one executable per project

IF(BUILD_TESTING)

SET(CurrentExe "check")
ADD_EXECUTABLE(${CurrentExe} ${CurrentExe}.cxx)
TARGET_LINK_LIBRARIES(${CurrentExe} ${Libraries})

SET(CurrentExe "binary_check")
ADD_EXECUTABLE(${CurrentExe} ${CurrentExe}.cxx)
TARGET_LINK_LIBRARIES(${CurrentExe} ${Libraries})

ENDIF(BUILD_TESTING)

#the following line is an example of how to add a test to your project.
#Testname is the title for this particular test.  ExecutableToRun is the
#program which will be running this test.  It can either be a part of this
#project or an external executable.  After that list any args that are needed
#for this test.  Include as many tests as you like.  If your project doesn't have
#any tests you can comment out or delete the following line.
# ADD_TEST(Testname ExecutableToRun arg1 arg2 arg3)

ADD_TEST(Run check ${INPUT_IMAGE} eq.png gt.png gte.png lt.png lte.png smooth.png ne.png)
ADD_TEST(CompareImageEq ${IMAGE_COMPARE} eq.png ${CMAKE_SOURCE_DIR}/images/eq.png)
ADD_TEST(CompareImageGT ${IMAGE_COMPARE} gt.png ${CMAKE_SOURCE_DIR}/images/gt.png)
ADD_TEST(CompareImageGTE ${IMAGE_COMPARE} gte.png ${CMAKE_SOURCE_DIR}/images/gte.png)
ADD_TEST(CompareImageLT ${IMAGE_COMPARE} lt.png ${CMAKE_SOURCE_DIR}/images/lt.png)
ADD_TEST(CompareImageLTE ${IMAGE_COMPARE} lte.png ${CMAKE_SOURCE_DIR}/images/lte.png)
ADD_TEST(CompareImageNE ${IMAGE_COMPARE} ne.png ${CMAKE_SOURCE_DIR}/images/ne.png)

ADD_TEST(BinaryRun binary_check ${BINARY_INPUT_IMAGE} ${CMAKE_SOURCE_DIR}/images/ne.png and.png or.png xor.png not.png 255 0)
ADD_TEST(CompareBinaryAnd ${IMAGE_COMPARE} and.png ${CMAKE_SOURCE_DIR}/images/and.png)
ADD_TEST(CompareBinaryOr ${IMAGE_COMPARE} or.png ${CMAKE_SOURCE_DIR}/images/or.png)
ADD_TEST(CompareBinaryXor ${IMAGE_COMPARE} xor.png ${CMAKE_SOURCE_DIR}/images/xor.png)
ADD_TEST(CompareBinaryNot ${IMAGE_COMPARE} not.png ${CMAKE_SOURCE_DIR}/images/not.png)
