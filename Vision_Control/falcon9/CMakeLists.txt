find_package (Qt4 REQUIRED)

set  (project_SOURCES main.cpp 
	pclviewer.cpp
	AbsoluteObjectKF.cpp
)

set  (project_HEADERS pclviewer.h)
set  (project_FORMS   pclviewer.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (pcl_visualizer ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (pcl_visualizer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} ${FREENECT2_LIBRARY} ${BOOST_LIBS})