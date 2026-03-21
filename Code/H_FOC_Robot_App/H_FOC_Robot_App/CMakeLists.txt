cmake_minimum_required(VERSION 3.19)
project(H_FOC_Robot_App LANGUAGES CXX)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets SerialPort)

qt_standard_project_setup()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

qt_add_executable(H_FOC_Robot_App
    WIN32 MACOSX_BUNDLE
    main.cpp
    core/appconfig.h
    core/armtypes.h
    protocol/canprotocol.h
    protocol/canprotocol.cpp
    canfd/slcancanfddevice.h
    canfd/slcancanfddevice.cpp
    kinematics/armkinematics.h
    kinematics/armkinematics.cpp
    controller/armcontroller.h
    controller/armcontroller.cpp
    ui/mainwindow.h
    ui/mainwindow.cpp
    ui/mainwindow.ui
    ui/widgets/armcanvaswidget.h
    ui/widgets/armcanvaswidget.cpp
)

target_include_directories(H_FOC_Robot_App
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(H_FOC_Robot_App
    PRIVATE
        Qt::Core
        Qt::Widgets
        Qt::SerialPort
)

install(TARGETS H_FOC_Robot_App
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET H_FOC_Robot_App
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})
