# ins10ui: the pages and widgets (Qt Widgets, OpenGL 3.3 core), a library so that the
# widget tests can use them. ins10studio: the application around them.
add_library(ins10ui STATIC
    boardview.cpp boardview.h
    calfieldswidget.cpp calfieldswidget.h
    calibrationpage.cpp calibrationpage.h
    calwizards.cpp calwizards.h
    compasswidget.cpp compasswidget.h
    connectionpanel.cpp connectionpanel.h
    consolepage.cpp consolepage.h
    firstrunpage.cpp firstrunpage.h
    glscene.cpp glscene.h
    language.cpp language.h
    mainwindow.cpp mainwindow.h
    overviewpage.cpp overviewpage.h
    plotspage.cpp plotspage.h
    plotwidget.cpp plotwidget.h
    recordpage.cpp recordpage.h
    settingsdialog.cpp settingsdialog.h
    sphereview.cpp sphereview.h
    statuspage.cpp statuspage.h
    uiutil.cpp uiutil.h
    wizardwidget.cpp wizardwidget.h
    xyplot.cpp xyplot.h
)
if(INS10_HAVE_REPLAY)
    target_sources(ins10ui PRIVATE replaywidget.cpp replaywidget.h)
    target_link_libraries(ins10ui PUBLIC ins10replay)
    target_compile_definitions(ins10ui PUBLIC INS10_HAVE_REPLAY)
endif()
target_include_directories(ins10ui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_compile_definitions(ins10ui PUBLIC INS10_STUDIO_VERSION="${PROJECT_VERSION}")
target_link_libraries(ins10ui PUBLIC
    ins10core ins10device
    Qt6::Gui Qt6::Widgets Qt6::OpenGL Qt6::OpenGLWidgets Qt6::Concurrent)
ins10_set_warnings(ins10ui)

qt_add_executable(ins10studio WIN32 MACOSX_BUNDLE main.cpp)
target_link_libraries(ins10studio PRIVATE ins10ui)
ins10_set_warnings(ins10studio)
set_target_properties(ins10studio PROPERTIES OUTPUT_NAME ins10studio)

# Ukrainian translation, compiled into the UI library (so into the application and the
# widget tests) as :/i18n/ins10studio_uk.qm.
# 'cmake --build <dir> --target update_translations' refreshes the .ts from the sources.
set(INS10_TS_FILES ${PROJECT_SOURCE_DIR}/i18n/ins10studio_uk.ts)
if(Qt6_VERSION VERSION_GREATER_EQUAL 6.7)
    qt_add_translations(TARGETS ins10ui
        SOURCE_TARGETS ins10core ins10device ins10ui ins10studio
        TS_FILES ${INS10_TS_FILES}
        RESOURCE_PREFIX /i18n
        LUPDATE_OPTIONS -no-obsolete -locations none
        IMMEDIATE_CALL NO_GENERATE_PLURALS_TS_FILE)
else()
    file(GLOB_RECURSE INS10_I18N_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.h)
    qt_add_translations(ins10ui TS_FILES ${INS10_TS_FILES} SOURCES ${INS10_I18N_SOURCES}
        RESOURCE_PREFIX /i18n LUPDATE_OPTIONS -no-obsolete -locations none)
endif()
