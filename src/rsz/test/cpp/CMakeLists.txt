include(openroad)

add_executable(TestBufRem1 TestBufferRemoval.cc)
target_link_libraries(TestBufRem1
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestBufRem1
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestBufRem1
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestBufRem1
)

add_executable(TestBufferRemoval2 TestBufferRemoval2.cc)
target_link_libraries(TestBufferRemoval2
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestBufferRemoval2
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestBufferRemoval2
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestBufferRemoval2
)


add_executable(TestBufferRemoval3 TestBufferRemoval3.cc)
target_link_libraries(TestBufferRemoval3
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst_integrated_fixture
        ${TCL_LIBRARY}
)

target_include_directories(TestBufferRemoval3
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestBufferRemoval3
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestBufferRemoval3
)

add_executable(TestResizer TestResizer.cc)
target_link_libraries(TestResizer
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        defin
        tst_integrated_fixture
        ${TCL_LIBRARY}
)

target_include_directories(TestResizer
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestResizer
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestResizer
)

add_executable(TestInsertBuffer TestInsertBuffer.cpp)
target_link_libraries(TestInsertBuffer
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        odb
        odb_test_helper
        tst_integrated_fixture
        ${TCL_LIBRARY}
)

target_include_directories(TestInsertBuffer
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestInsertBuffer
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestInsertBuffer
)

add_executable(TestNestedJournal TestNestedJournal.cc)
target_link_libraries(TestNestedJournal
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst_integrated_fixture
        ${TCL_LIBRARY}
)

target_include_directories(TestNestedJournal
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
      ${PROJECT_SOURCE_DIR}/src/rsz/src/move
)

gtest_discover_tests(TestNestedJournal
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestNestedJournal
)

add_executable(TestResizerMt TestResizerMt.cc)
target_link_libraries(TestResizerMt
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        odb
        tst_integrated_fixture
        ${TCL_LIBRARY}
)

target_include_directories(TestResizerMt
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
      ${PROJECT_SOURCE_DIR}/src/rsz/src/move
      ${PROJECT_SOURCE_DIR}/src/rsz/src/policy
)

gtest_discover_tests(TestResizerMt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestResizerMt
)

add_executable(TestRepairHoldRollback TestRepairHoldRollback.cc)
target_link_libraries(TestRepairHoldRollback
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst_integrated_fixture
        ${TCL_LIBRARY}
)

target_include_directories(TestRepairHoldRollback
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestRepairHoldRollback
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestRepairHoldRollback
)

add_executable(TestLambdaUpdater TestLambdaUpdater.cc)
target_link_libraries(TestLambdaUpdater
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestLambdaUpdater
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestLambdaUpdater
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestLambdaUpdater
)

add_executable(TestLambdaSeeder TestLambdaSeeder.cc)
target_link_libraries(TestLambdaSeeder
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestLambdaSeeder
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestLambdaSeeder
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestLambdaSeeder
)

add_executable(TestFlowProjection TestFlowProjection.cc)
target_link_libraries(TestFlowProjection
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestFlowProjection
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestFlowProjection
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestFlowProjection
)

add_executable(TestTimingScale TestTimingScale.cc)
target_link_libraries(TestTimingScale
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestTimingScale
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestTimingScale
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestTimingScale
)

add_executable(TestCostTerms TestCostTerms.cc)
target_link_libraries(TestCostTerms
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestCostTerms
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestCostTerms
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestCostTerms
)

add_executable(TestSweepEngine TestSweepEngine.cc)
target_link_libraries(TestSweepEngine
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestSweepEngine
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestSweepEngine
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestSweepEngine
)

add_executable(TestCapRecheck TestCapRecheck.cc)
target_link_libraries(TestCapRecheck
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestCapRecheck
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestCapRecheck
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestCapRecheck
)

add_executable(TestElectricalVeto TestElectricalVeto.cc)
target_link_libraries(TestElectricalVeto
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestElectricalVeto
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestElectricalVeto
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestElectricalVeto
)

add_executable(TestSizeVthGrid TestSizeVthGrid.cc)
target_link_libraries(TestSizeVthGrid
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestSizeVthGrid
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestSizeVthGrid
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestSizeVthGrid
)

add_executable(TestLambdaMuPairing TestLambdaMuPairing.cc)
target_link_libraries(TestLambdaMuPairing
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestLambdaMuPairing
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestLambdaMuPairing
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestLambdaMuPairing
)

add_executable(TestInitPass TestInitPass.cc)
target_link_libraries(TestInitPass
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestInitPass
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestInitPass
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestInitPass
)

add_executable(TestGuards TestGuards.cc)
target_link_libraries(TestGuards
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestGuards
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestGuards
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestGuards
)

add_executable(TestTermination TestTermination.cc)
target_link_libraries(TestTermination
        OpenSTA
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        dbSta_lib
        utl_lib
        rsz_lib
        grt_lib
        dpl_lib
        stt_lib
        odb
        tst
        ${TCL_LIBRARY}
)

target_include_directories(TestTermination
    PRIVATE
      ${PROJECT_SOURCE_DIR}/src/rsz/src
)

gtest_discover_tests(TestTermination
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
)

add_dependencies(build_and_test TestTermination
)
