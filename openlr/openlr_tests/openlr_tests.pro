TARGET = openlr_tests
CONFIG += console warn_on
CONFIG -= app_bundle
TEMPLATE = app

ROOT_DIR = ../..
DEPENDENCIES = routing search storage indexer editor platform platform_tests_support \
               geometry coding base protobuf \
               osrm stats_client pugixml openlr jansson succinct

include($$ROOT_DIR/common.pri)

QT *= core

HEADERS += \

SOURCES += \
    $$ROOT_DIR/testing/testingmain.cpp \
    openlr_sample_test.cpp \
