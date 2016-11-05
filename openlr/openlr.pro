# Match OpenLR datata to MWMs.
TARGET = openlr
TEMPLATE = lib
CONFIG += staticlib warn_on

ROOT_DIR = ..

include($$ROOT_DIR/common.pri)

SOURCES += \
  openlr_simple_decoder.cpp \
  openlr_sample.cpp \
  traffic_mode.cpp \

HEADERS += \
  openlr_simple_decoder.hpp \
  openlr_sample.hpp \
  traffic_mode.hpp \
