# Match OpenLR datata to MWMs.
TARGET = openlr
TEMPLATE = lib
CONFIG += staticlib warn_on

ROOT_DIR = ..

include($$ROOT_DIR/common.pri)

SOURCES += \
  openlr_sample.cpp \
  openlr_simple_decoder.cpp \
  openlr_simple_parser.cpp \
  traffic_mode.cpp \

HEADERS += \
  openlr_sample.hpp \
  openlr_simple_decoder.hpp \
  openlr_simple_parser.hpp \
  traffic_mode.hpp \
