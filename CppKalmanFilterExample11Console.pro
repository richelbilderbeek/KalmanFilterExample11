TEMPLATE = app
CONFIG += console
CONFIG -= qt
QMAKE_CXXFLAGS += -Wall -Wextra -Werror -std=c++0x

INCLUDEPATH += \
    ../../Libraries/fparser4.4.3

win32 {
  INCLUDEPATH += E:/boost_1_50_0

  LIBS += -LE:/boost_1_50_0/stage/lib
  SOURCES += ../../Libraries/fparser4.4.3/fparser.cc
  HEADERS += ../../Libraries/fparser4.4.3/fparser.hh
}

unix {
  SOURCES += ../../Libraries/fparser4.5.1/fparser.cc
  HEADERS += ../../Libraries/fparser4.5.1/fparser.hh

}

SOURCES += \
    main.cpp \
    kalmanfilter.cpp \
    whitenoisesystem.cpp \
    matrix.cpp \
    maindialog.cpp

HEADERS += \
    kalmanfilter.h \
    whitenoisesystem.h \
    matrix.h \
    maindialog.h

