TEMPLATE = app
CONFIG += console
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++0x

SOURCES += \
    src/GlutClass.cpp \
    src/Grid.cpp \
    src/main.cpp \
    src/MCL.cpp \
    src/PioneerBase.cpp \
    src/Planning.cpp \
    src/Robot.cpp \
    src/Utils.cpp

OTHER_FILES += \
    CONTROLE.txt

HEADERS += \
    src/GlutClass.h \
    src/Grid.h \
    src/PioneerBase.h \
    src/Planning.h \
    src/MCL.h \
    src/Robot.h \
    src/Utils.h


INCLUDEPATH+=/usr/local/Aria/include
LIBS+=-L/usr/local/Aria/lib -lAria
#INCLUDEPATH+=../ARIA/Aria-2.7.2/include
#LIBS+=-L../ARIA/Aria-2.7.2/lib -lAria

LIBS+=-lpthread -lglut -ldl -lrt -lGL -lfreeimage
