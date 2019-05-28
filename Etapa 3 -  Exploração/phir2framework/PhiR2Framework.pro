TEMPLATE = app
CONFIG += console
CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++0x

SOURCES += \
    src/GlutClass.cpp \
    src/PioneerBase.cpp \
    src/Grid.cpp \
    src/main.cpp \
    src/Robot.cpp \
    src/Utils.cpp \
    src/Planning.cpp

OTHER_FILES += \
    CONTROLE.txt

HEADERS += \
    src/Grid.h \
    src/GlutClass.h \
    src/PioneerBase.h \
    src/Robot.h \
    src/Utils.h \
    src/Planning.h


INCLUDEPATH+=/usr/local/Aria/include
LIBS+=-L/usr/local/Aria/lib -lAria
#INCLUDEPATH+=../ARIA/Aria-2.7.2/include
#LIBS+=-L../ARIA/Aria-2.7.2/lib -lAria

LIBS+=-lpthread -lglut -ldl -lrt -lGL -lfreeimage
