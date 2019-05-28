CXX = g++
CFLAGS = -g -O2 -DDEBUG -fPIC -std=c++0x #-Wall

ARIA_INCLUDE=-I/usr/local/Aria/include
ARIA_LINK=-L/usr/local/Aria/lib -lAria -lpthread -ldl -lrt

LFLAGS = $(ARIA_LINK) -lglut -lGL -lfreeimage

OBJS = Utils.o Grid.o GlutClass.o Planning.o PioneerBase.o Robot.o main.o

MKDIR_P = mkdir -p
OUT_DIR=../build-make
PREFIX_OBJS = $(patsubst %.o,${OUT_DIR}/%.o,$(OBJS))

EXEC = program

all: ${OUT_DIR} $(EXEC)

${OUT_DIR}:
	${MKDIR_P} ${OUT_DIR}

%.o: src/%.cpp $(DEPS)
	@echo "Compilando $@"
	@$(CXX) $(CFLAGS) $(ARIA_INCLUDE) -c $< -o ${OUT_DIR}/$@

$(EXEC): $(OBJS)
	@echo "\nLinkando $(EXEC)\n"
	@$(CXX) -o ${OUT_DIR}/$(EXEC) $(PREFIX_OBJS) $(LFLAGS)

clean:
	@echo "Limpando..."
	@rm -f $(PREFIX_OBJS) ${OUT_DIR}/$(EXEC) *~

