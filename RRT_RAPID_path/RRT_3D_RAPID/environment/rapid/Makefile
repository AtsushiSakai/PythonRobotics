OBJS		= rapid.o build.o collide.o overlap.o 

CFLAGS		= -Wall -fPIC -O2 -I. 
CXXFLAGS	= $(CFLAGS)
CPPFLAGS	= -pedantic

TARGET = libRAPID.so

all: $(OBJS)
	$(CXX) $(CFLAGS) -c rapidAPI.cc
	$(CXX) -shared -Wl,-soname,$(TARGET) -o $(TARGET) $(OBJS) rapidAPI.o  
	-cp $(TARGET) ..

$(OBJS): %.o: %.c
	$(CXX) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

clean:
	-rm *.so
	-rm *.o
