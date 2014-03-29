#libgtkmm-3.0-dev" is needed to build the demo_3d binary

all: code demo_3d

HDRS = code.h giro/helper_3dmath.h giro/I2Cdev.h giro/MPU6050_6Axis_MotionApps20.h giro/MPU6050.h giro/demo_3d.h
CMN_OBJS = giro/I2Cdev.o giro/MPU6050.o
D3D_OBJS = giro/main_3d.o giro/demo_3d.o
MAI_OBJS = code.o

# Set DMP FIFO rate to 20Hz to avoid overflows on 3d

CXXFLAGS = -DDMP_FIFO_RATE=9 -Wall -g -O2 `pkg-config gtkmm-3.0 --cflags --libs`

$(CMN_OBJS) : $(HDRS)

code: $(MAI_OBJS) $(CMN_OBJS) 	
	$(CXX) -o $@ $^ -lm `pkg-config gtkmm-3.0 --cflags --libs`

demo_3d: $(D3D_OBJS) $(CMN_OBJS)
	$(CXX) -o $@ $^ -lm `pkg-config gtkmm-3.0 --cflags --libs`

clean:
	rm -f $(CMN_OBJS) $(D3D_OBJS) $(MAI_OBJS) *~ demo_3d code
