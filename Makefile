cc = gcc
cxx = g++
CFLAGS = -O3 -Wall
CFLAGS += -I/usr/local/include/opencv4 -I/usr/local/include -L/usr/local/lib -lopencv_dnn -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core
CFLAGS += -lpthread
CFLAGS += -lwiringPi

all:
	@sudo $(cxx) main.cpp ImageFun.hpp LMZPID.hpp $(CFLAGS) -o main

clean:
	rm -rf $(EXE1) $(EXE2) $(EXE3) $(EXE4) $(EXE5) $(EXE6) $(EXE7) $(EXE8) $(EXE9) $(EXE10) $(EXE11) $(EXE12) *.o

# g++ -o3 -Wall -o main main.cpp -I/usr/local/include/opencv4 -I/usr/local/include -L/usr/local/lib -lopencv_dnn -lopencv_ml -lopencv_objdetect -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_imgproc -lopencv_flann -lopencv_core -lpthread -lwiringPi

# cd Desktop/PointMatch2_V1/
