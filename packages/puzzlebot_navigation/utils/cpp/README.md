### TO BUILD CPP BINDINGS PLEASE RUN
cd packages/puzzlebot_navigation/utils/cpp
g++ -I. -std=c++20 -fPIC -c -o mcl_utils.o mcl_utils.cpp
g++ -shared -o mcl_utils.so mcl_utils.o