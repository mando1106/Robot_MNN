#pragma once

#include <Python.h>
#include <vector>
#include <string>
#include <stdexcept>

class Plotter {
public:
    static void plot2D(const std::vector<double>& x, const std::vector<double>& y);
    static void plot3D(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z);
    static void show();  // 统一显示所有图
    static int figureCount;
private:
    static void initialize();
    static void finalize();
    static void setVector(const std::string& name, const std::vector<double>& vec);
    static bool initialized;
    
};
