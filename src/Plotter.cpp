#include "Plotter.h"
#include <string>
#include <iostream>
bool Plotter::initialized = false;
int Plotter::figureCount = 0;
void Plotter::initialize() {
    if (!initialized) {
        Py_Initialize();
        PyRun_SimpleString("import numpy as np");
        PyRun_SimpleString("import matplotlib.pyplot as plt");
        PyRun_SimpleString("from mpl_toolkits.mplot3d import Axes3D");
        initialized = true;
        
    }
}

void Plotter::finalize() {
    if (initialized) {
        Py_Finalize();
        initialized = false;
    }
}

void Plotter::setVector(const std::string& name, const std::vector<double>& vec) {
    PyObject* listObj = PyList_New(vec.size());
    for (size_t i = 0; i < vec.size(); ++i) {
        PyList_SetItem(listObj, i, PyFloat_FromDouble(vec[i]));
    }

    PyObject* main_module = PyImport_AddModule("__main__");
    PyObject* global_dict = PyModule_GetDict(main_module);
    PyDict_SetItemString(global_dict, name.c_str(), listObj);
    Py_DECREF(listObj);
}

void Plotter::plot2D(const std::vector<double>& x, const std::vector<double>& y) {
    if (x.size() != y.size()) throw std::runtime_error("x and y must be the same size");
    initialize();
    setVector("x_cpp", x);
    setVector("y_cpp", y);

//     const char* code = R"(
// import numpy as np
// plt.figure()
// plt.plot(x_cpp, y_cpp)
// plt.xlabel('x')
// plt.ylabel('y')
// plt.title('2D Plot')
// plt.grid(True)
// plt.show()
// )";
//     PyRun_SimpleString(code);

    std::string code = 
        "plt.figure(" + std::to_string(++figureCount) + ")\n"
        "plt.plot(x_cpp, y_cpp)\n"
        "plt.xlabel('x')\n"
        "plt.ylabel('y')\n"
        "plt.title('2D Plot')\n"
        "plt.grid(True)\n";

    PyRun_SimpleString(code.c_str());
    std::cout << "在画图了" << std::endl;
}

void Plotter::plot3D(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z) {
    if (x.size() != y.size() || y.size() != z.size()) throw std::runtime_error("x, y, z must be same size");
    initialize();
    setVector("x_cpp", x);
    setVector("y_cpp", y);
    setVector("z_cpp", z);

//     const char* code = R"(
// import numpy as np
// fig = plt.figure()
// ax = fig.add_subplot(111, projection='3d')
// ax.scatter(x_cpp, y_cpp, z_cpp, c=z_cpp, cmap='viridis', s=5)
// ax.set_xlabel('X')
// ax.set_ylabel('Y')
// ax.set_zlabel('Z')
// plt.title('3D Scatter Plot')
// plt.show()
// )";
//     PyRun_SimpleString(code);

    std::string code =
        "fig = plt.figure(" + std::to_string(++figureCount) + ")\n"
        "ax = fig.add_subplot(111, projection='3d')\n"
        "ax.scatter(x_cpp, y_cpp, z_cpp, c=z_cpp, cmap='viridis', s=5)\n"
        "ax.set_xlabel('X')\n"
        "ax.set_ylabel('Y')\n"
        "ax.set_zlabel('Z')\n"
        "plt.title('3D Scatter Plot')\n";

    PyRun_SimpleString(code.c_str());


}

void Plotter::show() {
    if (!initialized) return;
    PyRun_SimpleString("plt.show()");
    finalize();
}
