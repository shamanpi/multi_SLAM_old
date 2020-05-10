#include <visualization/vis.h>
//#include <visualization/vis_plot_path.h>

#include <iostream>
#include <string>

PyObject *LoadFunctionFromModule(PyObject *pModule, std::string func_name) {
  PyObject *func = PyObject_GetAttrString(pModule, func_name.c_str());
  if (!func || !PyCallable_Check(func)) {
    if (PyErr_Occurred()) {
      PyErr_Print();
    }
    std::cerr << "accessing " << func_name << " function failed!" << std::endl;
    exit(1);
  }
  return func;
}

Visualizer::Visualizer() {
  PyObject *pModule;
  Py_Initialize();
  PyRun_SimpleString("import sys");
  std::string cmd = std::string("sys.path.append('") +
                    std::string(PYTHON_PLOT_PATH) + std::string("')");
  PyRun_SimpleString(cmd.c_str());
  PyRun_SimpleString("sys.argv.append('plot')"); // Fixes error on building Tk
                                                 // window in Windows
  pModule = PyImport_Import(PyUnicode_FromString("plot"));
  if (!pModule) {
    PyErr_Print();
    std::cerr << "Error in pModule" << std::endl;
    exit(1);
  } else {
    addFigFunc = LoadFunctionFromModule(pModule, "AddFigure");

    showFigsFunc = LoadFunctionFromModule(pModule, "ShowFigures");

    pauseFigsFunc = LoadFunctionFromModule(pModule, "PauseFigures");

    plotFunc = LoadFunctionFromModule(pModule, "Plot");

    setFigAxisFunc = LoadFunctionFromModule(pModule, "SetAxisLims");

    clearFigFunc = LoadFunctionFromModule(pModule, "ClearFigure");
  }

  Py_DECREF(pModule);
}

size_t Visualizer::AddFigure(std::string figure_title, bool force_square) {
  PyObject *pValue;
  PyObject *pArgs = PyTuple_New(2);
  PyTuple_SetItem(pArgs, 0, PyUnicode_FromString(figure_title.c_str()));
  PyTuple_SetItem(pArgs, 1, force_square ? Py_True : Py_False);

  pValue = PyObject_CallObject(addFigFunc, pArgs);
  PyErr_Print();

  size_t fig_id = PyLong_AsSize_t(pValue);
  PyErr_Print();

  Py_XDECREF(pValue);
  Py_DECREF(pArgs);

  return fig_id;
}

void Visualizer::ShowFigures() {
  PyObject *pValue;
  PyObject *pArgs = PyTuple_New(0);

  pValue = PyObject_CallObject(showFigsFunc, pArgs);
  PyErr_Print();

  Py_XDECREF(pValue);
  Py_DECREF(pArgs);
}

void Visualizer::PauseFigures(double pause_time_s) {
  PyObject *pValue;
  PyObject *pArgs = PyTuple_New(1);
  PyTuple_SetItem(pArgs, 0, PyFloat_FromDouble(pause_time_s));

  pValue = PyObject_CallObject(pauseFigsFunc, pArgs);
  PyErr_Print();

  Py_XDECREF(pValue);
  Py_DECREF(pArgs);
}

void Visualizer::Plot(size_t fig_id, const std::vector<Eigen::VectorXd> &points,
                      std::string format_string) {
  PyObject *pValue;
  PyObject *pArgs = PyTuple_New(3);
  PyTuple_SetItem(pArgs, 0, PyLong_FromSize_t(fig_id));
  PyObject *X_list = PyList_New(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    PyObject *x_t = PyList_New(points[i].size());
    for (int j = 0; j < points[i].size(); ++j) {
      PyList_SetItem(x_t, j, PyFloat_FromDouble(points[i][j]));
    }
    PyList_SetItem(X_list, i, x_t);
  }
  PyTuple_SetItem(pArgs, 1, X_list);
  PyTuple_SetItem(pArgs, 2, PyUnicode_FromString(format_string.c_str()));

  pValue = PyObject_CallObject(plotFunc, pArgs);
  PyErr_Print();

  Py_XDECREF(pValue);
  Py_DECREF(pArgs);
}

void Visualizer::SetFigureAxisLimits(size_t fig_id,
                                     const Eigen::Vector2d &x_lim,
                                     const Eigen::Vector2d &y_lim) {
  PyObject *pValue;
  PyObject *pArgs = PyTuple_New(3);
  PyTuple_SetItem(pArgs, 0, PyLong_FromSize_t(fig_id));
  PyObject *x_lim_py = PyList_New(2);
  PyList_SetItem(x_lim_py, 0, PyFloat_FromDouble(x_lim[0]));
  PyList_SetItem(x_lim_py, 1, PyFloat_FromDouble(x_lim[1]));
  PyObject *y_lim_py = PyList_New(2);
  PyList_SetItem(y_lim_py, 0, PyFloat_FromDouble(y_lim[0]));
  PyList_SetItem(y_lim_py, 1, PyFloat_FromDouble(y_lim[1]));

  PyTuple_SetItem(pArgs, 1, x_lim_py);
  PyTuple_SetItem(pArgs, 2, y_lim_py);

  pValue = PyObject_CallObject(setFigAxisFunc, pArgs);
  PyErr_Print();
  Py_XDECREF(pValue);
  Py_DECREF(pArgs);
}

void Visualizer::ClearFigure(size_t fig_id) {
  PyObject *pValue;
  PyObject *pArgs = PyTuple_New(1);
  PyTuple_SetItem(pArgs, 0, PyLong_FromSize_t(fig_id));
  pValue = PyObject_CallObject(clearFigFunc, pArgs);
  PyErr_Print();
  Py_XDECREF(pValue);
  Py_DECREF(pArgs);
}

Visualizer::~Visualizer() {
  Py_XDECREF(addFigFunc);
  Py_XDECREF(showFigsFunc);
  Py_XDECREF(pauseFigsFunc);
  Py_XDECREF(plotFunc);
  Py_XDECREF(setFigAxisFunc);
  Py_XDECREF(clearFigFunc);
  Py_Finalize();
}