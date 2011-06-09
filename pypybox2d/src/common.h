#ifndef _H_COMMON
#define _H_COMMON
#include <Python.h>
#include "structmember.h"
#include <stdlib.h>
#include <math.h>
#include <float.h>

// http://wiki.python.org/moin/PortingExtensionModulesToPy3k
#if PY_MAJOR_VERSION >= 3
#define IS_PY3K
#endif

#define TYPE_NAME(name) ("pypybox2d.common." name)

typedef struct {
    PyObject_HEAD
    double x;
    double y;
} Vec2;

typedef struct {
    PyObject_HEAD
    Vec2* col1;
    Vec2* col2;
} Mat22;

typedef struct {
    PyObject_HEAD
    Vec2* position;
    Mat22* rotation;
} Transform;

typedef struct {
    PyObject_HEAD
    Vec2* lower_bound;
    Vec2* upper_bound;
} AABB;

extern PyTypeObject Vec2Type;
extern PyTypeObject Mat22Type;
extern PyTypeObject TransformType;
extern PyTypeObject AABBType;

// TODO: separate headers, clean-up
/* vec2 */
extern PyObject* new_Vec2(double x, double y);
extern PyObject* Vec2_richcompare(PyObject *self, PyObject *other, int op);
extern double Vec2_fast_dot(Vec2 *self, Vec2 *other);

/* mat22 */
extern PyObject * new_Mat22(double a11, double a12, double a21, double a22);
extern int Mat22_set_angle(Mat22 *self, PyObject *value, void *closure);
extern PyObject* Mat22_mul_t(Mat22 *self, PyObject *other);
extern PyObject* Mat22_copy(Mat22 *self);
extern PyObject* Mat22_set(Mat22 *self, PyObject* args);
extern PyObject* Mat22_richcompare(PyObject *self, PyObject *other, int op);

/* transform */
extern PyObject* new_Transform(Vec2* position, Mat22* rotation);

/* AABB */
extern PyObject* new_AABB(Vec2* lower_bound, Vec2* upper_bound);

/* common module */
extern PyObject* module_scalar_cross(PyObject *self, PyObject *other);
extern PyObject* module_max_vector(PyObject *self, PyObject *other);
extern PyObject* module_min_vector(PyObject *self, PyObject *other);
extern PyObject* module_clamp_vector(PyObject *none, PyObject *args);
extern int convert_to_double(PyObject *obj, double *dbl);
extern int sequence_to_doubles(PyObject* seq, double* x, double* y);
extern int point_sequence_to_doubles(PyObject* seq, double *x1, double *y1, double *x2, double *y2);

#ifdef IS_PY3K
#define CONVERT_INT_TO_DOUBLE(pyobj, dbl, fail_return) \
    else if (PyLong_Check(pyobj)) {                               \
        dbl = PyLong_AsDouble(pyobj);                               \
        if (dbl == -1.0 && PyErr_Occurred()) {                      \
            return fail_return;                                     \
        }
#else
#define CONVERT_INT_TO_DOUBLE(pyobj, dbl, fail_return) \
      else if (PyInt_Check(pyobj)) {                              \
        dbl = (double)PyInt_AS_LONG(pyobj);                       \
    } else if (PyLong_Check(pyobj)) {                             \
        dbl = PyLong_AsDouble(pyobj);                             \
        if (dbl == -1.0 && PyErr_Occurred()) {                    \
            return fail_return;                                   \
        }
#endif

#define CONVERT_TO_DOUBLE(pyobj, dbl, fail_return) \
    if (PyFloat_Check(pyobj))                                   \
        dbl = PyFloat_AS_DOUBLE(pyobj);                         \
    CONVERT_INT_TO_DOUBLE(pyobj, dbl, fail_return)              \
    } else {                                                    \
        PyErr_SetString(PyExc_ValueError, "Expected number");   \
        return fail_return;                                     \
    }

#define TUPLE_TO_DOUBLES(seq, x, y, fail_return)                 \
    {                                                            \
        PyObject* py_##x=PyTuple_GET_ITEM(seq, 0);               \
        PyObject* py_##y;                                        \
        CONVERT_TO_DOUBLE(py_##x, x, fail_return)                \
                                                                 \
        py_##y=PyTuple_GET_ITEM(seq, 1);                         \
        CONVERT_TO_DOUBLE(py_##y, y, fail_return)                \
    }

#define SEQUENCE_TO_DOUBLES(seq, x, y, fail_return)              \
    {                                                            \
        PyObject* fastseq=PySequence_Fast(seq, "");              \
        PyObject* py_##x;                                        \
        PyObject* py_##y;                                        \
        if (!fastseq) {                                          \
            PyErr_Clear();                                       \
            return fail_return;                                  \
        }                                                        \
        py_##x=PySequence_Fast_GET_ITEM(fastseq, 0);             \
                                                                 \
        if (!convert_to_double(py_##x, &x)) {                    \
            Py_XDECREF(fastseq);                                 \
            return fail_return;                                  \
        }                                                        \
                                                                 \
        py_##y=PySequence_Fast_GET_ITEM(fastseq, 1);             \
        if (!convert_to_double(py_##y, &y)) {                    \
            Py_XDECREF(fastseq);                                 \
            return fail_return;                                  \
        }                                                        \
        Py_XDECREF(fastseq);                                     \
    }

#define VEC2_OR_SEQUENCE(pyobj, _x, _y, fail_return) \
    if (PyObject_TypeCheck(pyobj, &Vec2Type)) {                                        \
        _x = ((Vec2*)pyobj)->x;                                                        \
        _y = ((Vec2*)pyobj)->y;                                                        \
    } else if (PyTuple_Check(pyobj) && PyTuple_Size(pyobj) == 2) {                     \
        TUPLE_TO_DOUBLES(pyobj, _x, _y, fail_return)                                   \
    } else if (PySequence_Check(pyobj) && PySequence_Length(pyobj) == 2) {             \
        PyObject* fastseq=PySequence_Fast(pyobj, "Bad sequence");                      \
        PyObject* py_##x;                                                              \
        PyObject* py_##y;                                                              \
        if (!fastseq) {                                                                \
            return fail_return;                                                        \
        }                                                                              \
        py_##x=PySequence_Fast_GET_ITEM(fastseq, 0);                                   \
                                                                                       \
        if (!convert_to_double(py_##x, &_x)) {                                         \
            Py_XDECREF(fastseq);                                                       \
            return fail_return;                                                        \
        }                                                                              \
                                                                                       \
        py_##y=PySequence_Fast_GET_ITEM(fastseq, 1);                                   \
        if (!convert_to_double(py_##y, &_y)) {                                         \
            Py_XDECREF(fastseq);                                                       \
            return fail_return;                                                        \
        }                                                                              \
        Py_XDECREF(fastseq);                                                           \
    } else {                                                                           \
        PyErr_SetString(PyExc_ValueError, "Expected Vec2 or sequence of length 2");    \
        return fail_return;                                                            \
    }

#define MAT22_OR_TUPLES(pyobj, _a11, _a12, _a21, _a22, fail_return) \
    if (PyObject_TypeCheck(pyobj, &Mat22Type)) {                                       \
        _a11 = ((Mat22*)pyobj)->col1->x;                                               \
        _a12 = ((Mat22*)pyobj)->col2->x;                                               \
        _a21 = ((Mat22*)pyobj)->col1->y;                                               \
        _a22 = ((Mat22*)pyobj)->col2->y;                                               \
    } else if (PyTuple_Check(pyobj) && PyTuple_Size(pyobj) == 2) {                     \
        TUPLE_TO_DOUBLES(PyTuple_GET_ITEM(pyobj, 0), _a11, _a21, fail_return)          \
        TUPLE_TO_DOUBLES(PyTuple_GET_ITEM(pyobj, 1), _a12, _a22, fail_return)          \
    } else if (PySequence_Check(pyobj) && PySequence_Size(pyobj) == 2) {               \
        if (!point_sequence_to_doubles(pyobj, &_a11, &_a21, &_a12, &_a22)) {           \
            return fail_return;                                                        \
        }                                                                              \
    } else {                                                                           \
        PyErr_SetString(PyExc_ValueError, "Expected Mat22 or tuple of length 2");      \
        return fail_return;                                                            \
    }

#define AABB_OR_TUPLES(pyobj, _lx, _ly, _hx, _hy, fail_return) \
    if (PyObject_TypeCheck(pyobj, &AABBType)) {                                       \
        _lx = ((AABB*)pyobj)->lower_bound->x;                                         \
        _ly = ((AABB*)pyobj)->lower_bound->y;                                         \
        _hx = ((AABB*)pyobj)->upper_bound->x;                                         \
        _hy = ((AABB*)pyobj)->upper_bound->y;                                         \
    } else if (PyTuple_Check(pyobj) && PyTuple_Size(pyobj) == 2) {                    \
        TUPLE_TO_DOUBLES(PyTuple_GET_ITEM(pyobj, 0), _lx, _hx, fail_return)           \
        TUPLE_TO_DOUBLES(PyTuple_GET_ITEM(pyobj, 1), _ly, _hy, fail_return)           \
    } else if (PySequence_Check(pyobj) && PySequence_Size(pyobj) == 2) {              \
        if (!point_sequence_to_doubles(pyobj, &_lx, &_ly, &_hx, &_hy)) {              \
            return fail_return;                                                       \
        }                                                                             \
    } else {                                                                          \
        PyErr_SetString(PyExc_ValueError, "Expected AABB or tuple of length 2");      \
        return fail_return;                                                           \
    }

#define VEC2_OR_SEQUENCE_OR_FLOAT(pyobj, _x, _y, _is_float, fail_return) \
    if (PyObject_TypeCheck(pyobj, &Vec2Type)) {                                        \
        _x = ((Vec2*)pyobj)->x;                                                        \
        _y = ((Vec2*)pyobj)->y;                                                        \
        _is_float = 0;                                                                 \
    } else if (PyTuple_Check(pyobj) && PyTuple_Size(pyobj) == 2) {                     \
        TUPLE_TO_DOUBLES(pyobj, _x, _y, fail_return)                                   \
        _is_float = 0;                                                                 \
    } else if (convert_to_double(pyobj, &_x)) {                                        \
        _y = _x;                                                                       \
        _is_float = 1;                                                                 \
    } else if (PyNumber_Check(pyobj)) {                                                \
        _x = PyFloat_AsDouble(pyobj);                                                  \
        _y = _x;                                                                       \
        _is_float = 1;                                                                 \
    } else if (PySequence_Check(pyobj) && PySequence_Length(pyobj) == 2) {             \
        SEQUENCE_TO_DOUBLES(pyobj, _x, _y, fail_return)                                \
        _is_float = 0;                                                                 \
    } else {                                                                           \
        PyErr_SetString(PyExc_ValueError, "Expected Vec2, sequence of length 2, or number"); \
        return fail_return;                                                                  \
    }

#ifndef MIN
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

#ifndef Py_TPFLAGS_CHECKTYPES // py3k assumes this
#define Py_TPFLAGS_CHECKTYPES 0
#endif

#ifdef IS_PY3K
#define PyString_FromString PyUnicode_FromString
//#else
//#ifndef PyString_FromString
//#define PyString_FromString PyUnicode_FromString
//#endif
#endif

// Python 2.5:
#ifndef Py_TYPE
#define Py_TYPE(o) (((PyObject*)o)->ob_type)
#endif

#ifndef PyVarObject_HEAD_INIT
#define PyVarObject_HEAD_INIT(type, size) \
	PyObject_HEAD_INIT(type) size,
#endif


#endif
