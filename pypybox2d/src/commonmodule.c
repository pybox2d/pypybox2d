#include "common.h"

/* adapted from Python source, floatobject.c:convert_to_double */
int
convert_to_double(PyObject *obj, double *dbl)
{
    if (PyFloat_Check(obj)) {
        *dbl = PyFloat_AS_DOUBLE(obj);
#ifndef IS_PY3K
    } else if (PyInt_Check(obj)) {
        *dbl = (double)PyInt_AS_LONG(obj);
#endif
    } else if (PyLong_Check(obj)) {
        *dbl = PyLong_AsDouble(obj);
        if (*dbl == -1.0 && PyErr_Occurred()) {
            return 0;
        }
    } else {
        PyErr_SetString(PyExc_ValueError, "Expected a number");
        return 0;
    }
    return 1;
}

/*
 *  (x, y) style sequence to the corresponding variables
 *
 * */
int sequence_to_doubles(PyObject* seq, double* x, double* y) {
    PyObject *fastseq=PySequence_Fast(seq, "");
    PyObject *py_x;
    PyObject *py_y;
    if (!fastseq) {
        PyErr_Clear();
        return 0;
    }

    py_x=PySequence_Fast_GET_ITEM(fastseq, 0);
    if (!convert_to_double(py_x, x)) {
        Py_XDECREF(fastseq);
        return 0;
    }

    py_y=PySequence_Fast_GET_ITEM(fastseq, 1);
    if (!convert_to_double(py_y, y)) {
        Py_XDECREF(fastseq);
        return 0;
    }

    Py_XDECREF(fastseq);
    return 1;
}

/*
 *  ((x1, y1), (x2, y2)) style sequence to the corresponding variables
 *
 * */
int point_sequence_to_doubles(PyObject* seq, double *x1, double *y1, double *x2, double *y2) {
    PyObject *outer_seq=PySequence_Fast(seq, "");
    PyObject *inner_seq1, *inner_seq2;
    if (!outer_seq) {
        return 0;
    }

    inner_seq1=PySequence_Fast_GET_ITEM(outer_seq, 0);
    if (!inner_seq1) {
        goto fail;
    }

    inner_seq2=PySequence_Fast_GET_ITEM(outer_seq, 1);
    if (!inner_seq2) {
        goto fail;
    }

    if (!sequence_to_doubles(inner_seq1, x1, y1)) {
        goto fail;
    }

    if (!sequence_to_doubles(inner_seq2, x2, y2)) {
        goto fail;
    }

    Py_XDECREF(outer_seq);
    return 1;

fail:
    Py_XDECREF(outer_seq);
    return 0;
}


PyObject *
module_clamp(PyObject *none, PyObject *args)
{
    double value, low, high;

    if (!PyArg_ParseTuple(args, "ddd:common.clamp", &value, &low, &high)) {
        return NULL;
    }
    return PyFloat_FromDouble(MAX(low, MIN(value, high)));
}

static PyMethodDef module_methods[] = {
    {"min_vector", (PyCFunction)module_min_vector, METH_VARARGS,
     "Get minimum vector from two vectors"
    },
    {"max_vector", (PyCFunction)module_max_vector, METH_VARARGS,
     "Get max vector from two vectors"
    },
    {"scalar_cross", (PyCFunction)module_scalar_cross, METH_VARARGS,
     "Scalar cross vector. Arguments: scalar, vector"
    },
    {"clamp", (PyCFunction)module_clamp, METH_VARARGS,
     "Clamp scalar value in range. Arguments: value, low, high"
    },
    {"clamp_vector", (PyCFunction)module_clamp_vector, METH_VARARGS,
     "Clamp vector in range. Arguments: vector, low_vector, high_vector"
    },
    {NULL}  /* Sentinel */
};

#ifdef IS_PY3K
    static struct PyModuleDef moduledef = {
        PyModuleDef_HEAD_INIT,
        "_common",         //m_name
        NULL,              //m_docstring
        -1,                //m_size
        module_methods,    //m_methods
        NULL,              //m_reload
        0,                 //m_traverse
        0,                 //m_clear
        NULL               //m_free
    };

    #define INITERROR return NULL

    PyObject *
    PyInit__common(void)
#else
    #define INITERROR return
    void
    init_common(void)
#endif
{
#ifdef IS_PY3K
    PyObject *module = PyModule_Create(&moduledef);
#else
    PyObject *module = Py_InitModule("_common", module_methods);
#endif

    if (!module)
        INITERROR;

    if (PyType_Ready(&Vec2Type) < 0)
        INITERROR;
    if (PyType_Ready(&Mat22Type) < 0)
        INITERROR;
    if (PyType_Ready(&TransformType) < 0)
        INITERROR;
    if (PyType_Ready(&AABBType) < 0)
        INITERROR;

    Py_INCREF(&Vec2Type);
    Py_INCREF(&Mat22Type);
    Py_INCREF(&TransformType);
    Py_INCREF(&AABBType);

    PyModule_AddObject(module, "Vec2", (PyObject *)&Vec2Type);
    PyModule_AddObject(module, "Mat22", (PyObject *)&Mat22Type);
    PyModule_AddObject(module, "Transform", (PyObject *)&TransformType);
    PyModule_AddObject(module, "AABB", (PyObject *)&AABBType);

#ifdef IS_PY3K
    return module;
#endif
}

