/*
 *
 * TODO: use PyCapsule to export this API
 */

#include <Python.h>
#include "structmember.h"
#include <stdlib.h>
#include <math.h>
#include "common.h"

#define VEC2_FORMAT "Vec2(%.4g, %.4g)"
#define BUF_SIZE    100
#define VEC2_DOC    "2D column vector"

PyObject* new_Vec2(double x, double y);
PyTypeObject Vec2Type;

static PyMemberDef Vec2_members[] = {
    {"x", T_DOUBLE, offsetof(Vec2, x), 0,
     "The x component"},
    {"y", T_DOUBLE, offsetof(Vec2, y), 0,
     "The y component"},
    {NULL}  /* Sentinel */
};

static void
Vec2_dealloc(Vec2* self)
{
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject *
Vec2_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    Vec2 *self;

    self = (Vec2 *)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->x = 0.0f;
        self->y = 0.0f;
    }

    return (PyObject *)self;
}

/* returns new Vec2 instance with given values */

static int
Vec2_init(Vec2 *self, PyObject *args, PyObject *kwds)
{
    static char *kwlist[] = {"x", "y", NULL};

    if (! PyArg_ParseTupleAndKeywords(args, kwds, "|dd", kwlist, 
                                      &self->x, &self->y))
        return -1; 

    return 0;
}

/* getters/setters */

static PyObject *
Vec2_length(Vec2 *self, void *closure)
{
    return PyFloat_FromDouble(sqrt((self->x * self->x) + (self->y * self->y)));
}

static PyObject *
Vec2_length_squared(Vec2 *self, void *closure)
{
    return PyFloat_FromDouble((double)((self->x * self->x) + (self->y * self->y)));
}

int valid_float(double* value) {
    /*if (isnan(*value) || isinf(*value)) {
        return 0; // NaN
    }*/
    if (*value != *value) {
        return 0; // NaN
    }
    
    //TODO: check for infinity
    //
    return 1;
}

static PyObject *
Vec2_valid(Vec2 *self, void *closure)
{
    PyObject* ret = Py_True;

    if (!valid_float(&self->x) || !valid_float(&self->y)) {
        ret = Py_False;
    }

    Py_INCREF(ret);
    return ret;
}

/* end getters/setters */

/* returns owned tuple (x, y) */
static PyObject *
Vec2_tuple(Vec2 *self)
{
    return Py_BuildValue("(ff)", self->x, self->y);
}

/* returns owned string formatted: Vec2(x, y) */
static PyObject *
Vec2_repr(Vec2 *self)
{
    char buf[BUF_SIZE];
    PyOS_snprintf(buf, BUF_SIZE, VEC2_FORMAT, self->x, self->y);
    return PyString_FromString(buf);
}

static PyObject *
Vec2_add(PyObject *self, PyObject *other)
{
    double sx, sy;
    double ox, oy;
    VEC2_OR_SEQUENCE(self, sx, sy, NULL)
    VEC2_OR_SEQUENCE(other, ox, oy, NULL)
    return new_Vec2(sx + ox, sy + oy);
}

static PyObject *
Vec2_iadd(PyObject *_self, PyObject *other)
{
    Vec2* self=(Vec2*)_self;
    double ox, oy;
    VEC2_OR_SEQUENCE(other, ox, oy, NULL)
    
    self->x += ox;
    self->y += oy;

    Py_INCREF(_self);
    return _self;
}


static PyObject *
Vec2_sub(PyObject *self, PyObject *other)
{
    double sx, sy;
    double ox, oy;
    VEC2_OR_SEQUENCE(self, sx, sy, NULL)
    VEC2_OR_SEQUENCE(other, ox, oy, NULL)
    return new_Vec2(sx - ox, sy - oy);
}

static PyObject *
Vec2_isub(PyObject *_self, PyObject *other)
{
    Vec2* self=(Vec2*)_self;
    double ox, oy;
    VEC2_OR_SEQUENCE(other, ox, oy, NULL)
    
    self->x -= ox;
    self->y -= oy;

    Py_INCREF(_self);
    return _self;
}

double
Vec2_fast_dot(Vec2 *self, Vec2 *other)
{
    return (self->x * other->x + self->y * other->y);
}

static PyObject *
Vec2_mul(PyObject *self, PyObject *other)
{
    double sx, sy;
    double ox, oy;
    int is_float1, is_float2;
    VEC2_OR_SEQUENCE_OR_FLOAT(self, sx, sy, is_float1, NULL)
    VEC2_OR_SEQUENCE_OR_FLOAT(other, ox, oy, is_float2, NULL)

    if (is_float1 || is_float2) {
        return new_Vec2(sx * ox, sy * oy);
    } else {
        return PyFloat_FromDouble(((sx * ox) + (sy * oy)));
    }
}

static PyObject *
Vec2_imul(PyObject *_self, PyObject *_value)
{
    Vec2* self=(Vec2*)_self;
    double value;
    CONVERT_TO_DOUBLE(_value, value, NULL)
    
    self->x *= value;
    self->y *= value;

    Py_INCREF(_self);
    return _self;
}

static PyObject *
Vec2_floor_divide(PyObject *self, PyObject *other)
{
    double sx, sy;
    double ox, oy;
    int is_float1, is_float2;
    VEC2_OR_SEQUENCE_OR_FLOAT(self, sx, sy, is_float1, NULL)
    VEC2_OR_SEQUENCE_OR_FLOAT(other, ox, oy, is_float2, NULL)

    return new_Vec2((double)((int)(sx / ox)), (double)((int)(sy / oy)));
}

static PyObject *
Vec2_ifloordiv(PyObject *_self, PyObject *_value)
{
    Vec2* self=(Vec2*)_self;
    double value;
    CONVERT_TO_DOUBLE(_value, value, NULL)
   
    self->x = (double)((int)(self->x / value));
    self->y = (double)((int)(self->y / value));

    Py_INCREF(_self);
    return _self;
}
static PyObject *
Vec2_true_divide(PyObject *self, PyObject *other)
{
    double sx, sy;
    double ox, oy;
    int is_float1, is_float2;
    VEC2_OR_SEQUENCE_OR_FLOAT(self, sx, sy, is_float1, NULL)
    VEC2_OR_SEQUENCE_OR_FLOAT(other, ox, oy, is_float2, NULL)

    return new_Vec2(sx / ox, sy / oy);
}

static PyObject *
Vec2_itruediv(PyObject *_self, PyObject *_value)
{
    Vec2* self=(Vec2*)_self;
    double value;
    CONVERT_TO_DOUBLE(_value, value, NULL)
    
    self->x /= value;
    self->y /= value;

    Py_INCREF(_self);
    return _self;
}
static PyObject *
Vec2_neg(Vec2 *self)
{
    return new_Vec2(-self->x, -self->y);
}

static PyObject *
Vec2_copy(Vec2 *self)
{
    return new_Vec2(self->x, self->y);
}

static PyObject *
Vec2_abs(Vec2 *self)
{
    return new_Vec2(fabs(self->x), fabs(self->y));
}

static int
Vec2_nonzero(Vec2 *self)
{
    return (self->x != 0.0 || self->y != 0.0);
}


PyObject *
Vec2_richcompare(PyObject *self, PyObject *other, int op)
{
    int ret=0;
    double sx, sy;
    double ox, oy;
    VEC2_OR_SEQUENCE(self, sx, sy, NULL)
    VEC2_OR_SEQUENCE(other, ox, oy, NULL)

    switch (op) {
    case Py_EQ:
        ret = (sx == ox && sy == oy);
        break;
    case Py_NE:
        ret = (sx != ox || sy != oy);
        break;
    case Py_LE:
        ret = (sx <= ox && sy <= oy);
        break;
    case Py_GE:
        ret = (sx >= ox && sy >= oy);
        break;
    case Py_LT:
        ret = (sx < ox && sy < oy);
        break;
    case Py_GT:
        ret = (sx > ox && sy > oy);
        break;
    }
    return PyBool_FromLong(ret);
}


/*** Sequence stuff ***/
static Py_ssize_t
Vec2_seq_length(Vec2 *self)
{
    return 2;
}

static PyObject *
Vec2_item(Vec2 *self, Py_ssize_t i)
{
    if (i == 0) {
        return PyFloat_FromDouble(self->x);
    } else if (i == 1) {
        return PyFloat_FromDouble(self->y);
    } else {
        PyErr_SetString(PyExc_IndexError, "Expected index in (0, 1)");
        return NULL;
    }
}


static int
Vec2_ass_item(Vec2 *self, Py_ssize_t i, PyObject *_value)
{
    double value;
    CONVERT_TO_DOUBLE(_value, value, -1)

    if (i == 0) {
        self->x = value;
    } else if (i == 1) {
        self->y = value;
    } else {
        PyErr_SetString(PyExc_IndexError, "Expected index in (0, 1)");
        return -1;
    }
    return 0;
}

static int
Vec2_contains(Vec2 *self, PyObject *_value)
{
    double value;
    CONVERT_TO_DOUBLE(_value, value, -1)

    return (self->x == value || self->y == value) ? 1 : 0;
}

static PySequenceMethods Vec2_as_sequence = {
    (lenfunc)Vec2_seq_length,                   /* sq_length */
    0,                                          /* sq_concat */
    0,                                          /* sq_repeat */
    (ssizeargfunc)Vec2_item,                    /* sq_item */
    0,                                          /* sq_slice */
    (ssizeobjargproc)Vec2_ass_item,             /* sq_ass_item */
    0,                                          /* sq_ass_slice */
    (objobjproc)Vec2_contains,                  /* sq_contains */
    0,                                          /* sq_inplace_concat */
    0,                                          /* sq_inplace_repeat */
};
/*** End sequence stuff ***/

/*** Methods ***/

static PyObject *
Vec2_dot(Vec2 *self, PyObject *other)
{
    return Vec2_mul((PyObject*)self, other);
}

static PyObject *
Vec2_cross(Vec2 *self, PyObject *other)
{
    double sx=self->x, sy=self->y;
    double ox, oy;
    int is_float;
    VEC2_OR_SEQUENCE_OR_FLOAT(other, ox, oy, is_float, NULL)

    if (is_float) {
        // Perform the cross product on a vector and a scalar. In 2D this produces
        // a vector.
        return new_Vec2(ox * sy, -ox * sx);
    } else {
        // Perform the cross product on two vectors. In 2D this produces a scalar.
        return PyFloat_FromDouble(sx * oy - sy * ox);
    }
}

static PyObject *
Vec2_zero(Vec2 *self, Vec2* other)
{
    PyObject* ret = Py_True;
    self->x = 0;
    self->y = 0;

    Py_INCREF(ret);
    return ret;
}

static PyObject *
Vec2_set(Vec2 *self, PyObject* args)
{
    double x, y;
    PyObject* ret = Py_True;
    if (PyArg_ParseTuple(args, "dd:Vec2.set", &x, &y)) {
        self->x = x;
        self->y = y;
    }

    Py_INCREF(ret);
    return ret;
}

static PyObject *
Vec2_normalize(Vec2 *self, PyObject* args)
{
    double length=sqrt((self->x * self->x) + (self->y * self->y));
    double inv_length;

    if (length < FLT_EPSILON) {
        return PyFloat_FromDouble(0.0);
    }

    inv_length=1.0 / length;
    self->x *= inv_length;
    self->y *= inv_length;
    return PyFloat_FromDouble(length);
}

static PyObject *
Vec2_skew(Vec2 *self, PyObject *other)
{
    return new_Vec2(-self->y, self->x);
}

PyObject *
Vec2_scalar_cross(Vec2 *self, PyObject *other)
{
    /*
    scalar x vector

    Perform the cross product on a scalar and this vector. In 2D this produces
    a vector.
    */

    double scalar;
    CONVERT_TO_DOUBLE(other, scalar, NULL)
    return new_Vec2(-scalar * self->y, scalar * self->x);
}

PyObject *
module_scalar_cross(PyObject *none, PyObject *args)
{
    double scalar, vx, vy;
    PyObject *_vector;

    if (!PyArg_ParseTuple(args, "dO:common.scalar_cross", &scalar, &_vector)) {
        return NULL;
    }

    VEC2_OR_SEQUENCE(_vector, vx, vy, NULL)
    return new_Vec2(-scalar * vy, scalar * vx);
}

PyObject *
module_min_vector(PyObject *none, PyObject *args)
{
    PyObject *v1, *v2;
    double sx, sy;
    double ox, oy;

    if (!PyArg_ParseTuple(args, "OO:common.min_vector", &v1, &v2)) {
        return NULL;
    }
    VEC2_OR_SEQUENCE(v1, sx, sy, NULL)
    VEC2_OR_SEQUENCE(v2, ox, oy, NULL)
    return new_Vec2(MIN(sx, ox), MIN(sy, oy));
}

PyObject *
module_clamp_vector(PyObject *none, PyObject *args)
{
    PyObject *value, *low, *high;
    double vx, vy;
    double lx, ly;
    double hx, hy;
    double min_x, min_y;

    if (!PyArg_ParseTuple(args, "OOO:common.clamp_vector", &value, &low, &high)) {
        return NULL;
    }

    VEC2_OR_SEQUENCE(value, vx, vy, NULL)
    VEC2_OR_SEQUENCE(low,   lx, ly, NULL)
    VEC2_OR_SEQUENCE(high,  hx, hy, NULL)

    min_x = MIN(vx, hx);
    min_y = MIN(vy, hy);
    return new_Vec2(MAX(lx, min_x), MAX(ly, min_y));
}


PyObject *
module_max_vector(PyObject *none, PyObject *args)
{
    PyObject *v1, *v2;
    double sx, sy;
    double ox, oy;

    if (!PyArg_ParseTuple(args, "OO:common.max_vector", &v1, &v2)) {
        return NULL;
    }
    VEC2_OR_SEQUENCE(v1, sx, sy, NULL)
    VEC2_OR_SEQUENCE(v2, ox, oy, NULL)
    return new_Vec2(MAX(sx, ox), MAX(sy, oy));
}


static PyMethodDef Vec2_methods[] = {
    {"__copy__", (PyCFunction)Vec2_copy, METH_NOARGS,
     "Copy the vector"
    },
    {"copy", (PyCFunction)Vec2_copy, METH_NOARGS,
     "Copy the vector"
    },
    {"dot", (PyCFunction)Vec2_dot, METH_O,
     "Dot product"
    },
    {"cross", (PyCFunction)Vec2_cross, METH_O,
     "Cross product"
    },
    {"_scalar_cross", (PyCFunction)Vec2_scalar_cross, METH_O,
     "Scalar x vector"
    },
    {"zero", (PyCFunction)Vec2_zero, METH_NOARGS,
     "Set vector to (0, 0)"
    },
    {"set", (PyCFunction)Vec2_set, METH_VARARGS,
     "Set vector to (x, y)"
    },
    {"normalize", (PyCFunction)Vec2_normalize, METH_NOARGS,
     "Normalize vector and return length"
    },
    {"skew", (PyCFunction)Vec2_skew, METH_NOARGS,
     "Get the skew of the vector: (-y, x)"
    },
    {NULL}  /* Sentinel */
};

/*** End methods ***/

static PyGetSetDef Vec2_getseters[] = {
    {"length", 
     (getter)Vec2_length, NULL,
     "Vector length",
     NULL},
    {"length_squared", 
     (getter)Vec2_length_squared, NULL,
     "Vector length squared",
     NULL},
    {"valid", 
     (getter)Vec2_valid, NULL,
     "A valid vector?",
     NULL},
    {NULL}  /* Sentinel */
};

static PyNumberMethods Vec2_as_number = {
    (binaryfunc)Vec2_add,            /*nb_add*/
    (binaryfunc)Vec2_sub,            /*nb_subtract*/
    (binaryfunc)Vec2_mul,            /*nb_multiply*/
#ifndef IS_PY3K
    (binaryfunc)Vec2_true_divide,    /*nb_divide*/
#endif
    0,                               /*nb_remainder*/
    0,                               /*nb_divmod*/
    0,                               /*nb_power*/
    (unaryfunc)Vec2_neg,             /*nb_negative*/
    (unaryfunc)Vec2_copy,            /*tp_positive*/
    (unaryfunc)Vec2_abs,             /*tp_absolute*/
    (inquiry)Vec2_nonzero,           /*tp_nonzero*/
    0,                               /*nb_invert*/
    0,                               /*nb_lshift*/
    0,                               /*nb_rshift*/
    0,                               /*nb_and*/
    0,                               /*nb_xor*/
    0,                               /*nb_or*/
#ifndef IS_PY3K
    0,                               /*nb_coerce*/
#endif
    0,                               /*nb_int*/
    0,                               /*nb_long or nb_reserved*/
    0,                               /*nb_float*/
#ifndef IS_PY3K
    0,                               /*nb_oct*/
    0,                               /*nb_hex*/
#endif
    (binaryfunc)Vec2_iadd,           /* nb_inplace_add */
    (binaryfunc)Vec2_isub,           /* nb_inplace_subtract */
    (binaryfunc)Vec2_imul,           /* nb_inplace_multiply */
#ifndef IS_PY3K
    (binaryfunc)Vec2_itruediv,       /* nb_inplace_divide */
#endif
    0,                               /* nb_inplace_remainder */
    0,                               /* nb_inplace_power */
    0,                               /* nb_inplace_lshift */
    0,                               /* nb_inplace_rshift */
    0,                               /* nb_inplace_and */
    0,                               /* nb_inplace_xor */
    0,                               /* nb_inplace_or */
    (binaryfunc)Vec2_floor_divide,   /* nb_floor_divide */
    (binaryfunc)Vec2_true_divide,    /* nb_true_divide */
    (binaryfunc)Vec2_ifloordiv,      /* nb_inplace_floor_divide */
    (binaryfunc)Vec2_itruediv,       /* nb_inplace_true_divide */
    0,                               /* nb_index */
};

PyTypeObject Vec2Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "common.Vec2",             /*tp_name*/
    sizeof(Vec2),              /*tp_basicsize*/
    0,                         /*tp_itemsize*/
    (destructor)Vec2_dealloc,  /*tp_dealloc*/
    0,                         /*tp_print*/
    0,                         /*tp_getattr*/
    0,                         /*tp_setattr*/
    0,                         /*tp_compare*/
    (reprfunc)Vec2_repr,       /*tp_repr*/
    &Vec2_as_number,           /*tp_as_number*/
    &Vec2_as_sequence,         /*tp_as_sequence*/
    0,                         /*tp_as_mapping*/
    0,                         /*tp_hash */
    0,                         /*tp_call*/
    0,                         /*tp_str*/
    0,                         /*tp_getattro*/
    0,                         /*tp_setattro*/
    0,                         /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_CHECKTYPES, /*tp_flags*/
    VEC2_DOC,                  /* tp_doc */
    0,                         /* tp_traverse */
    0,                         /* tp_clear */
    (richcmpfunc)Vec2_richcompare, /* tp_richcompare */
    0,                         /* tp_weaklistoffset */
    0,                         /* tp_iter */
    0,                         /* tp_iternext */
    Vec2_methods,              /* tp_methods */
    Vec2_members,              /* tp_members */
    Vec2_getseters,            /* tp_getset */
    0,                         /* tp_base */
    0,                         /* tp_dict */
    0,                         /* tp_descr_get */
    0,                         /* tp_descr_set */
    0,                         /* tp_dictoffset */
    (initproc)Vec2_init,       /* tp_init */
    0,                         /* tp_alloc */
    Vec2_new,                  /* tp_new */
    0,                         /* tp_free */
};

PyObject *
new_Vec2(double x, double y) {
    Vec2* ret=(Vec2*)Vec2_new(&Vec2Type, NULL, NULL);
    ret->x = x;
    ret->y = y;
    return (PyObject*)ret;
}
