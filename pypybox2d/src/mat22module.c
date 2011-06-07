#include "common.h"

#define MAT22_FORMAT "Mat22[%.2g, %.2g\n" \
                     "      %.2g, %2.g]"
#define BUF_SIZE 200

PyTypeObject Mat22Type;
static PyObject *
Mat22_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    Mat22 *self;

    self = (Mat22 *)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->col1 = (Vec2*)new_Vec2(1.0, 0.0);
        self->col2 = (Vec2*)new_Vec2(0.0, 1.0);
    }

    return (PyObject *)self;
}

PyObject *
new_Mat22(double a11, double a12, double a21, double a22) {
    Mat22* ret=(Mat22*)Mat22_new(&Mat22Type, NULL, NULL);
    ret->col1->x = a11;
    ret->col2->x = a12;
    ret->col1->y = a21;
    ret->col2->y = a22;
    return (PyObject*)ret;
}

static PyMemberDef Mat22_members[] = {
    {"_col1", T_OBJECT, offsetof(Mat22, col1), READONLY,
     "The x component"},
    {"_col2", T_OBJECT, offsetof(Mat22, col2), READONLY,
     "The y component"},
    {NULL}  /* Sentinel */
};

static void
Mat22_dealloc(Mat22* self)
{
    Py_XDECREF(self->col1);
    Py_XDECREF(self->col2);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

/* returns new Mat22 instance with given values */
static int
Mat22_init(Mat22 *self, PyObject *args, PyObject *kwds)
{
    static char *kwlist[] = {"col1", "col2", NULL};
    PyObject *col1=NULL, *col2=NULL;
    double cx, cy;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OO", kwlist, 
                                      &col1, &col2))
        return -1; 

    if (col1 && col1 != Py_None) {
        VEC2_OR_SEQUENCE(col1, cx, cy, -1)
        self->col1->x = cx;
        self->col1->y = cy;
    }

    if (col2 && col2 != Py_None) {
        VEC2_OR_SEQUENCE(col2, cx, cy, -1)
        self->col2->x = cx;
        self->col2->y = cy;
    }
    return 0;
}

/* getters/setters */
static PyObject *
Mat22_get_col1(Mat22 *self, void *closure)
{
    return new_Vec2(self->col1->x, self->col1->y);
}

static int
Mat22_set_col1(Mat22 *self, PyObject* value, void *closure)
{
    double x, y;
    VEC2_OR_SEQUENCE(value, x, y, -1);
    self->col1->x=x;
    self->col1->y=y;

    return 0;
}

static int
Mat22_set_col2(Mat22 *self, PyObject* value, void *closure)
{
    double x, y;
    VEC2_OR_SEQUENCE(value, x, y, -1);
    self->col2->x=x;
    self->col2->y=y;

    return 0;
}

static PyObject *
Mat22_get_col2(Mat22 *self, void *closure)
{
    return new_Vec2(self->col2->x, self->col2->y);
}
static PyObject *
Mat22_get_angle(Mat22 *self, void *closure)
{
    return PyFloat_FromDouble(atan2(self->col1->y, self->col1->x));
}

int
Mat22_set_angle(Mat22 *self, PyObject *value, void *closure)
{
    double angle, c, s;
    CONVERT_TO_DOUBLE(value, angle, -1)

    c=cos(angle);
    s=sin(angle);
    self->col1->x=c; self->col2->x=-s;
    self->col1->y=s; self->col2->y=c;
    return 0;
}

static PyObject *
Mat22_get_inverse(Mat22 *self, void *closure)
{
    double a = self->col1->x;
    double b = self->col2->x;
    double c = self->col1->y;
    double d = self->col2->y;

    double det=a*d - b*c;
    if (det != 0.0) {
        det = 1.0 / det;
    }

    return new_Mat22( det*d, -det*c,
                     -det*b,  det*a);
}

/* end getters/setters */

/* returns new ref string formatted */
static PyObject *
Mat22_repr(Mat22 *self)
{
    char buf[BUF_SIZE];
    PyOS_snprintf(buf, BUF_SIZE, MAT22_FORMAT, 
            self->col1->x, self->col2->x, self->col1->y, self->col2->y);
    return PyString_FromString(buf);
}

static PyObject *
Mat22_add(PyObject *self, PyObject *other)
{
    double a11, a12;
    double a21, a22;

    double b11, b12;
    double b21, b22;

    MAT22_OR_TUPLES(self, a11, a12, a21, a22, NULL)
    MAT22_OR_TUPLES(other, b11, b12, b21, b22, NULL)
    return new_Mat22(a11 + b11, a12 + b12,
                     a21 + b21, a22 + b22);
}

static PyObject *
Mat22_iadd(Mat22 *self, PyObject *other)
{
    double b11, b12;
    double b21, b22;

    MAT22_OR_TUPLES(other, b11, b12, b21, b22, NULL)

    self->col1->x += b11;
    self->col2->x += b12;
    self->col1->y += b21;
    self->col2->y += b22;

    Py_INCREF(self);
    return (PyObject*)self;
}

static PyObject *
Mat22_sub(PyObject *self, PyObject *other)
{
    double a11, a12;
    double a21, a22;

    double b11, b12;
    double b21, b22;

    MAT22_OR_TUPLES(self, a11, a12, a21, a22, NULL)
    MAT22_OR_TUPLES(other, b11, b12, b21, b22, NULL)
    return new_Mat22(a11 - b11, a12 - b12,
                     a21 - b21, a22 - b22);
}

static PyObject *
Mat22_isub(Mat22 *self, PyObject *other)
{
    double b11, b12;
    double b21, b22;

    MAT22_OR_TUPLES(other, b11, b12, b21, b22, NULL)

    self->col1->x -= b11;
    self->col2->x -= b12;
    self->col1->y -= b21;
    self->col2->y -= b22;

    Py_INCREF(self);
    return (PyObject*)self;
}

static PyObject *
Mat22_mul(Mat22 *self, PyObject *other)
{
    double v0, v1;
    VEC2_OR_SEQUENCE(other, v0, v1, NULL)

    return new_Vec2((self->col1->x * v0 + self->col2->x * v1),
                    (self->col1->y * v0 + self->col2->y * v1));
}

static PyObject*
Mat22_mul_t_vec(Mat22 *self, PyObject *other) {
    double v0, v1;
    double a11 = self->col1->x, a12 = self->col2->x;
    double a21 = self->col1->y, a22 = self->col2->y;

    VEC2_OR_SEQUENCE(other, v0, v1, NULL)
    return new_Vec2(a11 * v0 + a21 * v1,
                    a12 * v0 + a22 * v1);
}

PyObject *
Mat22_mul_t(Mat22 *self, PyObject *other)
{
    /*
        Transpose multiplication.

        If the argument is a vector, return:
          A^T * vector (where ^T is transpose)
        If the argument is a matrix, return:
          A^T * matrix
     */
    PyObject* ret=Mat22_mul_t_vec(self, other);
    if (!ret) {
        PyErr_Clear();
        if (PyObject_TypeCheck(other, &Mat22Type)) {
            Mat22* _other=(Mat22*)other;
            ret = (PyObject*)new_Mat22(
                        Vec2_fast_dot(self->col1, _other->col1), Vec2_fast_dot(self->col1, _other->col2),
                        Vec2_fast_dot(self->col2, _other->col1), Vec2_fast_dot(self->col2, _other->col2)
                    );
        } else {
            PyErr_SetString(PyExc_TypeError, "Expected Vec2 or Mat22");
            return NULL;
        }
    }
    return ret;
}

static PyObject *
Mat22_neg(Mat22 *self)
{
    return new_Mat22(-self->col1->x, -self->col2->x,
                     -self->col1->y, -self->col2->y);
}

PyObject *
Mat22_copy(Mat22 *self)
{
    return new_Mat22(self->col1->x, self->col2->x,
                     self->col1->y, self->col2->y);
}

static PyObject *
Mat22_abs(Mat22 *self)
{
    return new_Mat22(fabs(self->col1->x), fabs(self->col2->x),
                     fabs(self->col1->y), fabs(self->col2->y));
}

static int
Mat22_nonzero(Mat22 *self)
{
    return (self->col1->x != 0.0 || self->col1->y != 0.0 ||
            self->col2->x != 0.0 || self->col2->y != 0.0);
}


PyObject *
Mat22_richcompare(PyObject *self, PyObject *other, int op)
{
    double a11, a12;
    double a21, a22;

    double b11, b12;
    double b21, b22;

    int ret;
    MAT22_OR_TUPLES(self, a11, a12, a21, a22, NULL)
    MAT22_OR_TUPLES(other, b11, b12, b21, b22, NULL)

    switch (op) {
    case Py_EQ:
        ret = (a11 == b11 && 
               a12 == b12 && 
               a21 == b21 && 
               a22 == b22);
        break;
    case Py_NE:
        ret = (a11 != b11 ||
               a12 != b12 ||
               a21 != b21 ||
               a22 != b22);
        break;
    case Py_LE:
        ret = (a11 <= b11 && 
               a12 <= b12 && 
               a21 <= b21 && 
               a22 <= b22);
        break;
    case Py_GE:
        ret = (a11 >= b11 && 
               a12 >= b12 && 
               a21 >= b21 && 
               a22 >= b22);
        break;
    case Py_LT:
        ret = (a11 < b11 && 
               a12 < b12 && 
               a21 < b21 && 
               a22 < b22);
        break;
    case Py_GT:
        ret = (a11 > b11 && 
               a12 > b12 && 
               a21 > b21 && 
               a22 > b22);
        break;
    }
    return PyBool_FromLong(ret);
}


/*** Sequence stuff ***/
static Py_ssize_t
Mat22_seq_length(Mat22 *self)
{
    return 2;
}

static PyObject *
Mat22_item(Mat22 *self, Py_ssize_t i)
{
    if (i == 0) {
        Py_XINCREF(self->col1);
        return (PyObject*)self->col1;
    } else if (i == 1) {
        Py_XINCREF(self->col2);
        return (PyObject*)self->col2;
    } else {
        PyErr_SetString(PyExc_IndexError, "Expected index in (0, 1)");
        return NULL;
    }
}


static int
Mat22_ass_item(Mat22 *self, Py_ssize_t i, PyObject *_value)
{
    double x, y;
    VEC2_OR_SEQUENCE(_value, x, y, -1)

    if (i == 0) {
        self->col1->x = x;
        self->col1->y = y;
    } else if (i == 1) {
        self->col2->x = x;
        self->col2->y = y;
    } else {
        PyErr_SetString(PyExc_IndexError, "Expected index in (0, 1)");
        return -1;
    }
    return 0;
}

static int
Mat22_contains(Mat22 *self, PyObject *_value)
{
    double x, y;
    int is_float;
    VEC2_OR_SEQUENCE_OR_FLOAT(_value, x, y, is_float, -1)
    if (is_float) {
        return (self->col1->x == x ||
                self->col1->y == x ||
                self->col2->x == x ||
                self->col2->y == x) ? 1 : 0;

    } else {
        return ((self->col1->x == x &&
                 self->col1->y == y) ||
                (self->col2->x == x &&
                 self->col2->y == y)) ? 1 : 0;
    }
}

static PySequenceMethods Mat22_as_sequence = {
    (lenfunc)Mat22_seq_length,                /* sq_length */
    0,                                        /* sq_concat */
    0,                                        /* sq_repeat */
    (ssizeargfunc)Mat22_item,                 /* sq_item */
    0,                                        /* sq_slice */
    (ssizeobjargproc)Mat22_ass_item,          /* sq_ass_item */
    0,                                        /* sq_ass_slice */
    (objobjproc)Mat22_contains,               /* sq_contains */
    0,                                        /* sq_inplace_concat */
    0,                                        /* sq_inplace_repeat */
};
/*** End sequence stuff ***/

/*** Methods ***/
static PyObject *
Mat22_set_identity(Mat22 *self, Mat22* other)
{
    PyObject* ret = Py_True;
    self->col1->x = 1.0;
    self->col1->y = 0.0;
    self->col2->x = 0.0;
    self->col2->y = 1.0;

    Py_INCREF(ret);
    return ret;
}

static PyObject *
Mat22_set_zero(Mat22 *self, Mat22* other)
{
    PyObject* ret = Py_True;
    self->col1->x = 0.0;
    self->col1->y = 0.0;
    self->col2->x = 0.0;
    self->col2->y = 0.0;

    Py_INCREF(ret);
    return ret;
}

PyObject *
Mat22_set(Mat22 *self, PyObject* args)
{
    PyObject *ret = Py_True;
    
    double a11, a12;
    double a21, a22;

    if (PyTuple_Check(args) && PyTuple_Size(args) == 1) {
        args = PyTuple_GET_ITEM(args, 0);
    }

    MAT22_OR_TUPLES(args, a11, a12, a21, a22, NULL)
    self->col1->x = a11;
    self->col1->y = a21;

    self->col2->x = a12;
    self->col2->y = a22;

    Py_INCREF(ret);
    return ret;
}

static PyObject *
Mat22_solve(Mat22 *self, PyObject* other)
{
    double vx, vy;
    double a = self->col1->x;
    double b = self->col2->x;
    double c = self->col1->y;
    double d = self->col2->y;
    double det=a*d - b*c;
    VEC2_OR_SEQUENCE(other, vx, vy, NULL)

    if (det != 0.0) {
        det = 1.0 / det;
    }

    return new_Vec2(det * (d * vx - b * vy),
                    det * (a * vy - c * vx));
}

static PyMethodDef Mat22_methods[] = {
    {"__copy__", (PyCFunction)Mat22_copy, METH_NOARGS,
     "Copy the matrix"
    },
    {"copy", (PyCFunction)Mat22_copy, METH_NOARGS,
     "Copy the matrix"
    },
    {"set_zero", (PyCFunction)Mat22_set_zero, METH_NOARGS,
     "Zero the matrix [0 0; 0 0]"
    },
    {"set_identity", (PyCFunction)Mat22_set_identity, METH_NOARGS,
     "Identity matrix [1 0; 0 1]"
    },
    {"set", (PyCFunction)Mat22_set, METH_VARARGS,
     "Set the columns, given col1, col2"
    },
    {"solve", (PyCFunction)Mat22_solve, METH_O,
     "Solve A * x = vec, a column vector. This is more efficient "
     "than computing the inverse in one-shot cases."
    },
    {"mul_t", (PyCFunction)Mat22_mul_t, METH_O,
        "Transpose multiplication.\n\n"
        "If the argument is a vector, return:\n"
        "  A^T * vector (where ^T is transpose)\n"
        "If the argument is a matrix, return:\n"
        "  A^T * matrix\n"
    },
    {"mul_tv", (PyCFunction)Mat22_mul_t_vec, METH_O,
        "Transpose multiplication with vector.\n\n"
        "  A^T * vector (where ^T is transpose)\n"
    },
    {NULL}  /* Sentinel */
};

/*** End methods ***/

static PyGetSetDef Mat22_getseters[] = {
    {"col1", 
     (getter)Mat22_get_col1, (setter)Mat22_set_col1,
     "Get/set col1",
     NULL},
    {"col2", 
     (getter)Mat22_get_col2, (setter)Mat22_set_col2,
     "Get/set col2",
     NULL},
    {"angle", 
     (getter)Mat22_get_angle, (setter)Mat22_set_angle,
     "Angle in radians",
     NULL},
    {"inverse", 
     (getter)Mat22_get_inverse, NULL,
     "Matrix inverse, if it exists",
     NULL},
    {NULL}  /* Sentinel */
};

static PyNumberMethods Mat22_as_number = {
    (binaryfunc)Mat22_add,            /*nb_add*/
    (binaryfunc)Mat22_sub,            /*nb_subtract*/
    (binaryfunc)Mat22_mul,            /*nb_multiply*/
#ifndef IS_PY3K
    0,                               /*nb_divide*/
#endif
    0,                               /*nb_remainder*/
    0,                               /*nb_divmod*/
    0,                               /*nb_power*/
    (unaryfunc)Mat22_neg,             /*nb_negative*/
    (unaryfunc)Mat22_copy,            /*tp_positive*/
    (unaryfunc)Mat22_abs,             /*tp_absolute*/
    (inquiry)Mat22_nonzero,           /*tp_nonzero*/
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
    0,                               /*nb_long*/
    0,                               /*nb_float*/
#ifndef IS_PY3K
    0,                               /*nb_oct*/
    0,                               /*nb_hex*/
#endif
    (binaryfunc)Mat22_iadd,          /* nb_inplace_add */
    (binaryfunc)Mat22_isub,          /* nb_inplace_subtract */
    0,                               /* nb_inplace_multiply */
#ifndef IS_PY3K
    0,                               /* nb_inplace_divide */
#endif
    0,                               /* nb_inplace_remainder */
    0,                               /* nb_inplace_power */
    0,                               /* nb_inplace_lshift */
    0,                               /* nb_inplace_rshift */
    0,                               /* nb_inplace_and */
    0,                               /* nb_inplace_xor */
    0,                               /* nb_inplace_or */
    0,                               /* nb_floor_divide */
    0,                               /* nb_true_divide */
    0,                               /* nb_inplace_floor_divide */
    0,                               /* nb_inplace_true_divide */
    0,                               /* nb_index */
};

PyTypeObject Mat22Type = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "common.Mat22",                        /* tp_name*/
    sizeof(Mat22),                         /* tp_basicsize*/
    0,                                     /* tp_itemsize*/
    (destructor)Mat22_dealloc,             /* tp_dealloc*/
    0,                                     /* tp_print*/
    0,                                     /* tp_getattr*/
    0,                                     /* tp_setattr*/
    0,                                     /* tp_compare (reserved in py3k)*/
    (reprfunc)Mat22_repr,                  /* tp_repr*/
    &Mat22_as_number,                      /* tp_as_number*/
    &Mat22_as_sequence,                    /* tp_as_sequence*/
    0,                                     /* tp_as_mapping*/
    0,                                     /* tp_hash */
    0,                                     /* tp_call*/
    0,                                     /* tp_str*/
    0,                                     /* tp_getattro*/
    0,                                     /* tp_setattro*/
    0,                                     /* tp_as_buffer*/
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_CHECKTYPES, /*tp_flags*/
    "2D Vector",                           /* tp_doc */
    0,                                     /* tp_traverse */
    0,                                     /* tp_clear */
    (richcmpfunc)Mat22_richcompare,        /* tp_richcompare */
    0,                                     /* tp_weaklistoffset */
    0,                                     /* tp_iter */
    0,                                     /* tp_iternext */
    Mat22_methods,                         /* tp_methods */
    Mat22_members,                         /* tp_members */
    Mat22_getseters,                       /* tp_getset */
    0,                                     /* tp_base */
    0,                                     /* tp_dict */
    0,                                     /* tp_descr_get */
    0,                                     /* tp_descr_set */
    0,                                     /* tp_dictoffset */
    (initproc)Mat22_init,                  /* tp_init */
    0,                                     /* tp_alloc */
    Mat22_new,                             /* tp_new */
};

