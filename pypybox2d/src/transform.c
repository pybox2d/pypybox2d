#include "common.h"

#define TRANSFORM_FORMAT "Transform(position=(%.2g, %.2g), angle=%.2g)"
#define BUF_SIZE 200

PyTypeObject TransformType;
static PyObject *
Transform_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    Transform *self;

    self = (Transform *)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->rotation = (Mat22*)new_Mat22(1.0, 0.0, 0.0, 1.0);
        self->position = (Vec2*)new_Vec2(0.0, 0.0);
    }

    return (PyObject *)self;
}

PyObject *
new_Transform(Vec2* position, Mat22* rotation) {
    Transform* ret=(Transform*)Transform_new(&TransformType, NULL, NULL);
    ret->rotation->col1->x = rotation->col1->x;
    ret->rotation->col2->x = rotation->col2->x;
    ret->rotation->col1->y = rotation->col1->y;
    ret->rotation->col2->y = rotation->col2->y;

    ret->position->x = position->x;
    ret->position->y = position->y;
    return (PyObject*)ret;
}

static PyMemberDef Transform_members[] = {
    {"_rotation", T_OBJECT, offsetof(Transform, rotation), READONLY,
     "The rotation"},
    {"_position", T_OBJECT, offsetof(Transform, position), READONLY,
     "The translation"},
    {NULL}  /* Sentinel */
};

static void
Transform_dealloc(Transform* self)
{
    Py_XDECREF(self->rotation);
    Py_XDECREF(self->position);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

/* returns new Transform instance with given values */
static int
Transform_init(Transform *self, PyObject *args, PyObject *kwds)
{
    static char *kwlist[] = {"position", "rotation", "angle", NULL};
    PyObject *position=NULL, *rotation=NULL, *angle=NULL;
    double a, b, c, d;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OOO", kwlist, 
                                      &position, &rotation, &angle))
        return -1; 

    if (position && position != Py_None) {
        VEC2_OR_SEQUENCE(position, a, b, -1)
        self->position->x = a;
        self->position->y = b;
    }

    if (angle && angle != Py_None) {
        Mat22_set_angle(self->rotation, angle, NULL);
    } else if (rotation && rotation != Py_None) {
        MAT22_OR_TUPLES(rotation, a, b, c, d, -1)
        self->rotation->col1->x = a;
        self->rotation->col2->x = b;
        self->rotation->col1->y = c;
        self->rotation->col2->y = d;
    }
    return 0;
}

/* getters/setters */
static PyObject *
Transform_get_angle(Transform *self, void *closure)
{
    return PyFloat_FromDouble(atan2(self->rotation->col1->y, self->rotation->col1->x));
}

static int
Transform_set_angle(Transform *self, PyObject *value, void *closure)
{
    double angle, c, s;
    CONVERT_TO_DOUBLE(value, angle, -1)

    c=cos(angle);
    s=sin(angle);
    self->rotation->col1->x=c; self->rotation->col2->x=-s;
    self->rotation->col1->y=s; self->rotation->col2->y=c;
    return 0;
}

static PyObject *
Transform_get_position(Transform *self, void *closure)
{
    return new_Vec2(self->position->x, self->position->y);
}

static int
Transform_set_position(Transform *self, PyObject *value, void *closure)
{
    double x, y;
    VEC2_OR_SEQUENCE(value, x, y, -1)

    self->position->x = x;
    self->position->y = y;
    return 0;
}

static PyObject *
Transform_copy(Transform *self)
{
    return new_Transform(self->position, self->rotation);
}

static PyObject *
Transform_get_rotation(Transform *self, void *closure)
{
    return Mat22_copy(self->rotation);
}

static int
Transform_set_rotation(Transform *self, PyObject *value, void *closure)
{
    return Mat22_set(self->rotation, value) != NULL ? 0 : -1;
}

/* end getters/setters */

/* returns new ref string formatted */
static PyObject *
Transform_repr(Transform *self)
{
    char buf[BUF_SIZE];
    PyOS_snprintf(buf, BUF_SIZE, TRANSFORM_FORMAT, 
            self->position->x, self->position->y,
            atan2(self->rotation->col1->y, self->rotation->col1->x));
    return PyString_FromString(buf);
}

static PyObject *
Transform_mul(Transform *self, PyObject *other)
{
    double v0, v1;
    VEC2_OR_SEQUENCE(other, v0, v1, NULL)
    return new_Vec2(self->position->x + self->rotation->col1->x * v0 + self->rotation->col2->x * v1,
                    self->position->y + self->rotation->col1->y * v0 + self->rotation->col2->y * v1);
}

static PyObject*
Transform_mul_t_vec(Transform *self, PyObject *other) {
    double v0, v1;
    double a11 = self->rotation->col1->x, a12 = self->rotation->col2->x;
    double a21 = self->rotation->col1->y, a22 = self->rotation->col2->y;

    VEC2_OR_SEQUENCE(other, v0, v1, NULL)
    v0 -= self->position->x;
    v1 -= self->position->y;
    return new_Vec2(a11 * v0 + a21 * v1,
                    a12 * v0 + a22 * v1);
}

static PyObject *
Transform_mul_t(Transform *self, PyObject *other)
{
    PyObject* ret=Transform_mul_t_vec(self, other);
    if (!ret) {
        PyErr_Clear();
        if (PyObject_TypeCheck(other, &TransformType)) {
            Transform* _other=(Transform*)other;
            Vec2 new_position;
            Mat22* new_rotation;

            new_position.x = _other->position->x - self->position->x;
            new_position.y = _other->position->y - self->position->y;

            new_rotation=(Mat22*)Mat22_mul_t(self->rotation, (PyObject*)_other->rotation);
            ret = (PyObject*)new_Transform(&new_position, new_rotation);
            Py_DECREF(new_rotation);
        } else {
            PyErr_SetString(PyExc_TypeError, "Expected Vec2 or Transform");
            return NULL;
        }
    }
    return ret;
}

static PyObject *
Transform_richcompare(PyObject *_self, PyObject *_other, int op)
{
    Transform* self;
    Transform* other;
    PyObject* ret;

    if (!PyObject_TypeCheck(_self, &TransformType) || !PyObject_TypeCheck(_other, &TransformType)) {
        PyErr_SetString(PyExc_NotImplementedError, "Can only compare Transform types");
        return NULL;
    }

    self=(Transform*)_self;
    other=(Transform*)_other;

    ret = Vec2_richcompare((PyObject*)self->position, (PyObject*)other->position, op);
    if (ret == Py_True) {
        Py_DECREF(ret);
        ret = Mat22_richcompare((PyObject*)self->rotation, (PyObject*)other->rotation, op);
    }
    return ret;
}


/*** Methods ***/
static PyObject *
Transform_set_identity(Transform *self, Transform* other)
{
    PyObject* ret = Py_True;
    self->rotation->col1->x = 1.0;
    self->rotation->col1->y = 0.0;
    self->rotation->col2->x = 0.0;
    self->rotation->col2->y = 1.0;

    self->position->x = 0.0;
    self->position->y = 0.0;

    Py_INCREF(ret);
    return ret;
}

static PyMethodDef Transform_methods[] = {
    {"__copy__", (PyCFunction)Transform_copy, METH_NOARGS,
     "Copy the transform"
    },
    {"copy", (PyCFunction)Transform_copy, METH_NOARGS,
     "Copy the transform"
    },
    {"set_identity", (PyCFunction)Transform_set_identity, METH_NOARGS,
     "Identity rotation matrix [1 0; 0 1] and zero position"
    },
    {"mul_t", (PyCFunction)Transform_mul_t, METH_O,
       "Transpose multiplication.\n\n"
       "Takes either vector or another transform.\n"
       "Vector: A._rotation^T (vec - A.position)\n"
       "Transform: y2 = A.R' * (B.R * v1 + B.p - A.p) = (A.R' * B.R) * v1 + (B.p - A.p)\n"
    },
    {"mul_tv", (PyCFunction)Transform_mul_t_vec, METH_O,
        "Transpose multiplication with vector.\n\n"
        "Vector: A._rotation^T (vec - A.position)"
    },
    {NULL}  /* Sentinel */
};

/*** End methods ***/

static PyGetSetDef Transform_getseters[] = {
    {"angle", 
     (getter)Transform_get_angle, (setter)Transform_set_angle,
     "Angle in radians",
     NULL},
    {"position", 
     (getter)Transform_get_position, (setter)Transform_set_position,
     "The position (translation)",
     NULL},
    {"rotation", 
     (getter)Transform_get_rotation, (setter)Transform_set_rotation,
     "The 2x2 rotation matrix",
     NULL},
    {NULL}  /* Sentinel */
};

static PyNumberMethods Transform_as_number = {
    0,                          /*nb_add*/
    0,                          /*nb_subtract*/
    (binaryfunc)Transform_mul,  /*nb_multiply*/
#ifndef IS_PY3K
    0,                          /*nb_divide*/
#endif
    0,                          /*nb_remainder*/
    0,                          /*nb_divmod*/
    0,                          /*nb_power*/
    0,                          /*nb_negative*/
    (unaryfunc)Transform_copy,  /*tp_positive*/
    0,                          /*tp_absolute*/
    0,                          /*tp_nonzero*/
    0,                          /*nb_invert*/
    0,                          /*nb_lshift*/
    0,                          /*nb_rshift*/
    0,                          /*nb_and*/
    0,                          /*nb_xor*/
    0,                          /*nb_or*/
#ifndef IS_PY3K
    0,                          /*nb_coerce*/
#endif
    0,                          /*nb_int*/
    0,                          /*nb_long*/
    0,                          /*nb_float*/
#ifndef IS_PY3K
    0,                          /*nb_oct*/
    0,                          /*nb_hex*/
#endif
    0,                          /* nb_inplace_add */
    0,                          /* nb_inplace_subtract */
    0,                          /* nb_inplace_multiply */
#ifndef IS_PY3K
    0,                          /* nb_inplace_divide */
#endif
    0,                          /* nb_inplace_remainder */
    0,                          /* nb_inplace_power */
    0,                          /* nb_inplace_lshift */
    0,                          /* nb_inplace_rshift */
    0,                          /* nb_inplace_and */
    0,                          /* nb_inplace_xor */
    0,                          /* nb_inplace_or */
    0,                          /* nb_floor_divide */
    0,                          /* nb_true_divide */
    0,                          /* nb_inplace_floor_divide */
    0,                          /* nb_inplace_true_divide */
    0,                          /* nb_index */
};

PyTypeObject TransformType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "common.Transform",                                             /*tp_name*/
    sizeof(Transform),                                              /*tp_basicsize*/
    0,                                                              /*tp_itemsize*/
    (destructor)Transform_dealloc,                                  /*tp_dealloc*/
    0,                                                              /*tp_print*/
    0,                                                              /*tp_getattr*/
    0,                                                              /*tp_setattr*/
    0,                                                              /*tp_compare*/
    (reprfunc)Transform_repr,                                       /*tp_repr*/
    &Transform_as_number,                                           /*tp_as_number*/
    0,                                                              /*tp_as_sequence*/
    0,                                                              /*tp_as_mapping*/
    0,                                                              /*tp_hash */
    0,                                                              /*tp_call*/
    0,                                                              /*tp_str*/
    0,                                                              /*tp_getattro*/
    0,                                                              /*tp_setattro*/
    0,                                                              /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_CHECKTYPES,/*tp_flags*/
    "2D Vector",                                                    /* tp_doc */
    0,                                                              /* tp_traverse */
    0,                                                              /* tp_clear */
    (richcmpfunc)Transform_richcompare,                             /* tp_richcompare */
    0,                                                              /* tp_weaklistoffset */
    0,                                                              /* tp_iter */
    0,                                                              /* tp_iternext */
    Transform_methods,                                              /* tp_methods */
    Transform_members,                                              /* tp_members */
    Transform_getseters,                                            /* tp_getset */
    0,                                                              /* tp_base */
    0,                                                              /* tp_dict */
    0,                                                              /* tp_descr_get */
    0,                                                              /* tp_descr_set */
    0,                                                              /* tp_dictoffset */
    (initproc)Transform_init,                                       /* tp_init */
    0,                                                              /* tp_alloc */
    Transform_new,                                                  /* tp_new */
};
