#include "common.h"

#define AABB_FORMAT "AABB(lower_bound=(%.2g, %.2g), upper_bound=(%.2g, %.2g))"
#define BUF_SIZE 255

PyTypeObject AABBType;
static PyObject *
AABB_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    AABB *self;

    self = (AABB *)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->lower_bound = (Vec2*)new_Vec2(0.0, 0.0);
        self->upper_bound = (Vec2*)new_Vec2(0.0, 0.0);
    }

    return (PyObject *)self;
}

PyObject *
new_AABB(Vec2* lower_bound, Vec2* upper_bound) {
    AABB* ret=(AABB*)AABB_new(&AABBType, NULL, NULL);
    ret->lower_bound->x = lower_bound->x;
    ret->lower_bound->y = lower_bound->y;

    ret->upper_bound->x = upper_bound->x;
    ret->upper_bound->y = upper_bound->y;
    return (PyObject*)ret;
}

static PyMemberDef AABB_members[] = {
    {"_lower_bound", T_OBJECT_EX, offsetof(AABB, lower_bound), READONLY,
     "The lower bound"},
    {"_upper_bound", T_OBJECT_EX, offsetof(AABB, upper_bound), READONLY,
     "The upper bound"},
    {NULL}  /* Sentinel */
};

static void
AABB_dealloc(AABB* self)
{
    Py_XDECREF(self->lower_bound);
    Py_XDECREF(self->upper_bound);
    Py_TYPE(self)->tp_free((PyObject*)self);
}

/* returns new AABB instance with given values */
static int
AABB_init(AABB *self, PyObject *args, PyObject *kwds)
{
    static char *kwlist[] = {"lower_bound", "upper_bound", NULL};
    PyObject *lower_bound=NULL, *upper_bound=NULL;
    double a, b;
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|OO", kwlist, 
                                      &lower_bound, &upper_bound))
        return -1; 

    if (lower_bound && lower_bound != Py_None) {
        VEC2_OR_SEQUENCE(lower_bound, a, b, -1)
        self->lower_bound->x = a;
        self->lower_bound->y = b;
    }

    if (upper_bound && upper_bound != Py_None) {
        VEC2_OR_SEQUENCE(upper_bound, a, b, -1)
        self->upper_bound->x = a;
        self->upper_bound->y = b;
    }

    return 0;
}

/* getters/setters */
static PyObject *
AABB_get_perimeter(AABB *self, void *closure)
{
    double wx = self->upper_bound->x - self->lower_bound->x;
    double wy = self->upper_bound->y - self->lower_bound->y;
    return PyFloat_FromDouble(2.0 * (wx + wy));
}

static PyObject *
AABB_get_extents(AABB *self, void *closure)
{
    return new_Vec2(0.5 * (self->upper_bound->x - self->lower_bound->x), 
                    0.5 * (self->upper_bound->y - self->lower_bound->y));
}

static PyObject *
AABB_get_center(AABB *self, void *closure)
{
    return new_Vec2(0.5 * (self->upper_bound->x + self->lower_bound->x), 
                    0.5 * (self->upper_bound->y + self->lower_bound->y));
}

static PyObject *
AABB_is_valid(AABB *self, void *closure)
{
    double x=(self->upper_bound->x - self->lower_bound->x);
    double y=(self->upper_bound->y - self->lower_bound->y);
    // TODO: check vectors
    return PyBool_FromLong((x >= 0.0 && y >= 0.0) ? 1 : 0);
}

static PyObject *
AABB_test_overlap(AABB *self, PyObject* other)
{
    double blx, bly, bhx, bhy;
    PyObject* ret=Py_True;
    AABB_OR_TUPLES(other, blx, bly, bhx, bhy, NULL)

    if ((blx - self->upper_bound->x) > 0.0 || (bly - self->upper_bound->y) > 0.0 ||
        (self->lower_bound->x - bhx) > 0.0 || (self->lower_bound->y - bhy) > 0.0)
        ret = Py_False;

    Py_INCREF(ret);
    return ret;
}

static PyObject *
AABB_ray_cast(AABB *self, PyObject* args) {
    PyObject *_p1, *_p2, *normal=NULL;
    double max_fraction;
    double tmin=-FLT_MAX;
    double tmax= FLT_MAX;
    double p1x, p1y;
    double p2x, p2y;
    double d[2];
    double abs_d[2];
    double n[2]; //normal
    double p1[2];
    double p2[2];
    double temp;
    double lower[2] = { self->lower_bound->x, self->lower_bound->y };
    double upper[2] = { self->upper_bound->x, self->upper_bound->y };
    double s, inv_d;
    double t1, t2;
    int i=0;
    if (!PyArg_ParseTuple(args, "OOd:AABB.ray_cast", &_p1, &_p2, &max_fraction)) {
        return NULL;
    }

    // From Real-time Collision Detection, p179.
    VEC2_OR_SEQUENCE(_p1, p1x, p1y, NULL)
    VEC2_OR_SEQUENCE(_p2, p2x, p2y, NULL)
    p1[0] = p1x;
    p1[1] = p1y;

    p2[0] = p2x;
    p2[1] = p2y;

    for (; i < 2; i++) {
        d[i] = p2[i] - p1[i];
        abs_d[i] = fabs(d[i]);

        if (abs_d[i] < FLT_EPSILON) {
            // Parallel.
            if (p1[i] < lower[i] || upper[i] < p1[i]) {
                goto fail;
            }
        } else {
            inv_d = 1.0 / d[i];
            t1 = (lower[i] - p1[i]) * inv_d;
            t2 = (upper[i] - p1[i]) * inv_d;

            // Sign of the normal vector.
            s = -1.0;

            if (t1 > t2) {
                temp = t1;
                t1 = t2;
                t2 = temp;
                s = 1.0;
            }

            // Push the min up
            if (t1 > tmin) {
                n[0] = n[1] = 0.0;
                n[i] = s;
                tmin = t1;
            }

            // Pull the max down
            tmax = MIN(tmax, t2);

            if (tmin > tmax) {
                goto fail;
            }
        }
    }

    // Does the ray start inside the box?
    // Does the ray intersect beyond the max fraction?
    if (tmin < 0.0 || max_fraction < tmin) {
        goto fail;
    }

    // Intersection.
    normal = new_Vec2(n[0], n[1]);
    return Py_BuildValue("bOd", 1, normal, tmin);

fail:
    normal = Py_None;
    Py_XINCREF(normal);
    return Py_BuildValue("bOd", 0, normal, 0.0);
}


static PyObject *
AABB_get_lower_bound(AABB *self, void *closure)
{
    return new_Vec2(self->lower_bound->x, self->lower_bound->y);
}

static int
AABB_set_lower_bound(AABB *self, PyObject* value, void *closure)
{
    double x, y;
    VEC2_OR_SEQUENCE(value, x, y, -1);
    self->lower_bound->x=x;
    self->lower_bound->y=y;
    return 0;
}

static int
AABB_set_upper_bound(AABB *self, PyObject* value, void *closure)
{
    double x, y;
    VEC2_OR_SEQUENCE(value, x, y, -1);
    self->upper_bound->x=x;
    self->upper_bound->y=y;
    return 0;
}

static PyObject *
AABB_get_upper_bound(AABB *self, void *closure)
{
    return new_Vec2(self->upper_bound->x, self->upper_bound->y);
}
/* end getters/setters */

static PyObject *
AABB_add(PyObject *self, PyObject *other)
{
    double alx, aly, ahx, ahy;
    double blx, bly, bhx, bhy;
    Vec2 lower_bound;
    Vec2 upper_bound;
    AABB_OR_TUPLES(self, alx, aly, ahx, ahy, NULL)
    AABB_OR_TUPLES(other, blx, bly, bhx, bhy, NULL)

    lower_bound.x = MIN(alx, blx);
    lower_bound.y = MIN(aly, bly);

    upper_bound.x = MAX(ahx, bhx);
    upper_bound.y = MAX(ahy, bhy);
    return new_AABB(&lower_bound, &upper_bound);
}

static PyObject *
AABB_iadd(AABB *self, PyObject *other)
{
    double blx, bly, bhx, bhy;
    AABB_OR_TUPLES(other, blx, bly, bhx, bhy, NULL)

    self->lower_bound->x = MIN(self->lower_bound->x, blx);
    self->lower_bound->y = MIN(self->lower_bound->y, bly);

    self->upper_bound->x = MAX(self->upper_bound->x, bhx);
    self->upper_bound->y = MAX(self->upper_bound->y, bhy);

    Py_INCREF((PyObject*)self);
    return (PyObject*)self;
}

static PyObject *
AABB_combine_two(AABB *self, PyObject *args)
{
    double alx, aly, ahx, ahy;
    double blx, bly, bhx, bhy;
    PyObject *one, *two;

    if (!PyArg_ParseTuple(args, "OO:AABB.combine_two", &one, &two)) {
        return NULL;
    }

    AABB_OR_TUPLES(one, alx, aly, ahx, ahy, NULL)
    AABB_OR_TUPLES(two, blx, bly, bhx, bhy, NULL)

    self->lower_bound->x = MIN(alx, blx);
    self->lower_bound->y = MIN(aly, bly);

    self->upper_bound->x = MAX(ahx, bhx);
    self->upper_bound->y = MAX(ahy, bhy);

    Py_INCREF((PyObject*)self);
    return (PyObject*)self;
}

static PyObject *
AABB_contains(AABB *self, PyObject *other)
{
    double blx, bly, bhx, bhy;
    AABB_OR_TUPLES(other, blx, bly, bhx, bhy, NULL)

    return PyBool_FromLong(
        self->lower_bound->x <= blx && 
        self->lower_bound->y <= bly && 
        bhx <= self->upper_bound->x && 
        bhy <= self->upper_bound->y);
}

/* returns new ref string formatted */
static PyObject *
AABB_repr(AABB *self)
{
    char buf[BUF_SIZE];
    PyOS_snprintf(buf, BUF_SIZE, AABB_FORMAT, 
            self->lower_bound->x, self->lower_bound->y,
            self->upper_bound->x, self->upper_bound->y);
    return PyString_FromString(buf);
}

static PyObject *
AABB_richcompare(PyObject *_self, PyObject *_other, int op)
{
    AABB* self;
    AABB* other;
    PyObject* ret;

    if (!PyObject_TypeCheck(_self, &AABBType) || !PyObject_TypeCheck(_other, &AABBType)) {
        PyErr_SetString(PyExc_NotImplementedError, "Can only compare AABB types");
        return NULL;
    }

    self=(AABB*)_self;
    other=(AABB*)_other;

    ret = Vec2_richcompare((PyObject*)self->lower_bound, (PyObject*)other->lower_bound, op);
    if (ret == Py_True) {
        Py_DECREF(ret);
        ret = Vec2_richcompare((PyObject*)self->upper_bound, (PyObject*)other->upper_bound, op);
    }
    return ret;
}

/*** Sequence stuff ***/
static Py_ssize_t
AABB_seq_length(AABB *self)
{
    return 2;
}

static PyObject *
AABB_item(AABB *self, Py_ssize_t i)
{
    if (i == 0) {
        Py_XINCREF(self->lower_bound);
        return (PyObject*)self->lower_bound;
    } else if (i == 1) {
        Py_XINCREF(self->upper_bound);
        return (PyObject*)self->upper_bound;
    } else {
        PyErr_SetString(PyExc_IndexError, "Expected index in (0, 1)");
        return NULL;
    }
}


static int
AABB_ass_item(AABB *self, Py_ssize_t i, PyObject *_value)
{
    double x, y;
    VEC2_OR_SEQUENCE(_value, x, y, -1)

    if (i == 0) {
        self->lower_bound->x = x;
        self->lower_bound->y = y;
    } else if (i == 1) {
        self->upper_bound->x = x;
        self->upper_bound->y = y;
    } else {
        PyErr_SetString(PyExc_IndexError, "Expected index in (0, 1)");
        return -1;
    }
    return 0;
}

static int
AABB_seq_contains(AABB *self, PyObject *_value)
{
    double x, y;
    int is_float;
    VEC2_OR_SEQUENCE_OR_FLOAT(_value, x, y, is_float, -1)
    if (is_float) {
        return (self->lower_bound->x == x ||
                self->lower_bound->y == x ||
                self->upper_bound->x == x ||
                self->upper_bound->y == x) ? 1 : 0;

    } else {
        return ((self->lower_bound->x == x &&
                 self->lower_bound->y == y) ||
                (self->upper_bound->x == x &&
                 self->upper_bound->y == y)) ? 1 : 0;
    }
}

static PySequenceMethods AABB_as_sequence = {
    (lenfunc)AABB_seq_length,               /* sq_length */
    0,                                      /* sq_concat */
    0,                                      /* sq_repeat */
    (ssizeargfunc)AABB_item,                /* sq_item */
    0,                                      /* sq_slice */
    (ssizeobjargproc)AABB_ass_item,         /* sq_ass_item */
    0,                                      /* sq_ass_slice */
    (objobjproc)AABB_seq_contains,          /* sq_contains */
    0,                                      /* sq_inplace_concat */
    0,                                      /* sq_inplace_repeat */
};
/*** End sequence stuff ***/

/*** Methods ***/
static PyObject *
AABB_copy(AABB *self)
{
    return new_AABB(self->lower_bound, self->upper_bound);
}

static PyMethodDef AABB_methods[] = {
    {"__copy__", (PyCFunction)AABB_copy, METH_NOARGS,
     "Copy the AABB"
    },
    {"copy", (PyCFunction)AABB_copy, METH_NOARGS,
     "Copy the AABB"
    },
    {"combine_two", (PyCFunction)AABB_combine_two, METH_VARARGS,
     "Combine two AABBs into this one, ignoring the current AABB."
    },
    {"combine", (PyCFunction)AABB_iadd, METH_O,
     "Combine an AABB into this one. (or use +=)"
    },
    {"contains", (PyCFunction)AABB_contains, METH_O,
     "Does this aabb contain the provided AABB?"
    },
    {"test_overlap", (PyCFunction)AABB_test_overlap, METH_O,
     "Does this aabb overlap with the provided AABB?"
    },
    {"ray_cast", (PyCFunction)AABB_ray_cast, METH_VARARGS,
     "Cast a ray against this AABB\n"
     "Returns: (hit, normal, fraction)\n"
     "@param p1 point 1\n"
     "@param p2 point 2\n"
     "@param max_fraction maximum fraction\n"
    },
    {NULL}  /* Sentinel */
};

/*** End methods ***/

static PyGetSetDef AABB_getseters[] = {
    {"lower_bound", 
     (getter)AABB_get_lower_bound, (setter)AABB_set_lower_bound,
     "Get/set the lower_bound",
     NULL},
    {"upper_bound", 
     (getter)AABB_get_upper_bound, (setter)AABB_set_upper_bound,
     "Get/set the upper_bound",
     NULL},
    {"perimeter", 
     (getter)AABB_get_perimeter, NULL,
     "Get the perimeter length",
     NULL},
    {"extents", 
     (getter)AABB_get_extents, NULL,
     "Get the extents of the AABB (half-widths)",
     NULL},
    {"center", 
     (getter)AABB_get_center, NULL,
     "Get the center of the AABB.",
     NULL},
    {"valid", 
     (getter)AABB_is_valid, NULL,
     "Is this a valid AABB?",
     NULL},
    {NULL}  /* Sentinel */
};

static PyNumberMethods AABB_as_number = {
    (binaryfunc)AABB_add,       /*nb_add*/
    0,                          /*nb_subtract*/
    0,                          /*nb_multiply*/
#ifndef IS_PY3K
    0,                          /*nb_divide*/
#endif
    0,                          /*nb_remainder*/
    0,                          /*nb_divmod*/
    0,                          /*nb_power*/
    0,                          /*nb_negative*/
    (unaryfunc)AABB_copy,       /*tp_positive*/
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
    (binaryfunc)AABB_iadd,      /* nb_inplace_add */
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

PyTypeObject AABBType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "common.AABB",                                           /*tp_name*/
    sizeof(AABB),                                            /*tp_basicsize*/
    0,                                                       /*tp_itemsize*/
    (destructor)AABB_dealloc,                                /*tp_dealloc*/
    0,                                                       /*tp_print*/
    0,                                                       /*tp_getattr*/
    0,                                                       /*tp_setattr*/
    0,                                                       /*tp_compare*/
    (reprfunc)AABB_repr,                                     /*tp_repr*/
    &AABB_as_number,                                         /*tp_as_number*/
    &AABB_as_sequence,                                       /*tp_as_sequence*/
    0,                                                       /*tp_as_mapping*/
    0,                                                       /*tp_hash */
    0,                                                       /*tp_call*/
    0,                                                       /*tp_str*/
    0,                                                       /*tp_getattro*/
    0,                                                       /*tp_setattro*/
    0,                                                       /*tp_as_buffer*/
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_CHECKTYPES,/*tp_flags*/
    "2D Vector",                                             /* tp_doc */
    0,                                                       /* tp_traverse */
    0,                                                       /* tp_clear */
    (richcmpfunc)AABB_richcompare,                           /* tp_richcompare */
    0,                                                       /* tp_weaklistoffset */
    0,                                                       /* tp_iter */
    0,                                                       /* tp_iternext */
    AABB_methods,                                            /* tp_methods */
    AABB_members,                                            /* tp_members */
    AABB_getseters,                                          /* tp_getset */
    0,                                                       /* tp_base */
    0,                                                       /* tp_dict */
    0,                                                       /* tp_descr_get */
    0,                                                       /* tp_descr_set */
    0,                                                       /* tp_dictoffset */
    (initproc)AABB_init,                                     /* tp_init */
    0,                                                       /* tp_alloc */
    AABB_new,                                                /* tp_new */
};
