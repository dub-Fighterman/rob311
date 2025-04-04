# This file was automatically generated by SWIG (http://www.swig.org).
# Version 4.0.2
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info
if _swig_python_version_info < (2, 7, 0):
    raise RuntimeError("Python 2.7 or later required")

# Import the low-level C/C++ module
if __package__ or "." in __name__:
    from . import _FIR
else:
    import _FIR

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)


def _swig_setattr_nondynamic_instance_variable(set):
    def set_instance_attr(self, name, value):
        if name == "thisown":
            self.this.own(value)
        elif name == "this":
            set(self, name, value)
        elif hasattr(self, name) and isinstance(getattr(type(self), name), property):
            set(self, name, value)
        else:
            raise AttributeError("You cannot add instance attributes to %s" % self)
    return set_instance_attr


def _swig_setattr_nondynamic_class_variable(set):
    def set_class_attr(cls, name, value):
        if hasattr(cls, name) and not isinstance(getattr(cls, name), property):
            set(cls, name, value)
        else:
            raise AttributeError("You cannot add class attributes to %s" % cls)
    return set_class_attr


def _swig_add_metaclass(metaclass):
    """Class decorator for adding a metaclass to a SWIG wrapped class - a slimmed down version of six.add_metaclass"""
    def wrapper(cls):
        return metaclass(cls.__name__, cls.__bases__, cls.__dict__.copy())
    return wrapper


class _SwigNonDynamicMeta(type):
    """Meta class to enforce nondynamic attributes (no new attributes) for a class"""
    __setattr__ = _swig_setattr_nondynamic_class_variable(type.__setattr__)


class FIR(object):
    thisown = property(lambda x: x.this.own(), lambda x, v: x.this.own(v), doc="The membership flag")
    __repr__ = _swig_repr

    def __init__(self):
        _FIR.FIR_swiginit(self, _FIR.new_FIR())

    def getTaps(self):
        return _FIR.FIR_getTaps(self)

    def getCoeff(self, i):
        return _FIR.FIR_getCoeff(self, i)

    def getType(self):
        return _FIR.FIR_getType(self)

    def getFreq(self, i):
        return _FIR.FIR_getFreq(self, i)

    def getBuffer(self, i):
        return _FIR.FIR_getBuffer(self, i)

    def setTaps(self, M):
        return _FIR.FIR_setTaps(self, M)

    def setCoeff(self, x, i):
        return _FIR.FIR_setCoeff(self, x, i)

    def setType(self, a):
        return _FIR.FIR_setType(self, a)

    def setFreq(self, x, i):
        return _FIR.FIR_setFreq(self, x, i)

    def setBuffer(self, x, i):
        return _FIR.FIR_setBuffer(self, x, i)

    def lowpass(self, M, f):
        return _FIR.FIR_lowpass(self, M, f)

    def highpass(self, M, f):
        return _FIR.FIR_highpass(self, M, f)

    def stopband(self, M, f1, f2):
        return _FIR.FIR_stopband(self, M, f1, f2)

    def passband(self, M, f1, f2):
        return _FIR.FIR_passband(self, M, f1, f2)

    def filter(self, x):
        return _FIR.FIR_filter(self, x)
    __swig_destroy__ = _FIR.delete_FIR

# Register FIR in _FIR:
_FIR.FIR_swigregister(FIR)



