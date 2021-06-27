import numpy
import ctypes

name = "FORCESNLPsolver_normal"
requires_callback = True
lib = "lib/libFORCESNLPsolver_normal.so"
lib_static = "lib/libFORCESNLPsolver_normal.a"
c_header = "include/FORCESNLPsolver_normal.h"

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  9,   1),    9),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (340,   1),  340),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (2600,   1), 2600),
 ("num_of_threads"      , ""      , "solver_int32_unsigned", ctypes.c_uint  , numpy.uint32 , (  0,   1),    1)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x11"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x12"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x13"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x14"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x15"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x16"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x17"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x18"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x19"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17),
 ("x20"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 17,),   17)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
('it2opt', ctypes.c_int),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('rsnorm', ctypes.c_double),
('rcompnorm', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int),
('lsit_cc', ctypes.c_int),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double),
('fevalstime', ctypes.c_double)
]