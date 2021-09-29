from building import *

cwd = GetCurrentDir()
path = [cwd+'/inc']
src  = Glob('src/*.c')
 
group = DefineGroup('soft_serial', src, depend = ['PKG_USING_SOFT_SERIAL'], CPPPATH = path)

Return('group')