##### El nombre del archivo de es lo unico que se debe cambiar para otros ejemplos
from __future__ import print_function
from yade import pack
import gts, os.path, locale

locale.setlocale(
        locale.LC_ALL, 'en_US.UTF-8'
)  #gts is locale-dependend.  If, for example, german locale is used, gts.read()-function does not import floats normally
###################################################################

if not os.path.exists('ejemplogts2.coarse.gts'):
    if os.path.exists('ejemplogts2.gts'):
        surf = gts.read(open('ejemplogts2.gts'))
        surf.coarsen(100)
        surf.write(open('ejemplogts2.coarse.gts', 'w'))
    else:
        print(
                """ejemplogts2.gts not found, you need to download input data:

        wget http://gts.sourceforge.net/samples/ejemplogts2.gts.gz
        gunzip ejemplogts2.gts.gz
        """
        )
        quit()

surf = gts.read(open('ejemplogts2.coarse.gts'))
O.bodies.append(pack.gtsSurface2Facets(surf, wire=True))