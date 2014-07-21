from __future__ import division

from txros import util

import sub_scripting


@util.cancellableInlineCallbacks
def main(nh):
    sub = yield sub_scripting.get_sub(nh)
    
    yield sub.raise_impaler()
    #yield util.sleep(2)
    #yield sub.lower_impaler()
    #yield util.sleep(2)
    #yield sub.raise_impaler()
    yield sub.contract_impaler()
#    for i in xrange(10):
#        yield sub.contract_impaler()
