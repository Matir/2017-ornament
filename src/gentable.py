import math
import sys

gamma = 2.5;

template = '''
#include <stdint.h>
#include <avr/pgmspace.h>

const uint8_t consts_num_steps = %(nsteps)d;
const uint8_t gamma_table[] PROGMEM = {%(gamma_table)s};
'''

nsteps = int(sys.argv[1])
outfile = sys.argv[2]

# Gamma adjustment table per Adafruit
gamma_vals = [int(math.pow(i/256.0, gamma) * 256.0 + 0.5) for i in xrange(256)]

# Render them
templ = {}
templ['nsteps'] = nsteps
templ['gamma_table'] = ', '.join('%d' % x for x in gamma_vals)

with open(outfile, 'w') as fp:
    fp.write(template % templ)
