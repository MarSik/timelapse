from scipy import misc
import numpy as np
import sys
import os.path

fname = sys.argv[1]
name = os.path.basename(os.path.splitext(fname)[0])
symbol = name.replace(".", "_").replace("-", "_").replace(" ", "_").upper()

img = misc.imread(fname).transpose()
width, height = img.shape

sys.stderr.write("Size: %d x %d\n" % (width, height))

data = []
multiplier = np.array([2**x for x in xrange(8)])

for segment in xrange(0, height, 8):
    sys.stderr.write("Segment: <%d; %d)\n" % (segment, segment + 8))
    for column in img[:,segment:segment+8]:
        value = column.dot(multiplier)
        data.append(value)

sys.stderr.write("Total size: %dB\n" % len(data))

print """/* Generated from %(filename)s */
#ifndef _IMG_%(symbol)s_H
#define _IMG_%(symbol)s_H

#define %(symbol)s_WIDTH %(width)d
#define %(symbol)s_HEIGHT %(height)d

static const uint8_t %(variable)s_bits[] = {
  %(data)s
};

#endif /* _IMG_%(symbol)s_H */
""" % {
  "symbol": symbol,
  "variable": symbol.lower(),
  "filename": fname,
  "width": width,
  "height": height,
  "data": ", ".join(str(hex(int(x))) for x in data)
}

