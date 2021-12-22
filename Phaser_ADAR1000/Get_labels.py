import sys
sys.path.append('/usr/lib/python2.7/site-packages/')
import iio
for ctxname in iio.scan_contexts():
    ctx = iio.Context(ctxname)
    for dev in ctx.devices:
        if dev.name == 'adar1000':
            if dev.attrs['label'].value == 'BEAM3':
                Beam3=dev
            if dev.attrs['label'].value == 'BEAM2':
                Beam2=dev
            if dev.attrs['label'].value == 'BEAM1':
                Beam1=dev
            if dev.attrs['label'].value == 'BEAM0':
                Beam0=dev
print(dev)
